/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/agv/debug/debug_Planner.hpp>
#include <rmf_traffic/agv/RouteValidator.hpp>

#include <rmf_traffic/geometry/Circle.hpp>

#include <rmf_traffic/schedule/Database.hpp>
#include <rmf_traffic/schedule/Participant.hpp>

#include <rmf_fleet_adapter/agv/parse_graph.hpp>

#include <iostream>

#include <yaml-cpp/yaml.h>

const std::size_t NotObstacleID = std::numeric_limits<std::size_t>::max();

struct Plan
{
  std::size_t initial_time;
  double initial_orientation;
  std::string initial_waypoint, goal;
};

struct Scenario
{
  std::string map_file;
  std::string main_robot;
  std::size_t samples;

  std::unordered_map<std::string, rmf_traffic::agv::VehicleTraits> robots;
  std::unordered_map<std::string, Plan> plans;
};

void parse_scenario(std::string scenario_file, Scenario& scenario)
{
  if (scenario_file.rfind(".yaml") == std::string::npos)
  {
    scenario_file.append(".yaml");
  }

  YAML::Node scenario_config;

  try
  {
    scenario_config = YAML::LoadFile(std::string(
          TEST_SCENARIO_DIR) + scenario_file);
  }
  catch (YAML::BadFile& e)
  {
    throw std::runtime_error(
            "Failed to load scenario file [" + scenario_file + "]");
  }

  if (scenario_config["map"])
  {
    scenario.map_file = scenario_config["map"].as<std::string>();
  }
  else
  {
    throw std::runtime_error("Scenario file is missing the [map] key");
  }

  if (scenario_config["main_robot"])
  {
    scenario.main_robot = scenario_config["main_robot"].as<std::string>();
  }
  else
  {
    throw std::runtime_error("Scenario file is missing the [main_robot] key");
  }

  if (scenario_config["samples"])
  {
    scenario.samples = scenario_config["samples"].as<std::size_t>(100);
  }
  else
  {
    std::cout <<
      "Scenario file is missing the [samples] key. Using default value [100]" <<
      std::endl;
  }

  const YAML::Node robots = scenario_config["robots"];

  for (const auto& robot : robots)
  {
    if (!robot["robot"])
    {
      std::cout << "Missing [robot] key. Skipping entry" << std::endl;
      continue;
    }

    const std::string& name = robot["robot"].as<std::string>();

    double linear_velocity, linear_acceleration, angular_velocity,
      angular_acceleration;

    const auto& traits = robot["traits"];
    if (traits)
    {
      const auto& linear = traits["linear"];
      const auto& angular = traits["angular"];

      if (linear)
      {
        const auto& velocity = linear["velocity"];
        const auto& acceleration = linear["acceleration"];

        if (velocity)
        {
          linear_velocity = velocity.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [traits[linear[velocity]]]. Skipping entry" <<
            std::endl;
          continue;
        }

        if (acceleration)
        {
          linear_acceleration = acceleration.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [traits[linear[accleration]]]. Skipping entry" <<
            std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [traits[linear]]. Skipping entry" << std::endl;
      }

      if (angular)
      {
        const auto& velocity = angular["velocity"];
        const auto& acceleration = angular["acceleration"];

        if (velocity)
        {
          angular_velocity = velocity.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [traits[angular[velocity]]]. Skipping entry" <<
            std::endl;
          continue;
        }

        if (acceleration)
        {
          angular_acceleration = acceleration.as<double>();
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [traits[angular[accleration]]]. Skipping entry" <<
            std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [traits[angular]]. Skipping entry" << std::endl;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
        "] is missing key [traits]. Skipping entry" << std::endl;
      continue;
    }

    const auto& profile = robot["profile"];
    if (profile)
    {
      const auto& footprint = profile["footprint"];
      if (footprint)
      {
        const auto& shape = footprint["shape"];
        if (shape)
        {
          if (strcasecmp("circle", shape.as<std::string>().c_str()) == 0)
          {
            const auto& radius = footprint["radius"];
            if (radius)
            {
              scenario.robots.insert({
                  name,
                  rmf_traffic::agv::VehicleTraits {
                    {linear_velocity, linear_acceleration},
                    {angular_velocity, angular_acceleration},
                    rmf_traffic::Profile{
                      rmf_traffic::geometry::make_final_convex(
                        rmf_traffic::geometry::Circle(radius.as<double>()))
                    }}});
            }
            else
            {
              std::cout << "Robot [" << name <<
                "] is missing key [profile[footprint[radius]]]. Skipping entry"
                        << std::endl;
              continue;
            }
          }
          else
          {
            std::cout << "Robot [" << name <<
              "] has unsupported shape value [" << shape.as<std::string>() <<
              "]. Skipping entry" << std::endl;
            continue;
          }
        }
        else
        {
          std::cout << "Robot [" << name <<
            "] is missing key [profile[footprint[shape]]]. Skipping entry" <<
            std::endl;
          continue;
        }
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [profile[footprint]]. Skipping entry" << std::endl;
        continue;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
        "] is missing key [profile]. Skipping entry" << std::endl;
      continue;
    }
  }

  const YAML::Node plans = scenario_config["plans"];
  for (const auto& plan : plans)
  {
    if (!plan["robot"])
    {
      std::cout << "Missing [robot] key. Skipping entry" << std::endl;
      continue;
    }

    const std::string& name = plan["robot"].as<std::string>();

    std::size_t initial_time;
    double initial_orientation;
    std::string initial_waypoint;

    const auto& start = plan["start"];
    if (start)
    {
      const auto& time = start["initial_time"];
      if (time)
      {
        initial_time = time.as<std::size_t>(0);
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [start[initial_time]]. Using default value [0]." <<
          std::endl;
      }
      const auto& waypoint = start["initial_waypoint"];
      if (waypoint)
      {
        initial_waypoint = waypoint.as<std::string>();
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [start[initial_waypoint]]. Skipping entry." <<
          std::endl;
        continue;
      }
      const auto& orientation = start["initial_orientation"];
      if (orientation)
      {
        initial_orientation = waypoint.as<double>(0);
      }
      else
      {
        std::cout << "Robot [" << name <<
          "] is missing key [start[initial_orientation]]. Assuming initial_orientation 0."
                  << std::endl;
      }
    }
    else
    {
      std::cout << "Robot [" << name <<
        "] is missing key [start]. Skipping entry." << std::endl;
      continue;
    }

    const auto& goal = plan["goal"];
    if (goal)
    {
      scenario.plans.insert({name, {initial_time, initial_orientation,
            initial_waypoint, goal.as<std::string>()}});
    }
    else
    {
      std::cout << "Robot [" << name <<
        "] is missing key [goal]. Skipping entry." << std::endl;
      continue;
    }
  }
}

rmf_traffic::schedule::Participant add_obstacle(
  const rmf_traffic::agv::Planner& planner,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  const auto N = database->participant_ids().size();

  auto new_obstacle = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "obstacle_" + std::to_string(N),
      "obstacles",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      planner.get_configuration().vehicle_traits().profile()
    }, database);

  const auto plan = planner.plan(start, goal);
  new_obstacle.set(plan->get_itinerary());
  return new_obstacle;
}

void print_result(
  const std::string& label,
  const std::size_t samples,
  const double total_time,
  const std::size_t node_count)
{
  std::cout << label
            << "\n -- Total time for " << samples << " samples: "
            << total_time
            << "\n -- Average time per run: " << total_time/samples
            << "\n -- Node count: " << node_count
            << "\n" << std::endl;
}

double test_planner_timing_no_cache(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  // Run a test where we produce a new planner every time so we see what the
  // timing is if the cache is blank
  const auto begin_time = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < samples; ++i)
  {
    rmf_traffic::agv::Planner planner(config, options);
    planner.plan(start, goal);
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const double total_time = rmf_traffic::time::to_seconds(
    finish_time - begin_time);

  const auto nodes = rmf_traffic::agv::Planner::Debug::node_count(
    rmf_traffic::agv::Planner(config, options).plan(start, goal));

  print_result(label + " | No Cache", samples, total_time, nodes);

  return total_time;
}

double test_planner_timing_with_cache(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  // Run a test where we prime the planner by solving it once. Future runs will
  // not need to recompute the heuristic.
  rmf_traffic::agv::Planner planner(config, options);
  planner.plan(start, goal);

  const auto begin_time = std::chrono::steady_clock::now();
  for (std::size_t i = 0; i < samples; ++i)
  {
    planner.plan(start, goal);
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const double total_time = rmf_traffic::time::to_seconds(
    finish_time - begin_time);

  const auto nodes = rmf_traffic::agv::Planner::Debug::node_count(
    rmf_traffic::agv::Planner(config, options).plan(start, goal));

  print_result(label + " | With Cache", samples, total_time, nodes);

  return total_time;
}

void test_planner_timing(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  std::cout << " --------- \n" << std::endl;

  // For each variation of test, we run many samples and then see what the
  // average time is.
  const double no_cache_time = test_planner_timing_no_cache(
    label, samples, config, options, start, goal);

  const double with_cache_time = test_planner_timing_with_cache(
    label, samples, config, options, start, goal);

  std::cout << "Cache speed boost: x" << no_cache_time/with_cache_time
            << "\n" << std::endl;
}

void test_planner(
  const std::string& label,
  const std::size_t samples,
  const rmf_traffic::agv::Graph& graph,
  const rmf_traffic::agv::VehicleTraits& traits,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  test_planner_timing(
    label + " | No Obstacles",
    samples,
    {graph, traits},
    {nullptr},
    start, goal
  );

  const auto obstacle_validator =
    rmf_traffic::agv::ScheduleRouteValidator::make(
    database, NotObstacleID, traits.profile());

  test_planner_timing(
    label + " | With Obstacles",
    samples,
    {graph, traits},
    {obstacle_validator},
    start, goal
  );
}

int main(int argc, char* argv[])
{
  if (argc < 2)
  {
    std::cout << "Please provide scenario file name" << std::endl;
    return 0;
  }

  Scenario scenario;
  try
  {
    parse_scenario(argv[1], scenario);
  }
  catch (std::runtime_error e)
  {
    std::cout << e.what() << std::endl;
    return 0;
  }

  using namespace std::chrono_literals;

  const std::string map_file = std::string(TEST_MAP_DIR) + scenario.map_file;
  std::cout << "Loading [" << map_file << "]" << std::endl;

  const auto& main_robot = scenario.robots.find(scenario.main_robot);
  if (main_robot == scenario.robots.end())
  {
    std::cout << "Main robot [" << scenario.main_robot <<
      "]'s traits and profile missing" << std::endl;
    return 0;
  }

  rmf_traffic::agv::Graph graph;
  try
  {
    graph = rmf_fleet_adapter::agv::parse_graph(map_file, main_robot->second);
  }
  catch (YAML::BadFile& e)
  {
    std::cout << "Failed to load map file [" << map_file << "]" << std::endl;
    return 0;
  }

  const auto get_wp = [&](const std::string& name)
    {
      return graph.find_waypoint(name)->index();
    };

  const auto start_time = std::chrono::steady_clock::now();

  // We'll make some "obstacles" in the environment by planning routes between
  // various waypoints.
  rmf_traffic::agv::Planner planner{
    {graph, main_robot->second},
    {nullptr}
  };

  const auto database = std::make_shared<rmf_traffic::schedule::Database>();
  std::vector<rmf_traffic::schedule::Participant> obstacles;

  for (const auto& plan : scenario.plans)
  {
    if (std::strcmp(plan.first.c_str(), scenario.main_robot.c_str()) == 0)
    {
      continue;
    }

    const auto& robot = scenario.robots.find(plan.first);

    if (robot == scenario.robots.end())
    {
      std::cout << "Robot [" << plan.first <<
        "] is missing traits and profile. Using traits and profile of main_robot."
                << std::endl;
      rmf_traffic::agv::Planner planner{
        {graph, main_robot->second},
        {nullptr}
      };
    }
    else
    {
      rmf_traffic::agv::Planner planner{
        {graph, robot->second},
        {nullptr}
      };
    }

    obstacles.emplace_back(
      add_obstacle(
        planner, database,
        {
          start_time + std::chrono::seconds(plan.second.initial_time),
          get_wp(plan.second.initial_waypoint),
          plan.second.initial_orientation * M_PI / 180.0
        },
        get_wp(plan.second.goal)
      )
    );
  }

  const auto& plan = scenario.plans.find(scenario.main_robot);
  if (plan == scenario.plans.end())
  {
    std::cout << "Main robot [" << scenario.main_robot << "]'s plan missing" <<
      std::endl;
    return 0;
  }

  test_planner(
    scenario.map_file + " " + plan->second.initial_waypoint + " -> " + plan->second.goal,
    scenario.samples,
    graph, main_robot->second, database,
    {
      start_time + std::chrono::seconds(plan->second.initial_time),
      get_wp(plan->second.initial_waypoint), plan->second.initial_orientation
    },
    get_wp(plan->second.goal)
  );
}
