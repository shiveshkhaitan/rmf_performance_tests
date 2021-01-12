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

const std::size_t NotObstacleID = std::numeric_limits<std::size_t>::max();

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
  const double total_time)
{
  std::cout << label
            << "\n -- Total time for " << samples << " samples: "
            << total_time
            << "\n -- Average time per run: " << total_time/samples
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
  for (std::size_t i=0; i < samples; ++i)
  {
    rmf_traffic::agv::Planner planner(config, options);
    planner.plan(start, goal);
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const double total_time = rmf_traffic::time::to_seconds(
        finish_time - begin_time);

  print_result(label + " | No Cache", samples, total_time);

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
  rmf_traffic::agv::Planner planner(config,  options);
  planner.plan(start, goal);

  const auto begin_time = std::chrono::steady_clock::now();
  for (std::size_t i=0; i < samples; ++i)
  {
    planner.plan(start, goal);
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const double total_time = rmf_traffic::time::to_seconds(
        finish_time - begin_time);

  print_result(label + " | With Cache", samples, total_time);

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

int main()
{
  using namespace std::chrono_literals;

  rmf_traffic::agv::VehicleTraits traits{
    {0.5, 2.0}, {0.75, 1.5},
    rmf_traffic::Profile{
      rmf_traffic::geometry::make_final_convex(
            rmf_traffic::geometry::Circle(0.2))
    }
  };

  const std::string office_map_file = TEST_MAP_DIR"/office/nav_graphs/0.yaml";
  std::cout << "Loading [" << office_map_file << "]" << std::endl;
  const auto office_graph =
      rmf_fleet_adapter::agv::parse_graph(office_map_file, traits);

  const auto get_wp = [&](const std::string& name)
  {
    return office_graph.find_waypoint(name)->index();
  };

  const auto start_time = std::chrono::steady_clock::now();

  // We'll make some "obstacles" in the environment by planning routes between
  // various waypoints.
  rmf_traffic::agv::Planner planner{
    {office_graph, traits},
    {nullptr}
  };

  const auto database = std::make_shared<rmf_traffic::schedule::Database>();
  std::vector<rmf_traffic::schedule::Participant> obstacles;

  obstacles.emplace_back(
    add_obstacle(
      planner, database,
      {start_time, get_wp("tinyRobot2_charger"), 90.0*M_PI/180.0},
      get_wp("lounge")
    )
  );

  obstacles.emplace_back(
    add_obstacle(
      planner, database,
      {start_time + 30s, get_wp("supplies"), 0.0},
      get_wp("tinyRobot2_charger")
    )
  );

  test_planner(
    "Office hardware_2 -> supplies", 100,
    office_graph, traits, database,
    {start_time, get_wp("hardware_2"), 0.0}, get_wp("supplies")
  );
}
