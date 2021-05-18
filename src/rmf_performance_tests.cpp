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

#include "rmf_performance_tests/rmf_performance_tests.hpp"
#include <rmf_traffic/agv/debug/debug_Planner.hpp>

#include <iostream>

namespace rmf_performance_tests {

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

rmf_traffic::schedule::Participant add_obstacle(
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::Profile& profile,
  const rmf_traffic::Route& route)
{
  const auto N = database->participant_ids().size();

  auto new_obstacle = rmf_traffic::schedule::make_participant(
    rmf_traffic::schedule::ParticipantDescription{
      "obstacle_" + std::to_string(N),
      "obstacles",
      rmf_traffic::schedule::ParticipantDescription::Rx::Unresponsive,
      profile
    }, database);

  new_obstacle.set({route});
  return new_obstacle;
}

void print_result(
  const std::string& label,
  const std::size_t& samples,
  const double& total_time,
  const std::size_t& node_count)
{
  std::cout << label
            << "\n -- Total time for " << samples << " samples: "
            << total_time
            << "\n -- Average time per run: " << total_time / samples
            << "\n -- Node count: " << node_count
            << "\n" << std::endl;
}

double test_planner_timing_no_cache(
  const std::string& label,
  const std::size_t& samples,
  const std::optional<rmf_traffic::Duration>& max_duration,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal)
{
  // Run a test where we produce a new planner every time so we see what the
  // timing is if the cache is blank
  std::optional<rmf_traffic::agv::Plan> plan;
  const auto begin_time = std::chrono::steady_clock::now();
  std::size_t i;
  for (i = 0; i < samples; ++i)
  {
    rmf_traffic::agv::Planner planner(config, options);
    plan = *planner.plan(start, goal);

    const auto finish_time = std::chrono::steady_clock::now();
    if (max_duration.has_value() && finish_time - begin_time > max_duration)
    {
      i = i + 1;
      std::cout << "Aborting as maximum allowed time elapsed!" << std::endl;
      break;
    }
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const auto& graph = config.graph();
  std::cout << "Solution: ";
  for (const auto& wp : plan->get_waypoints())
  {
    std::cout << rmf_traffic::time::to_seconds(wp.time().time_since_epoch())
              << ": ";
    if (wp.graph_index().has_value())
    {
      const auto index = wp.graph_index().value();
      const auto& gwp = graph.get_waypoint(index);
      if (gwp.name())
        std::cout << *gwp.name();
      else
        std::cout << index;
    }
    else
    {
    }

    const auto& p = wp.position();
    std::cout << " (" << p[0] << ", " << p[1] << ")";

    std::cout << " -> ";
  }

  std::cout << "Done" << std::endl;

  const double total_time = rmf_traffic::time::to_seconds(
    finish_time - begin_time);

  const auto nodes = rmf_traffic::agv::Planner::Debug::node_count(
    rmf_traffic::agv::Planner(config, options).plan(start, goal));

  print_result(label + " | No Cache", i, total_time, nodes);

  return total_time;
}

double test_planner_timing_with_cache(
  const std::string& label,
  const std::size_t& samples,
  const std::optional<rmf_traffic::Duration>& max_duration,
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
  std::size_t i;
  for (i = 0; i < samples; ++i)
  {
    planner.plan(start, goal);
    const auto finish_time = std::chrono::steady_clock::now();
    if (max_duration.has_value() && finish_time - begin_time > max_duration)
    {
      i = i + 1;
      std::cout << "Aborting as maximum allowed time elapsed!" << std::endl;
      break;
    }
  }
  const auto finish_time = std::chrono::steady_clock::now();

  const double total_time = rmf_traffic::time::to_seconds(
    finish_time - begin_time);

  const auto nodes = rmf_traffic::agv::Planner::Debug::node_count(
    rmf_traffic::agv::Planner(config, options).plan(start, goal));

  print_result(label + " | With Cache", i, total_time, nodes);

  return total_time;
}

void test_planner_timing(
  const std::string& label,
  const std::size_t& samples,
  const std::optional<rmf_traffic::Duration>& max_duration,
  const rmf_traffic::agv::Planner::Configuration& config,
  const rmf_traffic::agv::Planner::Options& options,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal,
  const bool& include_cache_tests,
  const bool& include_no_cache_tests)
{
  std::cout << " --------- \n" << std::endl;

  // For each variation of test, we run many samples and then see what the
  // average time is.
  double no_cache_time;
  if (include_no_cache_tests)
  {
    no_cache_time = test_planner_timing_no_cache(
      label, samples, max_duration, config, options, start, goal);
  }

  double with_cache_time;
  if (include_cache_tests)
  {
    with_cache_time = test_planner_timing_with_cache(
      label, samples, max_duration, config, options, start, goal);
  }

  if (include_no_cache_tests && include_cache_tests)
  {
    std::cout << "Cache speed boost: x" << no_cache_time / with_cache_time
              << "\n" << std::endl;
  }
}

void test_planner(
  const std::string& label,
  const std::size_t& samples,
  const std::optional<rmf_traffic::Duration>& max_duration,
  const rmf_traffic::agv::Graph& graph,
  const rmf_traffic::agv::VehicleTraits& traits,
  const std::shared_ptr<rmf_traffic::schedule::Database>& database,
  const rmf_traffic::agv::Plan::Start& start,
  const rmf_traffic::agv::Plan::Goal& goal,
  const bool& include_obstacle_tests,
  const bool& include_no_obstacle_tests,
  const bool& include_cache_tests,
  const bool& include_no_cache_tests)
{
  if (include_no_obstacle_tests)
  {
    test_planner_timing(
      label + " | No Obstacles",
      samples,
      max_duration,
      {graph, traits},
      {nullptr},
      start, goal,
      include_cache_tests,
      include_no_cache_tests
    );
  }

  if (include_obstacle_tests)
  {
    const auto obstacle_validator =
      rmf_traffic::agv::ScheduleRouteValidator::make(
      database, NotObstacleID, traits.profile());

    test_planner_timing(
      label + " | With Obstacles",
      samples,
      max_duration,
      {graph, traits},
      {obstacle_validator},
      start, goal,
      include_cache_tests,
      include_no_cache_tests
    );
  }
}

std::string get_map_directory()
{
  return TEST_MAP_DIR;
}

}
