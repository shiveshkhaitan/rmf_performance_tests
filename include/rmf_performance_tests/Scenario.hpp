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

#ifndef RMF_PERFORMANCE_TESTS_SCENARIO_HPP
#define RMF_PERFORMANCE_TESTS_SCENARIO_HPP

#include <rmf_traffic/agv/Planner.hpp>

#include <yaml-cpp/yaml.h>

namespace rmf_performance_tests {
namespace scenario {

struct Plan
{
  std::string robot;
  std::size_t initial_time;
  double initial_orientation;
  std::string initial_waypoint;
  std::string goal;
};

struct Route
{
  std::string robot;
  rmf_traffic::Route route;
};

struct Description
{
  std::size_t samples;
  std::optional<rmf_traffic::Duration> max_duration;

  std::unordered_map<std::string,
    rmf_traffic::agv::Planner::Configuration> robots;
  std::vector<Plan> obstacle_plans;
  std::vector<Route> obstacle_routes;

  Plan plan;
};

struct Arguments
{
  std::string scenario_file;
  std::optional<std::size_t> samples;
  std::optional<double> max_duration;
  bool include_no_obstacle_tests = true;
  bool include_obstacle_tests = true;
  bool include_no_cache_tests = true;
  bool include_cache_tests = true;
};

bool load(std::string file_name, YAML::Node& node);

bool load_graph(
  std::string file_name,
  rmf_traffic::agv::VehicleTraits traits,
  rmf_traffic::agv::Graph& graph);

std::pair<std::string, std::string> parse_test_options(const std::string& args);

Arguments parse_arguments(int argc, char* argv[]);

void parse(Arguments arguments, Description& description);

}
}

#endif //RMF_PERFORMANCE_TESTS_SCENARIO_HPP
