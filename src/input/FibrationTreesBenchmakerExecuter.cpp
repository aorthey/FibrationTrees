#include "input/FibrationTreesBenchmakerExecuter.hpp"

#include <iostream>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/util/Console.h>

#include "FilePath.hpp"
#include "Common.hpp"
#include "input/FibrationTreesBenchmakerArguments.hpp"
#include "OmplHelper.hpp"
#include "RunBenchmark.hpp"
#include "gui/Visualizer.hpp"
#include "yaml/MakeFromYaml.hpp"
#include "yaml/MakePlannerFromYaml.hpp"

const double kDefaultMinimalTimeout = 1.0;
const size_t kDefaultMinimalRunCount = 1U;

double GetTimeout(const YAML::Node& node) {
  if(node["dry"]) {
    if(node["dry"].as<bool>()) {
      return kDefaultMinimalTimeout;
    }
  }
  if(!node["timeout"]) {
    return kDefaultMinimalTimeout;
  }
  return node["timeout"].as<double>();
}

size_t GetRunCount(const YAML::Node& node) {
  if(node["dry"]) {
    if(node["dry"].as<bool>()) {
      return kDefaultMinimalRunCount;
    }
  }
  if(!node["run_count"]) {
    return kDefaultMinimalRunCount;
  }
  return node["run_count"].as<size_t>();
}

int FibrationTreesBenchmakerExecuter(const int argc, const char* argv[]) {
  dart::math::Random::setSeed(0);

  auto program_options = FibrationTreesBenchmakerArguments();

  if(!program_options.Setup(argc, argv)) {
    return 1;
  }

  auto input_filename = program_options.Get<std::string>("filename");
  OMPL_INFORM("%s", input_filename.c_str());

  YAML::Node root_node = YAML::LoadFile(input_filename);
  OMPL_INFORM("Loading File %s", root_node["name"].as<std::string>().c_str());

  if(!root_node["scenarios"]) {
    OMPL_ERROR("Could not find scenarios in benchmark file.");
    return 1;
  }

  const auto scenarios = root_node["scenarios"];

  for(const auto& scenario : scenarios) {
    auto node_name = scenario.first.as<std::string>();
    auto node = scenario.second;
    auto raw_filename = node["filename"].as<std::string>();
    OMPL_INFORM("Loading scenario %s from file %s", node_name.c_str(), raw_filename.c_str());

    const auto filename = GetDataFolder() + raw_filename;

    dart::simulation::WorldPtr world(new dart::simulation::World);

    auto [factor, pdef, root_robot, child_robots, dynamic_obstacles] = MakeFactoredSpaceInformationFromYamlFilename(filename, world);

    auto timeout = GetTimeout(node);
    auto run_count = GetRunCount(node);
    auto scenario_name = node["name"].as<std::string>();
    auto planner_names = node["planners"].as<std::vector<std::string>>();

    std::vector<ompl::base::PlannerPtr> planners;
    planners.reserve(planner_names.size());

    for(const auto& planner_name : planner_names) {
      auto planner = MakePlannerFromYaml(root_node, planner_name, factor, child_robots);
      planners.push_back(planner);
    }

    ompl::base::ScopedState<> start(factor);
    start = pdef->getStartState(0);
    auto goal = pdef->getGoal();
    auto benchmark_result = RunBenchmark(scenario_name, factor, start, goal, timeout, run_count, planners);
    SaveBenchmarkToDatabase(scenario_name, benchmark_result);
  }

  return 0;

}
