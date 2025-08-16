#include "input/FibrationTreesBenchmakerExecuter.hpp"

#include <iostream>
#include <filesystem>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/util/Console.h>

#include "FilePath.hpp"
#include "Common.hpp"
#include "ToString.hpp"
#include "input/FibrationTreesBenchmakerArguments.hpp"
#include "OmplHelper.hpp"
#include "RunBenchmark.hpp"
#include "gui/Visualizer.hpp"
#include "yaml/MakeFromYaml.hpp"
#include "yaml/MakePlannerFromYaml.hpp"

const double kDefaultMinimalTimeout = 0.01;
const size_t kDefaultMinimalRunCount = 1U;

double GetTimeout(const YAML::Node& node, const FibrationTreesBenchmakerArguments& program_options) {
  if(program_options.HasValue("dry")) {
    return kDefaultMinimalTimeout;
  }
  if(!node["timeout"]) {
    return kDefaultMinimalTimeout;
  }
  return node["timeout"].as<double>();
}

size_t GetRunCount(const YAML::Node& node, const FibrationTreesBenchmakerArguments& program_options) {
  if(program_options.HasValue("dry")) {
    return kDefaultMinimalRunCount;
  }
  if(!node["run_count"]) {
    return kDefaultMinimalRunCount;
  }
  return node["run_count"].as<size_t>();
}

std::string MakeOutputFilename(const std::string& filename) {
  std::filesystem::path p(filename);
  return p.stem();
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
  const auto benchmark_name = root_node["name"].as<std::string>();
  OMPL_INFORM("Loading File %s", benchmark_name.c_str());

  if(!root_node["scenarios"]) {
    OMPL_ERROR("Could not find scenarios in benchmark file.");
    return 1;
  }

  const auto scenarios = root_node["scenarios"];

  std::vector<std::string> output_filenames;

  for(const auto& scenario : scenarios) {
    auto node_name = scenario.first.as<std::string>();
    auto node = scenario.second;
    auto raw_filename = node["filename"].as<std::string>();
    OMPL_INFORM("Loading scenario %s from file %s", node_name.c_str(), raw_filename.c_str());

    const auto filename = GetDataFolder() + raw_filename;

    dart::simulation::WorldPtr world(new dart::simulation::World);

    auto [factor, pdef, root_robot, child_robots, dynamic_obstacles] = MakeFactoredSpaceInformationFromYamlFilename(filename, world);

    const auto timeout = GetTimeout(node, program_options);
    const auto run_count = GetRunCount(node, program_options);
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

    if(!program_options.HasValue("dry")) {
      auto output_filename = MakeOutputFilename(filename);
      SaveBenchmarkToDatabase(output_filename, benchmark_result);
      output_filenames.push_back(output_filename);
      OMPL_INFORM("Saved benchmark to file logs/%s.db", output_filename.c_str());
    }
  }

  if(!program_options.HasValue("dry")) {
    std::string string_filename = "";
    for(const auto& output_filename : output_filenames) {
      string_filename += GetDataFolder() + "logs/" + output_filename + ".log ";
    }
    
    auto db_filename = GetDataFolder() + "logs/" + benchmark_name + ".db";
    OMPL_INFORM("To DB: %s", string_filename.c_str());
    auto cmd_log_to_db = "python3 " + GetMainFolder() + "scripts/ompl_benchmark_statistics.py "+string_filename+" -d "+db_filename;
    system(cmd_log_to_db.c_str());

    if(root_node["ompl_benchmark_plotter"]) {
      auto plotter = root_node["ompl_benchmark_plotter"];

      std::string option_str = "";
      if (plotter["options"]) {
        for (const auto& option : plotter["options"]) {
          option_str += option.as<std::string>() + " ";
        }
        std::cout << "Option: " << option_str << std::endl;
      }
      auto cmd_plot_db = "python3 " + GetMainFolder() + "../ompl_benchmark_plotter/ompl_benchmark_plotter.py "+option_str+" "+db_filename;
      system(cmd_plot_db.c_str());
    }
  }


  return 0;

}
