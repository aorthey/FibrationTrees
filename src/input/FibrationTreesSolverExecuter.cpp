#include "input/FibrationTreesSolverExecuter.hpp"

#include <iostream>
#include <chrono>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/util/Console.h>

#include "FilePath.hpp"
#include "Common.hpp"
#include "input/FibrationTreesSolverArguments.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "yaml/MakeFromYaml.hpp"
#include "yaml/MakePlannerFromYaml.hpp"
#include "yaml/MakeViewerFromYaml.hpp"

using namespace std::chrono;
using time_clock = std::chrono::steady_clock;
using std::chrono::seconds;


int FibrationTreesSolverExecuter(const int argc, const char* argv[]) {
  dart::math::Random::setSeed(0);

  auto program_options = FibrationTreesSolverArguments();

  if(!program_options.Setup(argc, argv)) {
    return 0;
  }

  auto input_filename = program_options.Get<std::string>("filename");

  const auto filename = GetDataFolder() + input_filename;

  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto [factor, pdef, root_robot, child_robots, dynamic_obstacles] = MakeFactoredSpaceInformationFromYamlFilename(filename, world);

  if(program_options.HasValue("dry")) {
    return 0;
  }

  if(!program_options.HasValue("planning")) {
    Visualizer visualizer = MakeViewerFromYamlFilename(filename, world);
    visualizer.Run();
    return 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize planner
  ////////////////////////////////////////////////////////////////////////////////

  auto input_planner_name = program_options.Get<std::string>("planning");
  OMPL_INFORM("Input planner %s", input_planner_name.c_str());

  auto planner = MakePlannerFromYaml(input_filename, input_planner_name, factor, child_robots);
  planner->setProblemDefinition(pdef);
  planner->setup();

  ////////////////////////////////////////////////////////////////////////////////
  // Run planner
  ////////////////////////////////////////////////////////////////////////////////

  auto time_start = time_clock::now();

  auto ptc = TimeOrSolutionPtc(pdef, program_options.Get<double>("timeout"));
  ompl::base::PlannerStatus status = planner->solve(ptc);

  auto time_end = time_clock::now();
  auto duration = std::chrono::duration_cast<seconds>(time_end - time_start);

  if(status) {
    OMPL_INFORM("\n%s\nSolved planning problem with %s in %ds.\n%s\n", std::string(80, '*').c_str(), 
        input_planner_name.c_str(), duration.count(), std::string(80, '*').c_str());
  } else {
    OMPL_ERROR("\n%s\nFailed to solve planning problem with %s in %ds.\n%s\n", std::string(80, '*').c_str(), 
        input_planner_name.c_str(), duration.count(), std::string(80, '*').c_str());
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Visualize results
  ////////////////////////////////////////////////////////////////////////////////
  if(program_options.HasValue("command-line-mode")) {
    return 0;
  }

  Visualizer visualizer = MakeViewerFromYamlFilename(filename, world);
  visualizer.SetCollisionChecker(root_robot->GetCollisionChecker());

  for(const auto& child_robot : child_robots) {
    if(!child_robot.second->ShouldShowPath()) {
      continue;
    }
    auto fplanner = std::dynamic_pointer_cast<ompl::multilevel::FibrationRRT>(planner);
    if(fplanner == nullptr) {
      continue;
    }

    auto child_pdef = fplanner->getProblemDefinition(child_robot.second->GetName());

    if(child_pdef == nullptr) {
      continue;
    }
    visualizer.AddPath(child_robot.second, child_pdef->getSolutionPath(), State3d(1, 1, 0));
  }

  if(status) {
    visualizer.AddPlanner(root_robot, planner);
  }
  for(const auto& dynamic_obstacle : dynamic_obstacles) {
    visualizer.AddPath(dynamic_obstacle.first, dynamic_obstacle.second, State3d(1, 1, 0));
  }

  visualizer.Run();

  return 0;
}
