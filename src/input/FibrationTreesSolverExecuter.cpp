#include "input/FibrationTreesSolverExecuter.hpp"

#include <iostream>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/util/Console.h>

#include "FilePath.hpp"
#include "Common.hpp"
#include "input/FibrationTreesSolverArguments.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "yaml/MakeFromYaml.hpp"

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
    Visualizer visualizer(world);
    visualizer.Run();
    return 0;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Initialize planner
  ////////////////////////////////////////////////////////////////////////////////

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(+Inf);

  // planner->setSamplingPerturbationBias(child->getName(), 0.0);
  // planner->setPathRestrictionSamplingBias(child->getName(), 0.5);
  // planner->setPathRestrictionSurroundingSamplingBias(child->getName(), 0.0);
  // planner->setSmoothIntermediateSolutions(child->getName(), true);

  ////////////////////////////////////////////////////////////////////////////////
  // Check smoothing for individual stages
  ////////////////////////////////////////////////////////////////////////////////

  planner->setSmoothIntermediateSolutions(false);
  for(const auto& child_robot : child_robots) {
    planner->setSmoothIntermediateSolutions(child_robot.second->GetName(), child_robot.second->ShouldSmoothPath());
  }
  planner->setSmoothIntermediateSolutions(root_robot->GetName(), root_robot->ShouldSmoothPath());


  ////////////////////////////////////////////////////////////////////////////////
  // Run planner
  ////////////////////////////////////////////////////////////////////////////////

  planner->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kUniform);
  planner->setName("FibrationRRT");

  auto ptc = TimeOrSolutionPtc(pdef, program_options.Get<double>("timeout"));
  ompl::base::PlannerStatus status = planner->solve(ptc);

  ////////////////////////////////////////////////////////////////////////////////
  // Visualize results
  ////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  visualizer.SetCollisionChecker(root_robot->GetCollisionChecker());

  for(const auto& child_robot : child_robots) {
    if(!child_robot.second->ShouldShowPath()) {
      continue;
    }
    auto child_pdef = planner->getProblemDefinition(child_robot.second->GetName());
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
