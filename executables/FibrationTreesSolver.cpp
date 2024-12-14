#include <iostream>
//#include <boost/program_options.hpp>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/util/Console.h>

#include "FilePath.hpp"
#include "Common.hpp"
#include "ProgramOptions.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "yaml/MakeFromYaml.hpp"


int main(int argc, char* argv[]) {

  dart::math::Random::setSeed(0);

  auto program_options = ProgramOptions();

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

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(+Inf);
  //planner->setSamplingPerturbationBias(child->getName(), 0.0);
  //planner->setPathRestrictionSamplingBias(child->getName(), 0.5);
  //planner->setPathRestrictionSurroundingSamplingBias(child->getName(), 0.0);
  //planner->setSmoothIntermediateSolutions(child->getName(), true);
  planner->setSmoothIntermediateSolutions(true);
  planner->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kExponential);
  planner->setName("FibrationRRT");

  auto ptc = TimeOrSolutionPtc(pdef, program_options.Get<double>("timeout"));
  ompl::base::PlannerStatus status = planner->solve(ptc);

  Visualizer visualizer(world);
  visualizer.SetCollisionChecker(root_robot->GetCollisionChecker());

  if(status) {
    visualizer.AddPlanner(root_robot, planner);
    // for(const auto& child_robot : child_robots) {
    //   auto child_pdef = planner->getProblemDefinition(child_robot.second->GetName());
    //   if(child_pdef == nullptr) {
    //     continue;
    //   }
    //   visualizer.AddPath(child_robot.second, child_pdef->getSolutionPath(), State3d(1, 1, 0));
    // }
  }
  for(const auto& dynamic_obstacle : dynamic_obstacles) {
    visualizer.AddPath(dynamic_obstacle.first, dynamic_obstacle.second, State3d(1, 1, 0));
  }
  visualizer.Run();

  return 0;
}
