#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "projections/ProjectionTaskSpace.hpp"
#include "gui/Visualizer.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "FilePath.hpp"
#include "RunBenchmark.hpp"
#include "samplers/TaskSpaceSampler.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRT.h>

const int kMaxResampleIteration = 100;
const float kGoalRegion = 0.025;

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating obstacles
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  obstacles.push_back(createFromURDF(GetDataFolder() + "objects/maze.urdf", State3d(+0.55, +0.1, 0.85)));
  obstacles.push_back(createFloor());
  obstacles.push_back(createBox(State3d(+0.5, +0.0, 0.75), 0.15, 2.0, 1.5));

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle : obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(State3d::Zero());
  addCoordinateFrameToWorld(world);

  auto robot = MakeRobot<KukaRobotTaskSpace>(world, obstacles);
  auto point = MakeRobot<SphereRobot>(world, obstacles);
  hide(point->GetSkeleton());

  const auto task_space_limits = std::make_pair(State3d(0.39, -0.5, 0), State3d(0.43, +0.5, 2));
  point->SetLimits(task_space_limits);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor = robot->GetSpaceInformation();
  auto child = point->GetSpaceInformation();

  ompl::multilevel::ProjectionPtr projection = std::make_shared<ProjectionTaskSpace>(factor, child, robot);
  factor->addChild(child, projection);

  factor->printFactorization(std::cout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::State *task_start = child->allocState();
  ompl::base::State *task_goal = child->allocState();

  point->EigenToState(MakeState({0.4, +0.35, 0.95}), task_start);
  point->EigenToState(MakeState({0.4, -0.25, 0.95}), task_goal);

  ompl::base::ScopedState<> start(factor);
  ompl::base::State *goal = factor->allocState();

  bool has_solution = true;

  if(!SampleValidLift(projection, factor, kMaxResampleIteration, task_start, start.get())) {
    OMPL_ERROR("Could not find valid start state after %d samples.", kMaxResampleIteration);
    child->printState(task_start);
    has_solution = false;
  }
  if(has_solution && !SampleValidLift(projection, factor, kMaxResampleIteration, task_goal, goal)) {
    OMPL_ERROR("Could not find valid goal state after %d samples.", kMaxResampleIteration);
    child->printState(task_goal);
    has_solution = false;
  }

  if(!has_solution) {
    OMPL_ERROR("Could not find a solution for start/goal configurations.");
    exit(0);
  }

  auto goal_region = std::make_shared<TaskSpaceGoal>(factor, goal, projection);
  goal_region->setThreshold(kGoalRegion);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoal(goal_region);

  auto start_vector = point->StateToEigen(task_start);
  auto goal_vector = point->StateToEigen(task_goal);
  world->addSimpleFrame(createCylinderFrame(start_vector.configuration, State3d(0.0, M_PI*0.5, 0.0), 0.01, 0.001, State4d(0.1, 0.5, 0.1, 0.5)));
  world->addSimpleFrame(createCylinderFrame(goal_vector.configuration, State3d(0.0, M_PI*0.5, 0.0), kGoalRegion, 0.001, State4d(0.1, 0.5, 0.1, 0.5)));

  ////////////////////////////////////////////////////////////////////////////////
  ////Planning
  ////////////////////////////////////////////////////////////////////////////////
  double timeout = 20.0;
  size_t run_count = 5;

  ////////////////////////////////////////////////////////////////////////////////
  //RRTtask
  ////////////////////////////////////////////////////////////////////////////////
  // factor->getStateSpace()->setStateSamplerAllocator(
  //         std::bind(&allocateTaskSpaceSampler, robot, task_space_limits));
  auto planner1 = std::make_shared<ompl::geometric::RRTtask>(factor);
  planner1->setProblemDefinition(pdef);
  planner1->setup();
  planner1->setName("RRT");

  ////////////////////////////////////////////////////////////////////////////////
  //FibrationRRT
  ////////////////////////////////////////////////////////////////////////////////
  auto planner2 = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner2->setProblemDefinition(pdef);
  planner2->setup();
  //planner2->setRange(+Inf);
  planner2->setRange(+Inf);
  //planner2->setRange(child->getName(), 0.2);
  //planner2->setSeed(0);


  planner2->setSamplingPerturbationBias(child->getName(), 0.0);
  planner2->setPathRestrictionSamplingBias(child->getName(), 0.5);
  planner2->setPathRestrictionSurroundingSamplingBias(child->getName(), 0.0);
  // planner2->setSamplingPerturbationBias(factor->getName(), 0.0);
  // planner2->setPathRestrictionSamplingBias(factor->getName(), 0.0);
  // planner2->setPathRestrictionSurroundingSamplingBias(factor->getName(), 0.0);

  planner2->setSmoothIntermediateSolutions(child->getName(), true);
  planner2->setSmoothIntermediateSolutions(factor->getName(), false);
  //planner2->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kExponential);
  planner2->setSelectorFunctionType(ompl::multilevel::SelectorFunctionType::kUniform);
  planner2->setName("FibrationRRT");

  ompl::tools::Benchmark::PreSetupEvent pre_setup_event = 
    [&](const ompl::base::PlannerPtr& planner) -> void {
      std::cout << planner->getName() << std::endl;
      if(planner->getName() == "RRT") {
        planner->getSpaceInformation()->getStateSpace()->setStateSamplerAllocator(
                std::bind(&allocateTaskSpaceSampler, robot, task_space_limits));
      } else {
        planner->getSpaceInformation()->getStateSpace()->allocDefaultStateSampler();
      }
    };


  // auto name = "Scenario2";
  // auto benchmark = RunBenchmark(name, factor, start, goal_region, timeout, run_count, {planner2}, pre_setup_event);
  // SaveBenchmarkToDatabase(name, benchmark);
  // return 0;
  auto ptc = ompl::base::plannerOrTerminationCondition(
          ompl::base::exactSolnPlannerTerminationCondition(pdef),
          ompl::base::timedPlannerTerminationCondition(timeout)
      );
  ompl::base::PlannerStatus status = planner2->solve(ptc);
  Visualizer visualizer(world);
  visualizer.AddPlanner(robot, planner2);
  visualizer.SetCollisionChecker(robot->GetCollisionChecker());
  //visualizer.AddPath(point, planner->getProblemDefinition(child->getName())->getSolutionPath(), State3d(1, 1, 0));
  visualizer.Run();

  return 0;
}
