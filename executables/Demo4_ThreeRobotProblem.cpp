#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/MobileKukaBase.hpp"
#include "robots/MobileCar.hpp"
#include "robots/MobileCarDisk.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/MultiRobot.hpp"
#include "projections/ProjectionTaskSpace.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RNSO2_RN.h>
#include <ompl/multilevel/datastructures/projections/XRN_X_SE2.h>
#include <ompl/multilevel/datastructures/projections/SE2RN_R2.h>
#include <ompl/multilevel/datastructures/projections/SE2_R2.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/base/goals/FactoredGoal.h>

#include <ranges>

const float kZheight = 0.8;
const float kAccuracy = 0.02;
const float kWireWidth = 0.02;

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  obstacles.push_back(createFloor(-0.255));
  //obstacles.push_back(createBox(State3d(0,0,0), 0.15, 0.15, 0.5));
  obstacles.push_back(createCylinder(State3d(0, 0, kZheight+kAccuracy+2*kWireWidth), State3d(0.5*M_PI, 0.5*M_PI, 0), kWireWidth, 3.5));

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle: obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(State3d::Zero());

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  //
  //                            factor (all robots)
  //              ______________/        |       \________________
  //             |                       |                        |
  //      factor1 (robot1)        factor2 (robot2)         factor3 (robot3)
  //             |                       |                        |
  //   child1 (child_robot1)    child2 (child_robot2)    child3 (child_robot3)
  //
  ////////////////////////////////////////////////////////////////////////////////

  auto robot1 = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles);
  auto factor1 = robot1->GetSpaceInformation();

  auto robot2 = MakeRobot<MobileCar>(world, obstacles);
  auto factor2 = robot2->GetSpaceInformation();

  auto robot3 = MakeRobot<MobileKukaRobot>(world, obstacles);
  auto factor3 = robot3->GetSpaceInformation();

  ////////////////////////////////////////////////////////////////////////////////
  ////Setup state spaces for projection robots
  ////////////////////////////////////////////////////////////////////////////////
  auto child_robot1 = MakeRobot<SphereRobot>(world, obstacles);
  const auto limits1 = std::make_pair(State3d(-1.5, 0.0-kAccuracy, kZheight - kAccuracy), State3d(1.5, 0.0+kAccuracy, kZheight + kAccuracy));
  child_robot1->SetLimits(limits1);
  auto child1 = child_robot1->GetSpaceInformation();
  auto projection1 = std::make_shared<ProjectionTaskSpace>(factor1, child1, robot1);
  factor1->addChild(child1, projection1);

  auto child_robot2 = MakeRobot<MobileCarDisk>(world, obstacles);
  auto child2 = child_robot2->GetSpaceInformation();
  auto projection2 = std::make_shared<ompl::multilevel::Projection_SE2_R2>(factor2->getStateSpace(), child2->getStateSpace());
  factor2->addChild(child2, projection2);

  auto child_robot3 = MakeRobot<MobileKukaBase>(world, obstacles);
  auto child3 = child_robot3->GetSpaceInformation();
  auto projection3 = std::make_shared<ompl::multilevel::Projection_SE2RN_SE2>(factor3->getStateSpace(), child3->getStateSpace());
  factor3->addChild(child3, projection3);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create total space for all robots
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<RobotPtr> robots = {robot1, robot2, robot3};
  auto multi_robot = MultiRobot::MakeMultiRobot(robots);

  auto factor = multi_robot->GetSpaceInformation();

  auto projection_to_1 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor->getStateSpace(), factor1->getStateSpace(), 0);
  auto projection_to_2 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor->getStateSpace(), factor2->getStateSpace(), 1);
  auto projection_to_3 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor->getStateSpace(), factor3->getStateSpace(), 2);

  bool computer_fiber_space = false;
  ReturnOnFalse(factor->addChild(factor1, projection_to_1, computer_fiber_space), 1);
  ReturnOnFalse(factor->addChild(factor2, projection_to_2, computer_fiber_space), 1);
  ReturnOnFalse(factor->addChild(factor3, projection_to_3, computer_fiber_space), 1);

  auto pairwise_collision_checker = std::make_shared<DartMultiRobotCollisionChecker>(factor, world, robots);
  factor->setStateValidityChecker(pairwise_collision_checker);

  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<MotionValidatorTaskSpaceMultiRobot>(factor);
  factor->setMotionValidator(motion_validator);

  factor->printFactorization(std::cout);

  //////////////////////////////////////////////////////////////////////////////////
  //////Create start configuration
  //////////////////////////////////////////////////////////////////////////////////
  auto start_state1 = MakeState({-1.5, 0.0, kZheight});
  auto start1 = child1->allocState();
  child_robot1->EigenToState(start_state1, start1);

  auto start_state2 = MakeState({+1.0, +1.0});
  auto start2 = child2->allocState();
  child_robot2->EigenToState(start_state2, start2);

  auto start_state3 = MakeState({+0.0, -1.0, 0.0});
  auto start3 = child3->allocState();
  child_robot3->EigenToState(start_state3, start3);

  std::unordered_map<std::string, ompl::base::State*> task_space_start_states;
  task_space_start_states[child1->getName()] = start1;
  task_space_start_states[child2->getName()] = start2;
  task_space_start_states[child3->getName()] = start3;

  auto maybe_start = ComputeValidTotalState(factor, task_space_start_states);
  if(!maybe_start.has_value()){
    OMPL_ERROR("Could not compute valid start.");
    return 1;
  }

  auto start = maybe_start.value();
  factor->printState(start);
  //////////////////////////////////////////////////////////////////////////////////
  //////Create goal configuration
  //////////////////////////////////////////////////////////////////////////////////
  auto goal_state1 = MakeState({+1.5, 0.0, kZheight});
  auto goal1 = child1->allocState();
  child_robot1->EigenToState(goal_state1, goal1);

  auto goal_state2 = MakeState({-1.0, -1.0});
  auto goal2 = child2->allocState();
  child_robot2->EigenToState(goal_state2, goal2);

  auto goal_state3 = MakeState({+0.0, +1.0, 0.0});
  auto goal3 = child3->allocState();
  child_robot3->EigenToState(goal_state3, goal3);

  std::unordered_map<std::string, ompl::base::State*> task_space_goal_states;
  task_space_goal_states[child1->getName()] = goal1;
  task_space_goal_states[child2->getName()] = goal2;
  task_space_goal_states[child3->getName()] = goal3;

  auto maybe_goal = ComputeValidTotalState(factor, task_space_goal_states);
  if(!maybe_goal.has_value()){
    OMPL_ERROR("Could not compute goal goal.");
    return 1;
  }

  auto goal = maybe_goal.value();
  factor->printState(goal);

  world->addSimpleFrame(createSphereFrame(start_state1, 0.02, color_red));
  world->addSimpleFrame(createSphereFrame(goal_state1, 0.02, color_red_light));

  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////Create problem definition
  ////////////////////////////////////////////////////////////////////////////////////////
  // auto goalStates = factor->allocChildStates();
  // factor->project(goal, goalStates);
  // auto goalState1 = goalStates.at(factor1->getName());
  // auto goalState2 = goalStates.at(factor2->getName());
  // auto goalState3 = goalStates.at(factor3->getName());

  // auto goal_region1 = std::make_shared<TaskSpaceGoal>(factor1, goalState1, projection1);
  // goal_region1->setThreshold(0.1);
  // auto goal_region2 = std::make_shared<TaskSpaceGoal>(factor2, goalState2, projection2);
  // goal_region2->setThreshold(0.1);
  // auto goal_region3 = std::make_shared<TaskSpaceGoal>(factor3, goalState3, projection3);
  // goal_region3->setThreshold(0.1);

  // std::unordered_map<std::string, ompl::base::GoalSampleableRegionPtr> goal_regions;
  // goal_regions[factor1->getName()] = goal_region1;
  // goal_regions[factor2->getName()] = goal_region2;
  // goal_regions[factor3->getName()] = goal_region3;

  // auto goal_region = std::make_shared<ompl::base::FactoredGoal>(factor, goal_regions);
  // goal_region->setThreshold(0.5);
  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->setStartAndGoalStates(start, goal, 0.5);
 
  ////////////////////////////////////////////////////////////////////////////////////////
  ////////////Planning
  ////////////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(Inf);
  planner->setSmoothIntermediateSolutions(false);
 
  float timeout = 1000.0;

  auto ptc = TimeOrSolutionPtc(pdef, timeout);
  ompl::base::PlannerStatus status = planner->solve(ptc);
 
  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  hide(child_robot1->GetSkeleton());
  hide(child_robot2->GetSkeleton());
  hide(child_robot3->GetSkeleton());

  Visualizer visualizer(world);
  visualizer.SetCollisionChecker(pairwise_collision_checker->GetCollisionChecker());
  visualizer.AddPlanner(multi_robot, planner);

  visualizer.Run();

  return 0;
}


