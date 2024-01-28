#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "TaskSpaceMultiRobotMotionValidator.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/MultiRobot.hpp"
#include "TaskSpaceProjection.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RNSO2_RN.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/base/goals/FactoredGoal.h>

#include <ranges>

const float kZheight = 0.5;
const float kAccuracy = 0.02;
int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  dart::dynamics::SkeletonPtr floor = createFloor(-0.255);
  dart::dynamics::SkeletonPtr box = createBox(Eigen::Vector3d(0,0,0), 0.3, 0.3, 0.5);
  obstacles.push_back(floor);
  obstacles.push_back(box);

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle: obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(Eigen::Vector3d::Zero());

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  //
  //                    factor (all robots)
  //                  /          |          \
  //                /            |            \
  //   factor1 (robot1)   factor2 (robot2)   factor3 (robot3)
  //      |                  |                   |
  //   child1 (point1)    child2 (point2)    child3 (point3)
  //
  //
  //CompoundSpace15 (R2xSO2xR7xR2xSO2xR7xR2xSO2xR7, dimensionality 30)
  // └────MobileManipulatorCompoundSpace0 (R2xSO2xR7, dimensionality 10)
  //      └────RealVectorSpace12 (R3, dimensionality 3)
  // └────MobileManipulatorCompoundSpace4 (R2xSO2xR7, dimensionality 10)
  //      └────RealVectorSpace13 (R3, dimensionality 3)
  // └────MobileManipulatorCompoundSpace8 (R2xSO2xR7, dimensionality 10)
  //      └────RealVectorSpace14 (R3, dimensionality 3)
  ////////////////////////////////////////////////////////////////////////////////

  auto robot1 = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles);
  auto factor1 = robot1->GetSpaceInformation();
  auto robot2 = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles);
  auto factor2 = robot2->GetSpaceInformation();
  auto robot3 = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles);
  auto factor3 = robot3->GetSpaceInformation();

  ////////////////////////////////////////////////////////////////////////////////
  ////Setup state spaces for projection robots
  ////////////////////////////////////////////////////////////////////////////////
  auto point1 = MakeRobot<SphereRobot>(world, obstacles);
  const auto limits1 = std::make_pair(Eigen::Vector3d(-1.5, 0.0-kAccuracy, kZheight - kAccuracy), Eigen::Vector3d(1.5, 0.0+kAccuracy, kZheight + kAccuracy));
  point1->SetLimits(limits1);
  auto child1 = point1->GetSpaceInformation();
  auto projection1 = std::make_shared<TaskSpaceProjection>(factor1, child1, robot1);
  factor1->addChild(child1, projection1);

  auto point2 = MakeRobot<SphereRobot>(world, obstacles);
  const auto limits2 = std::make_pair(Eigen::Vector3d(0.0-kAccuracy, -1.5, kZheight - kAccuracy), Eigen::Vector3d(0.0+kAccuracy, 1.5, kZheight + kAccuracy));
  point2->SetLimits(limits2);
  auto child2 = point2->GetSpaceInformation();
  auto projection2 = std::make_shared<TaskSpaceProjection>(factor2, child2, robot2);
  factor2->addChild(child2, projection2);

  auto point3 = MakeRobot<SphereRobot>(world, obstacles);
  const auto limits3 = std::make_pair(Eigen::Vector3d(-1.5, -1.5, kZheight - kAccuracy), Eigen::Vector3d(1.5, 1.5, kZheight + kAccuracy));
  point3->SetLimits(limits3);
  auto child3 = point3->GetSpaceInformation();
  auto projection3 = std::make_shared<TaskSpaceProjection>(factor3, child3, robot3);
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

  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<TaskSpaceMultiRobotMotionValidator>(factor);
  factor->setMotionValidator(motion_validator);

  factor->printFactorization(std::cout);

  //////////////////////////////////////////////////////////////////////////////////
  //////Create problem definition
  //////////////////////////////////////////////////////////////////////////////////
  auto task_start1 = child1->allocState();
  auto task_goal1 = child1->allocState();
  auto task_start2 = child2->allocState();
  auto task_goal2 = child2->allocState();
  auto task_start3 = child3->allocState();
  auto task_goal3 = child3->allocState();

  auto task_start1_eigen = MakeEigen({-1.3, -0.0, kZheight});
  auto task_goal1_eigen = MakeEigen({+1.3, +0.0, kZheight});
  auto task_start2_eigen = MakeEigen({-0.0, -1.3, kZheight});
  auto task_goal2_eigen = MakeEigen({+0.0, +1.3, kZheight});
  auto task_start3_eigen = MakeEigen({+1.3, +1.3, kZheight});
  auto task_goal3_eigen = MakeEigen({-1.3, -1.3, kZheight});

  point1->EigenToState(task_start1_eigen, task_start1);
  point1->EigenToState(task_goal1_eigen, task_goal1);
  point2->EigenToState(task_start2_eigen, task_start2);
  point2->EigenToState(task_goal2_eigen, task_goal2);
  point3->EigenToState(task_start3_eigen, task_start3);
  point3->EigenToState(task_goal3_eigen, task_goal3);

  std::unordered_map<std::string, ompl::base::State*> task_space_start_states;
  task_space_start_states[child1->getName()] = task_start1;
  task_space_start_states[child2->getName()] = task_start2;
  task_space_start_states[child3->getName()] = task_start3;

  auto maybe_start = ComputeValidTotalState(factor, task_space_start_states);
  if(!maybe_start.has_value()){
    OMPL_ERROR("Could not compute valid start.");
    return 1;
  }
  auto start = maybe_start.value();
  
  OMPL_INFORM("Found start state:");
  factor->printState(start);

  std::unordered_map<std::string, ompl::base::State*> task_space_goal_states;
  task_space_goal_states[child1->getName()] = task_goal1;
  task_space_goal_states[child2->getName()] = task_goal2;
  task_space_goal_states[child3->getName()] = task_goal3;

  auto maybe_goal = ComputeValidTotalState(factor, task_space_goal_states);
  if(!maybe_goal.has_value()){
    OMPL_ERROR("Could not compute valid goal.");
    return 1;
  }
  auto goal = maybe_goal.value();
  OMPL_INFORM("Found goal state:");
  factor->printState(goal);

  world->addSimpleFrame(createSphereFrame(task_start1_eigen, 0.02, color_red));
  world->addSimpleFrame(createSphereFrame(task_goal1_eigen, 0.02, color_red_light));
  world->addSimpleFrame(createSphereFrame(task_start2_eigen, 0.02, color_green));
  world->addSimpleFrame(createSphereFrame(task_goal2_eigen, 0.02, color_green_light));
  world->addSimpleFrame(createSphereFrame(task_start3_eigen, 0.02, color_blue));
  world->addSimpleFrame(createSphereFrame(task_goal3_eigen, 0.02, color_blue_light));

  auto goalStates = factor->allocChildStates();
  factor->project(goal, goalStates);
  auto goal1 = goalStates.at(factor1->getName());
  auto goal2 = goalStates.at(factor2->getName());
  auto goal3 = goalStates.at(factor3->getName());

  auto goal_region1 = std::make_shared<TaskSpaceGoal>(factor1, goal1, projection1);
  goal_region1->setThreshold(0.1);
  auto goal_region2 = std::make_shared<TaskSpaceGoal>(factor2, goal2, projection2);
  goal_region2->setThreshold(0.1);
  auto goal_region3 = std::make_shared<TaskSpaceGoal>(factor3, goal3, projection3);
  goal_region3->setThreshold(0.1);

  std::unordered_map<std::string, ompl::base::GoalSampleableRegionPtr> goal_regions;
  goal_regions[factor1->getName()] = goal_region1;
  goal_regions[factor2->getName()] = goal_region2;
  goal_regions[factor3->getName()] = goal_region3;

  auto goal_region = std::make_shared<ompl::base::FactoredGoal>(factor, goal_regions);
  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoal(goal_region);
 
  //////////////////////////////////////////////////////////////////////////////////////
  //////////Planning
  //////////////////////////////////////////////////////////////////////////////////////
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
  hide(point1->GetSkeleton());
  hide(point2->GetSkeleton());
  hide(point3->GetSkeleton());

  Visualizer visualizer(world);
  visualizer.SetCollisionChecker(pairwise_collision_checker->GetCollisionChecker());
  visualizer.AddPlanner(multi_robot, planner);

  visualizer.Run();

  return 0;
}


