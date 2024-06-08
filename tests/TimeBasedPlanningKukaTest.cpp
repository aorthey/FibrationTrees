#include <gtest/gtest.h>

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
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "TimeGoal.hpp"
#include "validators/MotionValidatorTimeBased.hpp"
#include "TimeOrSolutionTerminationCondition.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/multilevel/datastructures/projections/TimeBasedProjection.h>

float kObstacleEndTime = 20.0;

std::pair<RobotPtr, ompl::base::PathPtr> MakeStraightRobotPath(
    const std::vector<double>& start_xy,
    const std::vector<double>& goal_xy,
    const double end_time,
    const dart::simulation::WorldPtr& world, 
    const std::vector<dart::dynamics::SkeletonPtr>& static_obstacles
    ) {
  auto robot = MakeRobot<TimeBasedMobileKukaRobotTaskSpace>(world, static_obstacles);

  auto state1 = MakeState({start_xy.at(0), start_xy.at(1), 0, 0, 0, 0, 0, 0, 0, 0});
  state1.time = 0.0;
  auto state2 = MakeState({goal_xy.at(0), goal_xy.at(1),   0, 0, 0, 0, 0, 0, 0, 0});
  state2.time = end_time;
  auto si = robot->GetSpaceInformation();
  auto start = si->allocState();
  auto goal = si->allocState();
  robot->EigenToState(state1, start);
  robot->EigenToState(state2, goal);
  auto path = std::make_shared<ompl::geometric::PathGeometric>(si, start, goal);
  path->interpolate();
  return std::make_pair(robot, path);
}

TEST(TimeBasedPlanningKukaTest, DynamicObstaclePositionTest) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Create static_obstacles
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> static_obstacles;
  static_obstacles.push_back(createBox(State3d(+0.0,0,0), 0.3, 0.3, 0.8));
  auto floor = createFloor(-0.255);
  static_obstacles.push_back(floor);
  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle : static_obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(State3d::Zero());

  ////////////////////////////////////////////////////////////////////////////////
  ////Create dynamic_obstacles
  ////////////////////////////////////////////////////////////////////////////////

  std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> dynamic_obstacles;
  auto dynamic_obstacle1 = MakeStraightRobotPath({+1.0, +1.0}, {+1.0, -1.0}, kObstacleEndTime, world, static_obstacles);
  auto dynamic_obstacle2 = MakeStraightRobotPath({-1.0, +1.0}, {-1.0, -1.0}, kObstacleEndTime, world, static_obstacles);

  std::vector<dart::dynamics::SkeletonPtr> all_obstacles;
  for(const auto& obstacle : static_obstacles) {
    all_obstacles.push_back(obstacle);
  }
  all_obstacles.push_back(dynamic_obstacle1.first->GetSkeleton());
  all_obstacles.push_back(dynamic_obstacle2.first->GetSkeleton());

  ////////////////////////////////////////////////////////////////////////////////
  ////Create Robots
  ////////////////////////////////////////////////////////////////////////////////

  auto robot_in_time = MakeRobot<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(world, all_obstacles);
  robot_in_time->AddDynamicalObstacle(dynamic_obstacle1);
  robot_in_time->AddDynamicalObstacle(dynamic_obstacle2);

  const double kEndTime = robot_in_time->GetTMax();
  auto factor = robot_in_time->GetSpaceInformation();
  auto start_state = MakeState({-1.0, -1.0, 0.0, -1.0, 0.0, +1.57, -1, 2, 0.24, -0.21});
  auto goal_state = MakeState({+1.0, -1.0, 0.0, 0.0, 0.0, -1.57, +1, 2, 0.24, -0.21});
  auto start = factor->allocState();
  auto goal = factor->allocState();

  robot_in_time->EigenToState(start_state, start);
  robot_in_time->EigenToState(goal_state, goal);
  robot_in_time->TimeToState(0.0, start);
  robot_in_time->TimeToState(kEndTime, goal);

  factor->printState(start);
  factor->printState(goal);

  auto ompl_path = std::make_shared<ompl::geometric::PathGeometric>(factor, start, goal);
  ompl_path->interpolate();

  EigenPath path(robot_in_time, ompl_path);
  path.GetConfigAt(0.0);

  auto p1 = path.GetConfigAt(0.0);
  auto p2 = path.GetConfigAt(10.0);
  auto p3 = path.GetConfigAt(20.0);

  EXPECT_EQ(p1.time, 0.0);
  EXPECT_EQ(p2.time, 10.0);
  EXPECT_EQ(p3.time, 20.0);

  EXPECT_TRUE(robot_in_time->IsValid(start));
  EXPECT_FALSE(robot_in_time->IsValid(goal));

  auto tmp = factor->allocState();

  robot_in_time->EigenToState(goal_state, tmp);
  robot_in_time->TimeToState(0.0 * kEndTime, tmp);
  EXPECT_TRUE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(0.5 * kEndTime, tmp);
  EXPECT_TRUE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(0.7 * kEndTime, tmp);
  EXPECT_TRUE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(0.9 * kEndTime, tmp);
  EXPECT_FALSE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(1.0 * kEndTime, tmp);
  EXPECT_FALSE(robot_in_time->IsValid(tmp));

  robot_in_time->EigenToState(start_state, tmp);
  robot_in_time->TimeToState(0.0 * kEndTime, tmp);
  EXPECT_TRUE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(0.5 * kEndTime, tmp);
  EXPECT_TRUE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(0.7 * kEndTime, tmp);
  EXPECT_TRUE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(0.9 * kEndTime, tmp);
  EXPECT_FALSE(robot_in_time->IsValid(tmp));
  robot_in_time->TimeToState(1.0 * kEndTime, tmp);
  EXPECT_FALSE(robot_in_time->IsValid(tmp));

  factor->freeState(tmp);
  factor->freeState(goal);
  factor->freeState(start);
}

void TestConnection(
    const std::shared_ptr<TimeBasedMobileKukaRobotTaskSpace>& robot,
    double x_start, double x_goal, 
    double t_start, double t_goal, bool expect_true = true) 
{
  auto factor = robot->GetSpaceInformation();
  const double kEndTime = robot->GetTMax();

  auto start_state = MakeState({x_start, -0.0, 0.0, -1.0, 0.0, +1.57, -1, 2, 0.24, -0.21});
  auto goal_state = MakeState({x_goal, -0.0, 0.0, 0.0, 0.0, -1.57, +1, 2, 0.24, -0.21});
  auto start = factor->allocState();
  auto goal = factor->allocState();

  robot->EigenToState(start_state, start);
  robot->EigenToState(goal_state, goal);
  robot->TimeToState(t_start, start);
  robot->TimeToState(t_goal, goal);

  auto validator = std::make_shared<MotionValidatorTimeBased>(factor, robot, robot->GetVMax());
  auto extent = factor->getStateSpace()->getMaximumExtent();
  auto dist = factor->getStateSpace()->distance(start, goal);
  auto count_factor = factor->getStateSpace()->getValidSegmentCountFactor();
  auto segment_count = factor->getStateSpace()->validSegmentCount(start, goal);
  auto segment_length = factor->getStateSpace()->getLongestValidSegmentLength();
  auto segment_fraction = factor->getStateSpace()->getLongestValidSegmentFraction();
  EXPECT_GT(dist, std::fabs(x_start - x_goal));
  EXPECT_GT(extent, 0.0);
  EXPECT_GT(count_factor, 0u);
  EXPECT_GT(segment_count, 2u);
  EXPECT_GT(segment_length, 0.0);
  EXPECT_GT(segment_fraction, 0.0);
  EXPECT_GT(ceil(dist/factor->getStateSpace()->getLongestValidSegmentLength()), 1.0);

  if(expect_true) {
    EXPECT_TRUE(validator->checkMotion(start, goal));
  } else {
    EXPECT_FALSE(validator->checkMotion(start, goal));
  }
  factor->freeState(goal);
  factor->freeState(start);
}

TEST(TimeBasedPlanningKukaTest, MotionValidatorDynamicObstacleTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create dynamic_obstacles
  ////////////////////////////////////////////////////////////////////////////////

  std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> dynamic_obstacles;
  dynamic_obstacles.push_back(MakeStraightRobotPath({+0.0, -1.0}, {+0.0, +1.0}, kObstacleEndTime, world, {}));
  dynamic_obstacles.push_back(MakeStraightRobotPath({+1.0, +1.0}, {+1.0, +1.0}, kObstacleEndTime, world, {}));

  std::vector<dart::dynamics::SkeletonPtr> all_obstacles;
  for(const auto& dynamic_obstacle: dynamic_obstacles) {
    all_obstacles.push_back(dynamic_obstacle.first->GetSkeleton());
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////Create Robots
  ////////////////////////////////////////////////////////////////////////////////

  auto robot_in_time = MakeRobot<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(world, all_obstacles);
  for(const auto& dynamic_obstacle: dynamic_obstacles) {
    robot_in_time->AddDynamicalObstacle(dynamic_obstacle);
  }

  auto factor = robot_in_time->GetSpaceInformation();
  factor->setup();

  const double kEndTime = robot_in_time->GetTMax();

  TestConnection(robot_in_time, -1.0, +1.0, 0.0, kEndTime, false);
  TestConnection(robot_in_time, -1.0, +1.0, 0.5*kEndTime, kEndTime, false);

  TestConnection(robot_in_time, -0.0, +1.0, 0.0, 0.5*kEndTime, true);
  TestConnection(robot_in_time, -1.0, +0.0, 0.5*kEndTime, kEndTime, true);

  TestConnection(robot_in_time, -0.0, +0.0, 0.8*kEndTime, kEndTime, true);
  TestConnection(robot_in_time, -1.0, -1.0, 0.0, kEndTime, true);
  TestConnection(robot_in_time, +1.0, +1.0, 0.0, kEndTime, true);
}

TEST(TimeBasedPlanningKukaTest, MotionValidatorDynamicObstaclePropagateMotionTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);
  std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> dynamic_obstacles;
  dynamic_obstacles.push_back(MakeStraightRobotPath({+0.0, -1.0}, {+0.0, +1.0}, kObstacleEndTime, world, {}));
  dynamic_obstacles.push_back(MakeStraightRobotPath({+1.0, +1.0}, {+1.0, +1.0}, kObstacleEndTime, world, {}));

  std::vector<dart::dynamics::SkeletonPtr> all_obstacles;
  for(const auto& dynamic_obstacle: dynamic_obstacles) {
    all_obstacles.push_back(dynamic_obstacle.first->GetSkeleton());
  }

  auto robot_in_time = MakeRobot<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(world, all_obstacles);
  for(const auto& dynamic_obstacle: dynamic_obstacles) {
    robot_in_time->AddDynamicalObstacle(dynamic_obstacle);
  }

  auto factor = robot_in_time->GetSpaceInformation();
  factor->setup();

  const double kEndTime = robot_in_time->GetTMax();
  ////////////////////////////////////////////////////////////////////////////////
  ////Check propagation
  ////////////////////////////////////////////////////////////////////////////////
  auto start_state = MakeState({-1.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0});
  auto goal_state =  MakeState({+1.0, 0.0, 0, 0, 0, 0, 0, 0, 0, 0});
  auto start = factor->allocState();
  auto goal = factor->allocState();

  robot_in_time->EigenToState(start_state, start);
  robot_in_time->EigenToState(goal_state, goal);
  robot_in_time->TimeToState(0.0, start);
  robot_in_time->TimeToState(kEndTime, goal);

  auto validator = std::make_shared<MotionValidatorTimeBased>(factor, robot_in_time, robot_in_time->GetVMax());
  auto states = validator->propagateMotion(start, goal);
  EXPECT_GT(states.size(), 2u);
  factor->printState(states.back());
}
