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

std::pair<RobotPtr, ompl::base::PathPtr> MakeStraightRobotPath(
    const std::vector<double>& start_xy,
    const std::vector<double>& goal_xy,
    const dart::simulation::WorldPtr& world, 
    const std::vector<dart::dynamics::SkeletonPtr>& static_obstacles
    ) {
  auto robot = MakeRobot<MobileKukaRobot>(world, static_obstacles);

  auto state1 = MakeState({start_xy.at(0), start_xy.at(1), 0.0, -1.0, 0.0, +1.57, -1, 2, 0.24, -0.21});
  auto state2 = MakeState({goal_xy.at(0), goal_xy.at(1), 0.0, 0.0, 0.0, -1.57, +1, 2, 0.24, -0.21});
  auto si = robot->GetSpaceInformation();
  auto start = si->allocState();
  auto goal = si->allocState();
  robot->EigenToState(state1, start);
  robot->EigenToState(state2, goal);
  auto path = std::make_shared<ompl::geometric::PathGeometric>(si, start, goal);
  path->interpolate();
  return std::make_pair(robot, path);
}

TEST(TimeBasedPlanningKuka, TestDynamicObstaclePositions) {

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
  auto dynamic_obstacle = MakeStraightRobotPath({+1.0, +1.0}, {+1.0, -1.0}, world, static_obstacles);

  std::vector<dart::dynamics::SkeletonPtr> all_obstacles;
  for(const auto& obstacle : static_obstacles) {
    all_obstacles.push_back(obstacle);
  }
  all_obstacles.push_back(dynamic_obstacle.first->GetSkeleton());

  ////////////////////////////////////////////////////////////////////////////////
  ////Create Robots
  ////////////////////////////////////////////////////////////////////////////////

  auto robot_in_time = MakeRobot<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(world, all_obstacles);
  robot_in_time->AddDynamicalObstacle(dynamic_obstacle);

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
  auto p2 = path.GetConfigAt(0.5);
  auto p3 = path.GetConfigAt(1.0);

  EXPECT_EQ(p1.time, 0.0);
  EXPECT_EQ(p2.time, 0.5 * kEndTime);
  EXPECT_EQ(p3.time, kEndTime);

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

  factor->freeState(tmp);
  factor->freeState(goal);
  factor->freeState(start);
}

