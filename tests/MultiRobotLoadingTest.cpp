#include <gtest/gtest.h>

#include <iostream>
#include <ompl/base/goals/GoalState.h>

#include "FilePath.hpp"
#include "yaml/MakeFromYaml.hpp"
#include "yaml/MakeObstaclesFromYaml.hpp"

TEST(MultiRobotLoadingTest, LoadPrioritizedMultiRobotScenario) {
  dart::math::Random::setSeed(0);
  const auto filename = GetMainFolder() + "tests/data/multi_robot_inclusion.yaml";

  dart::simulation::WorldPtr world(new dart::simulation::World);
  EXPECT_NO_THROW(
      MakeFactoredSpaceInformationFromYamlFilename(filename, world);
  );
}

TEST(MultiRobotLoadingTest, CheckValidSegmentCount) {
  dart::math::Random::setSeed(0);
  const auto filename = GetMainFolder() + "tests/data/multi_robot_decomposition_2robots.yaml";

  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto [factor, pdef, root_robot, child_robots, dynamic_obstacles] = MakeFactoredSpaceInformationFromYamlFilename(filename, world);

  factor->setup();

  auto space = factor->getStateSpace();
  EXPECT_GT(space->getMeasure(), 0.0);

  auto motion_validator = factor->getMotionValidator();

  auto start = pdef->getStartState(0);
  auto goal = pdef->getGoal()->as<ompl::base::GoalState>()->getState();

  int nd = space->validSegmentCount(start, goal);
  EXPECT_GT(nd, 100);
  OMPL_INFORM("%d", nd);
}

unsigned int ComputeValidSegmentCountBetweenStartAndGoalInYaml(const std::string& filename) {
  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto [factor, pdef, root_robot, child_robots, dynamic_obstacles] = MakeFactoredSpaceInformationFromYamlFilename(filename, world);
  factor->setup();
  auto start = pdef->getStartState(0);
  auto goal = pdef->getGoal()->as<ompl::base::GoalState>()->getState();
  return factor->getStateSpace()->validSegmentCount(start, goal);
}

TEST(MultiRobotLoadingTest, CheckValidSegmentCountMobileRobots) {
  dart::math::Random::setSeed(0);

  const auto filename1 = GetMainFolder() + "tests/data/multi_mobile_robots_2robots.yaml";
  const auto filename2 = GetMainFolder() + "tests/data/multi_mobile_robots_3robots.yaml";
  auto nd1 = ComputeValidSegmentCountBetweenStartAndGoalInYaml(filename1);
  auto nd2 = ComputeValidSegmentCountBetweenStartAndGoalInYaml(filename2);

  OMPL_INFORM("%d", nd1);
  OMPL_INFORM("%d", nd2);
  EXPECT_EQ(nd1, nd2);

}
