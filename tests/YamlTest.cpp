#include <gtest/gtest.h>

#include <iostream>
#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "FilePath.hpp"
#include "Common.hpp"
#include "robots/CubeRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "yaml/MakeFromYaml.hpp"
#include "yaml/MakeObstaclesFromYaml.hpp"

TEST(YamlTest, ObstacleTest) {
  const auto filename = GetMainFolder() + "tests/data/test_scenario.yaml";
  YAML::Node config = YAML::LoadFile(filename);
  auto obstacles = MakeObstaclesFromYamlFilename(filename);
  EXPECT_EQ(obstacles.size(), 3u);
}

TEST(YamlTest, WrongObstacleTest) {
  const auto filename = GetMainFolder() + "tests/data/wrong_obstacles.yaml";
  YAML::Node config = YAML::LoadFile(filename);
  EXPECT_ANY_THROW(MakeObstaclesFromYamlFilename(filename));
}

TEST(YamlTest, FullScenarioTest) {
  dart::math::Random::setSeed(0);
  const auto filename = GetMainFolder() + "tests/data/full_scenario.yaml";
  dart::simulation::WorldPtr world(new dart::simulation::World);
  EXPECT_NO_THROW(
      MakeFactoredSpaceInformationFromYamlFilename(filename, world);
  );
}

TEST(YamlTest, LoadCubeRobotWithLimitsTest) {
  const auto filename = GetMainFolder() + "tests/data/cube_robot.yaml";
  YAML::Node config = YAML::LoadFile(filename);

  EXPECT_TRUE(config["name"]);
  EXPECT_TRUE(config["upper_limits"]);
  EXPECT_TRUE(config["lower_limits"]);
  EXPECT_TRUE(config["size"]);

  EXPECT_EQ(config["name"].as<std::string>(), "CubeRobot");

  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto robot = MakeRobot<CubeRobot>(world, {}, config);

  auto si = robot->GetSpaceInformation();
  EXPECT_EQ(si->getStateSpace()->getType(), ompl::base::StateSpaceType::STATE_SPACE_SE2);

  auto space = si->getStateSpace()->as<ompl::base::SE2StateSpace>();
  auto bounds = space->getBounds();

  EXPECT_EQ(bounds.low.size(), 2u);
  EXPECT_EQ(bounds.high.size(), 2u);

  EXPECT_NEAR(bounds.low.at(0), -6.0, Epsilon);
  EXPECT_NEAR(bounds.low.at(1), -7.0, Epsilon);
  EXPECT_NEAR(bounds.high.at(0), +8.0, Epsilon);
  EXPECT_NEAR(bounds.high.at(1), +9.0, Epsilon);

  EXPECT_EQ(robot->GetSkeleton()->getNumShapeNodes(), 1u);
  auto shape_node = robot->GetSkeleton()->getShapeNode(0);
  auto shape = shape_node->getShape();

  EXPECT_NEAR(shape->getVolume(), 1.0*2.0*3.0, Epsilon);

  auto box_shape = std::dynamic_pointer_cast<dart::dynamics::BoxShape>(shape);
  EXPECT_NE(box_shape, nullptr);

  auto size = box_shape->getSize();
  EXPECT_EQ(size.size(), 3u);

  auto expected_size = config["size"].as<std::vector<double>>();
  EXPECT_EQ(expected_size.size(), 3u);

  EXPECT_NEAR(size[0], 1.0, Epsilon);
  EXPECT_NEAR(size[1], 2.0, Epsilon);
  EXPECT_NEAR(size[2], 3.0, Epsilon);
  EXPECT_NEAR(size[0], expected_size.at(0), Epsilon);
  EXPECT_NEAR(size[1], expected_size.at(1), Epsilon);
  EXPECT_NEAR(size[2], expected_size.at(2), Epsilon);
}
