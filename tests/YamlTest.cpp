#include <gtest/gtest.h>

#include <iostream>
#include "FilePath.hpp"
#include "yaml/MakeFromYaml.hpp"

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
  //const auto filename = GetMainFolder() + "tests/data/full_scenario.yaml";
  dart::math::Random::setSeed(0);
  const auto filename = GetDataFolder() + "scenarios/02_VerticalMaze.yaml";

  dart::simulation::WorldPtr world(new dart::simulation::World);
  EXPECT_NO_THROW(
      auto factor = MakeFactoredSpaceInformationFromYamlFilename(filename, world);
      //MakeProblemDefinitionFromYamlFilename(filename, factor);
  );
}
