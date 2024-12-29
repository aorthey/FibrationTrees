#include <gtest/gtest.h>

#include <iostream>
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
