#include <gtest/gtest.h>
#include <filesystem>

#include "yaml/MakeFromYaml.hpp"
#include "FilePath.hpp"
#include "testing/TestHelpers.hpp"

std::vector<std::string> GetScenarios() {
  return GetFilesRecursively(GetDataFolder() + "scenarios");
}

class ScenarioLoaderTest  : public testing::TestWithParam<std::string> {
};

TEST_P(ScenarioLoaderTest, LoadFromYamlTest) {
  std::string scenario_filename = GetParam();
  dart::math::Random::setSeed(0);
  dart::simulation::WorldPtr world(new dart::simulation::World);
  EXPECT_NO_THROW(
      MakeFactoredSpaceInformationFromYamlFilename(scenario_filename, world);
  );
}

INSTANTIATE_TEST_SUITE_P(
    ScenarioTests,
    ScenarioLoaderTest,
    ::testing::ValuesIn(GetScenarios()),
    FileStemFromParam
);

