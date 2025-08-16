#include <gtest/gtest.h>
#include <filesystem>
#include <chrono>

#include "yaml/MakeFromYaml.hpp"
#include "yaml/MakePlannerFromYaml.hpp"
#include "FilePath.hpp"
#include "testing/TestHelpers.hpp"
#include "OmplHelper.hpp"

typedef std::pair<std::string, double> TestParameter;

std::vector<TestParameter> GetScenarios(const std::string& path_name, const double timeout) {
  auto files = GetFilesRecursively(GetDataFolder() + path_name);
  std::vector<TestParameter> output;
  for(const auto& file : files) {
    if(file.find("_None") == std::string::npos) {
      output.push_back(std::make_pair(file, timeout));
    }
  }
  return output;
}

std::vector<TestParameter> GetEasyScenarios() {
  return GetScenarios("scenarios/01_multirobots_easy/", 600.0);
}

std::vector<TestParameter> GetHardScenarios() {
  return GetScenarios("scenarios/02_multirobots_hard/", 1800.0);
}

std::vector<TestParameter> GetTaskSpaceScenarios() {
  return GetScenarios("scenarios/03_taskspace/", 1800.0);
}

void VerifyScenario(const std::string& filename, double timeout) {
  std::cout << "Testing scenario: " << filename << std::endl;
  dart::math::Random::setSeed(0);
  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto [factor, pdef, root_robot, child_robots, dynamic_obstacles] = MakeFactoredSpaceInformationFromYamlFilename(filename, world);
  auto planner = MakePlannerFromYaml(filename, "FibrationRrt", factor, child_robots);
  planner->setProblemDefinition(pdef);
  planner->setup();

  auto ptc = TimeOrSolutionPtc(pdef, timeout);
  ompl::base::PlannerStatus status = planner->solve(ptc);

  EXPECT_TRUE(status);
  if(status) {
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    auto states = pgeo.getStates();
    EXPECT_GT(states.size(), 1u);
  }
}

class PlanForAllScenariosIntegrationTest : public testing::TestWithParam<TestParameter> {
  protected:
    std::stringstream outputBuffer;
    std::streambuf* cout_sbuf;

    void SetUp() override {
      cout_sbuf = std::cout.rdbuf();
      std::cout.rdbuf(outputBuffer.rdbuf());
    }

    void TearDown() override {
      std::cout.rdbuf(cout_sbuf);

      const std::string& output = outputBuffer.str();
      size_t secondLastNewline = output.rfind('\n');
      if (secondLastNewline != std::string::npos) {
        size_t lastNewline = output.rfind('\n', secondLastNewline - 1);
        std::cout << output.substr(lastNewline + 1, secondLastNewline - lastNewline - 1) << std::endl;
      }
    }
};

TEST_P(PlanForAllScenariosIntegrationTest, PlanPathAndCheckValidity) {
  auto start_time = std::chrono::high_resolution_clock::now();

  auto [filename, timeout] = GetParam();
  VerifyScenario(filename, timeout);

  auto stop_time = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::seconds>(stop_time - start_time);

  auto name = std::filesystem::path(filename).stem();
  std::cout << ">>>>>>>> " << name << " finished: " << duration.count() << " seconds." << std::endl;
}

INSTANTIATE_TEST_SUITE_P(
    ScenarioTests1,
    PlanForAllScenariosIntegrationTest,
    ::testing::ValuesIn(GetEasyScenarios()),
    FileStemFromParamPairStringDouble
);

INSTANTIATE_TEST_SUITE_P(
    ScenarioTests2,
    PlanForAllScenariosIntegrationTest,
    ::testing::ValuesIn(GetHardScenarios()),
    FileStemFromParamPairStringDouble
);

INSTANTIATE_TEST_SUITE_P(
    ScenarioTests3,
    PlanForAllScenariosIntegrationTest,
    ::testing::ValuesIn(GetTaskSpaceScenarios()),
    FileStemFromParamPairStringDouble
);
