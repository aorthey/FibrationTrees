#include <gtest/gtest.h>
#include <filesystem>

#include "yaml/MakeFromYaml.hpp"
#include "FilePath.hpp"
#include "testing/TestHelpers.hpp"
#include "input/FibrationTreesBenchmakerExecuter.hpp"

std::vector<std::string> GetBenchmarks() {
  return GetFilesRecursively(GetDataFolder() + "benchmarks");
}

class BenchmarkLoaderTest  : public testing::TestWithParam<std::string> {
};

TEST_P(BenchmarkLoaderTest, LoadFromYamlTest) {
  std::string benchmark_filename = GetParam();

  const char* testArgv[] = {
        "BenchmarkLoaderTest",
        "--dry",
        benchmark_filename.c_str()
    };
  int testArgc = 3;

  EXPECT_EQ(FibrationTreesBenchmakerExecuter(testArgc, testArgv), 0u);
}

INSTANTIATE_TEST_SUITE_P(
    BenchmarkTests,
    BenchmarkLoaderTest,
    ::testing::ValuesIn(GetBenchmarks()),
    FileStemFromParam
);


