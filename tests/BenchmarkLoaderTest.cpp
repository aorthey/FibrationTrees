#include <gtest/gtest.h>
#include <filesystem>

#include "yaml/MakeFromYaml.hpp"
#include "FilePath.hpp"
#include "testing/TestHelpers.hpp"
#include "input/FibrationTreesBenchmakerExecuter.hpp"

std::string UniqueFileNameFromParam(const testing::TestParamInfo<std::string>& info) {
    std::filesystem::path p(info.param);
    // Use relative path from data folder, replace / and . with _
    // (must be valid C++ identifier: letters, digits, _)
    std::string rel = p.lexically_relative(GetDataFolder() + "benchmarks").string();
    std::replace(rel.begin(), rel.end(), '/', '_');
    std::replace(rel.begin(), rel.end(), '\\', '_');
    std::replace(rel.begin(), rel.end(), '.', '_');
    // Optional: trim extension again if needed
    if (rel.ends_with("_yaml")) rel = rel.substr(0, rel.size() - 5);
    return rel;
}

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
    UniqueFileNameFromParam
);
