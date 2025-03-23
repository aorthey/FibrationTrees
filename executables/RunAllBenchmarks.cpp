#include "yaml/MakeFromYaml.hpp"
#include "FilePath.hpp"
#include "testing/TestHelpers.hpp"
#include "input/FibrationTreesBenchmakerExecuter.hpp"

std::vector<std::string> GetBenchmarks() {
  return GetFilesRecursively(GetDataFolder() + "benchmarks/03/");
}

int main() {
  auto benchmarks = GetBenchmarks();
  for(const auto& benchmark : benchmarks) {
    const char* testArgv[] = {
          "RunAllBenchmark",
          benchmark.c_str()
      };
    int testArgc = 2;

    FibrationTreesBenchmakerExecuter(testArgc, testArgv);
  }
}
