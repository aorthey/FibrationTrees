#include "yaml/MakeFromYaml.hpp"
#include "FilePath.hpp"
#include "testing/TestHelpers.hpp"
#include "input/FibrationTreesBenchmakerExecuter.hpp"

template <typename T>
std::vector<T> combine_vectors(const std::vector<T>& v1, const std::vector<T>& v2, const std::vector<T>& v3) {
    std::vector<T> result;
    result.reserve(v1.size() + v2.size() + v3.size());

    result.insert(result.end(), v1.begin(), v1.end());
    result.insert(result.end(), v2.begin(), v2.end());
    result.insert(result.end(), v3.begin(), v3.end());

    return result;
}

std::vector<std::string> GetBenchmarks() {
  // auto f1 = GetFilesRecursively(GetDataFolder() + "benchmarks/01/");
  // auto f2 = GetFilesRecursively(GetDataFolder() + "benchmarks/02/");
  // auto f3 = GetFilesRecursively(GetDataFolder() + "benchmarks/03/");
  // return combine_vectors(f1, f2, f3);
  return GetFilesRecursively(GetDataFolder() + "benchmarks/03/");
}

int main() {
  auto benchmarks = GetBenchmarks();
  std::cout << "Evaluating the following benchmarks:" << std::endl;
  for(const auto& benchmark : benchmarks) {
    std::cout << benchmark << std::endl;
  }
  for(const auto& benchmark : benchmarks) {
    const char* testArgv[] = {
          "RunAllBenchmark",
          benchmark.c_str()
      };
    int testArgc = 2;

    FibrationTreesBenchmakerExecuter(testArgc, testArgv);
  }
}
