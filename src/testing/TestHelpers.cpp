#include "testing/TestHelpers.hpp"

#include <filesystem>

std::vector<std::string> GetFilesRecursively(const std::string& path) {
  std::vector<std::string> files;
  if (!std::filesystem::exists(path)) return files;
  for (const auto& entry : std::filesystem::recursive_directory_iterator(path)) {
    if (std::filesystem::is_regular_file(entry)) {
      files.push_back(entry.path().string());
    }
  }
  std::sort(files.begin(), files.end());
  return files;
}

std::string FileStemFromParam(const testing::TestParamInfo<std::string>& info) {
  return std::filesystem::path(info.param).stem();
}
std::string FileStemFromParamPairStringDouble(const testing::TestParamInfo<std::pair<std::string, double>>& info) {
  return std::filesystem::path(info.param.first).stem();
}
