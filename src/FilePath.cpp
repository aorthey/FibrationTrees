#include "FilePath.hpp"

#include <cstdio>

std::string GetMainFolder() {
  return std::string(PROGRAM_FOLDER_PATH) + "/";
}
std::string GetDataFolder() {
  return GetMainFolder() + "data/";
}
