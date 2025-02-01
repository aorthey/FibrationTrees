#include "yaml-cpp/yaml.h"

#include <iostream>
#include "FilePath.hpp"

template <typename T>
T ReadConfigVariable(const std::string& name) {
  YAML::Node config = YAML::LoadFile(GetMainFolder() + "config.yaml");
  return config[name].as<T>();
}

