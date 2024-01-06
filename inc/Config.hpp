#include "yaml-cpp/yaml.h"

#include <iostream>

template <typename T>
T ReadConfigVariable(const std::string& name) {
  YAML::Node config = YAML::LoadFile("/home/aorthey/git/FibrationTrees/config.yaml");
  return config[name].as<T>();
}

