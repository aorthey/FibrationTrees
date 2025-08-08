#pragma once

#include <iostream>
#include <vector>
#include <string>

#include "robots/Robot.hpp"

std::string ToString(const std::vector<std::string>& input);
std::string ToString(const std::vector<RobotPtr>& input);
std::string ToString(const Eigen::MatrixXd& mat);

template<typename T>
std::string ToString(const std::vector<T>& input);

template<typename T>
std::string ToString(const std::vector<T>& input) {
  std::string output = "[";
  std::string delimiter = "";
  for(const auto& element : input) {
    output += delimiter + std::to_string(element);
    delimiter = ", ";
  }
  return output;
}
