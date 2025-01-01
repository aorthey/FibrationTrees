#include "yaml/SkeletonHelpers.hpp"

#include <ranges>
#include <vector>

#include "State.hpp"

void SetSkeletonLowerLimits(const dart::dynamics::SkeletonPtr& skeleton, const YAML::Node& node, const size_t& dof) {
  if(!node["lower_limits"]) {
    return;
  }
  std::cout << dof << std::endl;
  const auto lower_limits = node["lower_limits"].as<std::vector<double>>();
  if(lower_limits.size() != dof) {
    throw std::domain_error("CubeRobot lower_limits must be size " + std::to_string(dof) + ".");
  }
  for(size_t k = 0; k < dof; k++) {
    skeleton->setPositionLowerLimit(k, lower_limits.at(k));
  }
}
void SetSkeletonUpperLimits(const dart::dynamics::SkeletonPtr& skeleton, const YAML::Node& node, const size_t& dof) {
  if(!node["upper_limits"]) {
    return;
  }
  const auto upper_limits = node["upper_limits"].as<std::vector<double>>();
  if(upper_limits.size() != dof) {
    throw std::domain_error("CubeRobot upper_limits must be size " + std::to_string(dof) + ".");
  }
  for(size_t k = 0; k < dof; k++) {
    skeleton->setPositionUpperLimit(k, upper_limits.at(k));
  }
}


