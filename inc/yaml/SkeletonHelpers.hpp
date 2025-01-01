#pragma once

#include <dart/dart.hpp>
#include <yaml-cpp/yaml.h>

void SetSkeletonLowerLimits(const dart::dynamics::SkeletonPtr& skeleton, const YAML::Node& node, const size_t& dof);
void SetSkeletonUpperLimits(const dart::dynamics::SkeletonPtr& skeleton, const YAML::Node& node, const size_t& dof);
