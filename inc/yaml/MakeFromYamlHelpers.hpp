#pragma once

#include <yaml-cpp/yaml.h>

#include <dart/dart.hpp>

#include <ompl/util/ClassForward.h>

OMPL_CLASS_FORWARD(Robot);

void MaybeChangeColor(const YAML::Node& node, const dart::dynamics::SkeletonPtr& skeleton);

void VerifyRobotNameExistsAndIsNotRoot(const std::string& name, const RobotPtr& root_robot, 
    const std::unordered_map<std::string, RobotPtr>& child_robots);
