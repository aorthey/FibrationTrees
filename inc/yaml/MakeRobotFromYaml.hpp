#pragma once

#include <yaml-cpp/yaml.h>

#include <dart/dart.hpp>

#include <ompl/util/ClassForward.h>

OMPL_CLASS_FORWARD(Robot);

RobotPtr MakeAtomicRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);

RobotPtr MakeMultiRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles,
    std::unordered_map<std::string, RobotPtr> child_robots);

RobotPtr MakeAnyRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles,
    std::unordered_map<std::string, RobotPtr> child_robots);

std::unordered_map<std::string, RobotPtr> MakeChildRobotsFromYamlFilename(const std::string& filename, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);

RobotPtr MakeRootRobotFromYamlFilename(const std::string& filename,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles, 
    std::unordered_map<std::string, RobotPtr> child_robots);
