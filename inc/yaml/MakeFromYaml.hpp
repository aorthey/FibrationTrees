#pragma once

#include <ompl/base/ProblemDefinition.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include "yaml-cpp/yaml.h"
#include "robots/Robot.hpp"

std::vector<dart::dynamics::SkeletonPtr> MakeObstaclesFromYamlFilename(const std::string& filename);

RobotPtr MakeRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);

RobotPtr MakeRootRobotFromYamlFilename(const std::string& filename,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);

std::unordered_map<std::string, RobotPtr> MakeChildRobotsFromYamlFilename(const std::string& filename, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);

std::string GetRootRobotNameFromYamlFilename(const std::string& filename);

std::tuple<ompl::multilevel::FactoredSpaceInformationPtr, ompl::base::ProblemDefinitionPtr, RobotPtr, std::unordered_map<std::string, RobotPtr>,
std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> >
MakeFactoredSpaceInformationFromYamlFilename(const std::string& filename, const dart::simulation::WorldPtr& world);

