#pragma once

#include <yaml-cpp/yaml.h>
#include <dart/dart.hpp>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "robots/Robot.hpp"

ompl::base::ProblemDefinitionPtr MakeProblemDefinitionFromYamlFilename(
    const std::string& filename, const dart::simulation::WorldPtr& world, 
    const ompl::multilevel::FactoredSpaceInformationPtr& root_factor,
    const RobotPtr& root_robot, const std::unordered_map<std::string, RobotPtr>& child_robots);
