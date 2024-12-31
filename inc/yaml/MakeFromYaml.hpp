#pragma once

#include <ompl/base/ProblemDefinition.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <yaml-cpp/yaml.h>

#include "robots/Robot.hpp"

std::tuple<
  ompl::multilevel::FactoredSpaceInformationPtr, 
  ompl::base::ProblemDefinitionPtr, 
  RobotPtr, 
  std::unordered_map<std::string, RobotPtr>, 
  std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> >
MakeFactoredSpaceInformationFromYamlFilename(const std::string& filename, const dart::simulation::WorldPtr& world);
