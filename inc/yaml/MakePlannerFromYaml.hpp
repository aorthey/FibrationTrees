#pragma once

#include <ompl/base/Planner.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include "yaml-cpp/yaml.h"

#include "robots/Robot.hpp"

namespace ompl {
  namespace base {
    OMPL_CLASS_FORWARD(Planner);
  }
}

ompl::base::PlannerPtr MakePlannerFromYaml(const YAML::Node& node, const std::string& planner_name, 
    const ompl::multilevel::FactoredSpaceInformationPtr& factor, const std::unordered_map<std::string, RobotPtr>& child_robots);
