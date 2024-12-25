#include "yaml-cpp/yaml.h"

#include <ompl/util/ClassForward.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <dart/dart.hpp>

OMPL_CLASS_FORWARD(Robot);

void MakeProjectionsFromYamlFilename(const std::string& filename, const dart::simulation::WorldPtr& world, 
    const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& root_robot, 
    const std::unordered_map<std::string, RobotPtr>& child_robots);
