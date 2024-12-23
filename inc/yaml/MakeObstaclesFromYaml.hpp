#include "yaml-cpp/yaml.h"

#include <dart/dart.hpp>

#include <ompl/base/Path.h>
#include <ompl/util/ClassForward.h>

OMPL_CLASS_FORWARD(Robot);
OMPL_CLASS_FORWARD(Path);

std::vector<dart::dynamics::SkeletonPtr> MakeObstaclesFromYamlFilename(const std::string& filename);

std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> 
MakeDynamicObstaclesFromYamlFilename(const std::string& filename, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& static_obstacles);
