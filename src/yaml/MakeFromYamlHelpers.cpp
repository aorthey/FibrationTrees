#include "yaml/MakeFromYamlHelpers.hpp"

#include "robots/Robot.hpp"
#include "DartHelper.hpp"

void MaybeChangeColor(const YAML::Node& node, const dart::dynamics::SkeletonPtr& skeleton) {
  if(!node["color"]) {
    return;
  }
  auto color_vector = node["color"].as<std::vector<double>>();

  if(color_vector.size() != 4) {
    throw std::out_of_range("Color needs to have four values, but has " + std::to_string(color_vector.size()));
  }
  const Eigen::Vector4d color(color_vector.data());
  changeBodyColor(skeleton, color);
}

void VerifyRobotNameExistsAndIsNotRoot(const std::string& name, const RobotPtr& root_robot, 
    const std::unordered_map<std::string, RobotPtr>& child_robots) {
  if(root_robot->GetName() == name) {
    OMPL_WARN("Cannot specify a task goal on the root space %s.", name.c_str());
    throw std::runtime_error("No root space allowed for task goal.");
  }
  if(child_robots.find(name) == child_robots.end()) {
    OMPL_ERROR("Could not find child robot %s.", name.c_str());
    throw std::domain_error("Child robot " + name + " does not exist.");
  }
}



