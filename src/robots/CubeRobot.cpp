#include "robots/CubeRobot.hpp"

#include "DartHelper.hpp"
#include "yaml/SkeletonHelpers.hpp"

dart::dynamics::SkeletonPtr CubeRobot::MakeSkeleton(const YAML::Node& node) {
  if(node["size"]) {
    const auto size = node["size"].as<std::vector<double>>();
    if(size.size() != 3) {
      throw std::domain_error("CubeRobot size must be 3 (width, length, height).");
    }
    width_ = size[0];
    length_ = size[1];
    height_ = size[2];
  }

  auto skeleton = createPlanarBox(State3d(0, 0, 0.5*height_), width_, length_, height_);

  SetSkeletonLowerLimits(skeleton, node, 2u);
  SetSkeletonUpperLimits(skeleton, node, 2u);
  return skeleton;
}
