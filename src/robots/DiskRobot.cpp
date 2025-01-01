#include "robots/DiskRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "DartHelper.hpp"
#include "yaml/SkeletonHelpers.hpp"

dart::dynamics::SkeletonPtr DiskRobot::MakeSkeleton(const YAML::Node& node) {
  if(node["radius"]) {
    radius_ = node["radius"].as<double>();
  }
  if(node["height"]) {
    height_ = node["height"].as<double>();
  }

  auto skeleton = createCylinder(State3d(0,0,0), radius_, height_);
  SetSkeletonLowerLimits(skeleton, node, 2u);
  SetSkeletonUpperLimits(skeleton, node, 2u);
  return skeleton;
}

void DiskRobot::SetLimits(const std::pair<State2d, State2d>& limits) {
  const auto& lb = limits.first;
  const auto& ub = limits.second;

  ompl::base::RealVectorBounds bounds(2);
  for(size_t k =0; k< 2; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  GetSpaceInformation()->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  skeleton_->setPositionLowerLimits(lb);
  skeleton_->setPositionUpperLimits(ub);
}
