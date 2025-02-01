#include "robots/SphereRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "DartHelper.hpp"
#include "yaml/SkeletonHelpers.hpp"

dart::dynamics::SkeletonPtr SphereRobot::MakeSkeleton(const YAML::Node& node) {
  if(node["radius"]) {
    radius_ = node["radius"].as<double>();
  }
  auto skeleton = createSphere(radius_);
  SetSkeletonLowerLimits(skeleton, node, 3u);
  SetSkeletonUpperLimits(skeleton, node, 3u);
  return skeleton;
}

void SphereRobot::SetLimits(const std::pair<State3d, State3d>& limits) {
  const auto& lb = limits.first;
  const auto& ub = limits.second;
  ompl::base::RealVectorBounds bounds(3);
  for(size_t k =0; k < 3; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  GetSpaceInformation()->getStateSpace()->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  skeleton_->setPositionLowerLimits(lb);
  skeleton_->setPositionUpperLimits(ub);
}
