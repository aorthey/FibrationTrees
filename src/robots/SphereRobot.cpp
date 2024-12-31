#include "robots/SphereRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "DartHelper.hpp"

dart::dynamics::SkeletonPtr SphereRobot::MakeSkeleton(const YAML::Node& /*node*/) {
  return createSphere(0.01);
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
