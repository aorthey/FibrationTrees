#include "robots/DiskRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "DartHelper.hpp"

dart::dynamics::SkeletonPtr DiskRobot::MakeSkeleton() {
  return createCylinder(State3d(0,0,0), radius_, height_);
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
