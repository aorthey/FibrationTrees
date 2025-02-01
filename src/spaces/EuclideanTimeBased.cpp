#include "spaces/EuclideanTimeBased.hpp"

#include "EigenPath.hpp"
#include "Common.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/TimeStateSpace.h>

EuclideanTimeBased::EuclideanTimeBased(const RobotPtr& robot, double tMax, double vMax) :
  robot_(robot), vMax_(vMax) {
  auto numDofs = robot->GetSkeleton()->getNumDofs();
  setName("EuclideanTimeBased" + getName());
  auto RN = std::make_shared<ompl::base::RealVectorStateSpace>(numDofs);
  auto T = std::make_shared<ompl::base::TimeStateSpace>();
  addSubspace(RN, 1.0);
  addSubspace(T, 1.0);
  
  T->setBounds(0.0, tMax);
  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();

  ompl::base::RealVectorBounds bounds(numDofs);
  for(size_t k = 0; k < numDofs; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  RN->setBounds(bounds);
  lock();
}

double EuclideanTimeBased::distance(const ompl::base::State *from, const ompl::base::State *to) const {
  const auto from_vector = robot_->StateToEigen(from);
  const auto to_vector = robot_->StateToEigen(to);
  if(!IsReachableInTime(from_vector, to_vector, vMax_)) {
    return Inf;
  }

  return ompl::base::CompoundStateSpace::distance(from, to);
}

EuclideanTimeBased::~EuclideanTimeBased() {
}

bool EuclideanTimeBased::isMetricSpace() const
{
  return false;
}
