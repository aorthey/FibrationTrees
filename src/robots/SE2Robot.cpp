#include "robots/SE2Robot.hpp"

#include <ompl/base/spaces/SE2StateSpace.h>

#include "DartHelper.hpp"
#include "yaml/SkeletonHelpers.hpp"

void SE2Robot::SetLimits(const std::pair<State2d, State2d>& limits) {
  const auto& lb = limits.first;
  const auto& ub = limits.second;

  ompl::base::RealVectorBounds bounds(2);
  for(size_t k =0; k< 2; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  GetSpaceInformation()->getStateSpace()->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
  skeleton_->setPositionLowerLimits(lb);
  skeleton_->setPositionUpperLimits(ub);
}

ompl::multilevel::FactoredSpaceInformationPtr SE2Robot::MakeSpaceInformation(const RobotPtr& robot) {
  auto space = std::make_shared<ompl::base::SE2StateSpace>();

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::RealVectorBounds bounds(2);
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k =0; k<2; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->setBounds(bounds);
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}

StateXd SE2Robot::StateToEigen(const ompl::base::State* state) const {
  Eigen::VectorXd v(3);
  const auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  v[0] = state_SE2->getX();
  v[1] = state_SE2->getY();
  v[2] = state_SE2->getYaw();
  return MakeState(v);
}

void SE2Robot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  state_SE2->setX(v.configuration[0]);
  state_SE2->setY(v.configuration[1]);
  state_SE2->setYaw(v.configuration[2]);
}
