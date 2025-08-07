#include "robots/TimeBasedMobileKukaBase.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

TimeBasedMobileKukaBase::TimeBasedMobileKukaBase(float vMax, float tMax) :
  vMax_(vMax), tMax_(tMax) {}

ompl::multilevel::FactoredSpaceInformationPtr TimeBasedMobileKukaBase::MakeSpaceInformation(const RobotPtr& robot) {
  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
  auto position_space = std::make_shared<ompl::base::RealVectorStateSpace>(2);

  ompl::base::RealVectorBounds bounds(2);
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k =0; k<2; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  position_space->setBounds(bounds);

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds Time
  ////////////////////////////////////////////////////////////////////////////////
  auto time_space = std::make_shared<ompl::base::TimeStateSpace>();
  time_space->setBounds(0.0, tMax_);

  auto space = position_space + time_space;
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}

StateXd TimeBasedMobileKukaBase::StateToEigen(const ompl::base::State* state) const {
  const auto *state_R = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
  const auto *state_T = state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1);
  auto N = GetDimension();
  auto result = MakeState(N, state_R);
  result.time = state_T->position;
  return result;
}

void TimeBasedMobileKukaBase::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto *state_R = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values;
  auto *state_T = state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1);
  for(size_t k = 0; k < (size_t) v.configuration.size(); k++) {
    state_R[k] = v.configuration[k];
  }
  state_T->position = v.time;
}

float TimeBasedMobileKukaBase::GetVMax() const {
  return vMax_;
}

float TimeBasedMobileKukaBase::GetTMax() const {
  return tMax_;
}

void TimeBasedMobileKukaBase::SetVMax(float vMax) {
  vMax_ = vMax;
}

void TimeBasedMobileKukaBase::SetTMax(float tMax) {
  tMax_ = tMax;
}
