#include "robots/TimeBasedSphereRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

#include "DartHelper.hpp"
#include "spaces/EuclideanTimeBased.hpp"
#include "validators/MotionValidatorTimeBased.hpp"

dart::dynamics::SkeletonPtr TimeBasedSphereRobot::MakeSkeleton() {
  return createSphere(0.01);
}

float TimeBasedSphereRobot::GetVMax() const {
  return vMax_;
}
float TimeBasedSphereRobot::GetTMax() const {
  return tMax_;
}

void TimeBasedSphereRobot::SetLimits(const std::pair<State3d, State3d>& limits) {
  const auto& lb = limits.first;
  const auto& ub = limits.second;
  ompl::base::RealVectorBounds bounds(3);
  for(size_t k =0; k< 3; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  auto space = GetSpaceInformation()->getStateSpace();
  space->as<ompl::base::CompoundStateSpace>()->as<ompl::base::RealVectorStateSpace>(0)->setBounds(bounds);
  skeleton_->setPositionLowerLimits(lb);
  skeleton_->setPositionUpperLimits(ub);
}

ompl::multilevel::FactoredSpaceInformationPtr TimeBasedSphereRobot::MakeSpaceInformation(const RobotPtr& robot) {
  ompl::base::StateSpacePtr space_time(new EuclideanTimeBased(robot, tMax_, vMax_));
  return std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space_time);
}

ompl::base::MotionValidatorPtr TimeBasedSphereRobot::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
  return std::make_shared<MotionValidatorTimeBased>(factor, robot, vMax_);
}

float TimeBasedSphereRobot::StateToTime(const ompl::base::State* state) const {
  return state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;
}
void TimeBasedSphereRobot::TimeToState(const float time, ompl::base::State* state) const {
  state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position = time;
}
StateXd TimeBasedSphereRobot::StateToEigen(const ompl::base::State* state) const {
  auto N = GetDimension();
  Eigen::VectorXd v(N-1);

  const auto *cstate = state->as<ompl::base::CompoundState>();
  const auto *state_RN = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const auto *state_T = cstate->as<ompl::base::TimeStateSpace::StateType>(1);

  for (unsigned int k = 0; k < N-1; k++)
  {
      v[k] = state_RN->values[k];
  }

  auto eigen_state = MakeState(v);
  eigen_state.time = state_T->position;
  return eigen_state;
}

void TimeBasedSphereRobot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.configuration.size();

  auto *cstate = state->as<ompl::base::CompoundState>();
  auto *state_RN = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
  auto *state_T = cstate->as<ompl::base::TimeStateSpace::StateType>(1);

  for (unsigned int k = 0; k < N; k++)
  {
      state_RN->values[k] = v.configuration[k];
  }

  state_T->position = v.time;
}
