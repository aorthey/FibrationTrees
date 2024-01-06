#include "robots/KukaRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

ompl::multilevel::FactoredSpaceInformationPtr EuclideanRobot::MakeSpaceInformation(const dart::dynamics::SkeletonPtr& manipulator) {
  auto numDofs = manipulator->getNumDofs();
  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(numDofs));
  ompl::base::RealVectorBounds bounds(numDofs);
  auto lb = manipulator->getPositionLowerLimits();
  auto ub = manipulator->getPositionUpperLimits();
  for(size_t k =0; k< numDofs; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  return std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
}

Eigen::VectorXd EuclideanRobot::StateToEigen(const ompl::base::State* state) const {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  auto N = GetDimension();
  Eigen::VectorXd v(N);
  for(size_t k = 0; k < N; k++) {
    v[k] = state_R[k];
  }
  return v;
}

void EuclideanRobot::EigenToState(const Eigen::VectorXd& v, ompl::base::State* state) const {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for(size_t k = 0; k < v.size(); k++) {
    state_R[k] = v[k];
  }
}

