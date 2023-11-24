#include "OmplHelper.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

Eigen::VectorXd StateToEigenVectorXd(const int Ndimension, const ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  Eigen::VectorXd v(Ndimension);
  for(size_t k = 0; k < Ndimension; k++) {
    v[k] = state_R[k];
  }
  return v;
}

Eigen::VectorXd StateToEigenVectorXd(const ompl::base::SpaceInformation* si, const ompl::base::State* state) {
  return StateToEigenVectorXd(si->getStateDimension(), state);
}

Eigen::VectorXd StateToEigenVectorXd(const ompl::base::SpaceInformationPtr& si, const ompl::base::State* state) {
  return StateToEigenVectorXd(si->getStateDimension(), state);
}

Eigen::Vector3d StateToEigenVector3d(const ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  Eigen::Vector3d v;
  for(size_t k = 0; k < 3; k++) {
    v[k] = state_R[k];
  }
  return v;
}

Eigen::Vector3d ProjectStateToEigenVector3d(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::State* state) {
  ompl::base::State *projected_state = projection->getBase()->allocState();
  projection->project(state, projected_state);
  double *state_R3 = projected_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  Eigen::Vector3d v;
  v << state_R3[0], state_R3[1], state_R3[2];
  projection->getBase()->freeState(projected_state);
  return v;
}

Eigen::VectorXd LiftStateToEigenVectorXd(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::State* base_state) {
  ompl::base::State *projected_state = projection->getBundle()->allocState();
  projection->lift(base_state, projected_state);
  double *state_R = projected_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  int N = projection->getBundle()->getDimension();
  Eigen::VectorXd v = Eigen::VectorXd::Zero(N);
  for(size_t k = 0; k < N; k++) {
    v[k] = state_R[k];
  }
  projection->getBase()->freeState(projected_state);
  return v;
}

void EigenVector3dToState(const Eigen::Vector3d& v, ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for(size_t k = 0; k < 3; k++) {
    state_R[k] = v[k];
  }
}

void EigenVectorXdToState(const Eigen::VectorXd& v, ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for(size_t k = 0; k < v.size(); k++) {
    state_R[k] = v[k];
  }
}

bool SampleValidLift(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::SpaceInformationPtr& si, 
    const size_t max_iterations, const ompl::base::State *xBase, ompl::base::State *xBundle) {
  int iterations=0;
  while(iterations++ < max_iterations) {
    projection->lift(xBase, xBundle);
    if(si->getStateValidityChecker()->isValid(xBundle)) {
      return true;
    }
  }
  return false;
}

ompl::geometric::PathGeometricPtr PathFromEigenVectors(const std::vector<Eigen::VectorXd>& configs,
    const ompl::base::SpaceInformationPtr& si) {
  std::vector<const ompl::base::State*> states;
  for(const auto& config : configs) {
    auto state = si->allocState();
    EigenVectorXdToState(config, state);
    states.push_back(state);
  }
  return std::make_shared<ompl::geometric::PathGeometric>(si, states);
}
