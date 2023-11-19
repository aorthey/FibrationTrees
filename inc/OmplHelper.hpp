#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/multilevel/datastructures/Projection.h>

Eigen::VectorXd StateToEigenVectorXd(const ompl::base::SpaceInformationPtr& si, const ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  int N = si->getStateDimension();
  Eigen::VectorXd v(N);
  for(size_t k = 0; k < N; k++) {
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

