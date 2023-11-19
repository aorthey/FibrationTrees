#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/State.h>
#include <ompl/multilevel/datastructures/Projection.h>

Eigen::Vector3d ProjectStateToEigenVector3d(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::State* state) {
  ompl::base::State *projected_state = projection->getBase()->allocState();
  projection->project(state, projected_state);
  double *state_R3 = projected_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  Eigen::Vector3d v;
  v << state_R3[0], state_R3[1], state_R3[2];
  projection->getBase()->freeState(projected_state);
  return v;
}
