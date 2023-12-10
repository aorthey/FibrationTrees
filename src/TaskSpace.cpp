#include "TaskSpace.hpp"

#include "EigenPath.hpp"

TaskSpace::TaskSpace(unsigned int dim, const KinematicsSolverPtr& kinematics_solver) 
  : ompl::base::RealVectorStateSpace(dim), kinematics_solver_(kinematics_solver) {
}

TaskSpace::~TaskSpace() {
}

void TaskSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const {
  const auto from_vector = StateToEigenVectorXd(getDimension(), from);
  const auto to_vector = StateToEigenVectorXd(getDimension(), to);
  const auto maybe_to_tcp = kinematics_solver_->solve_fk(to_vector);

  if(!maybe_to_tcp.has_value()) {
    copyState(state, from);
    return;
  }
  const auto to_tcp = maybe_to_tcp.value();

  const auto configs = kinematics_solver_->solve_edge_ik(from_vector, to_tcp);

  if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
    copyState(state, from);
    return;
  }
  EigenPath path(configs);
  EigenVectorXdToState(path.GetConfigAt(t), state);
}
