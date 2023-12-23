#include "TaskSpace.hpp"

#include "EigenPath.hpp"
#include "Common.hpp"

TaskSpace::TaskSpace(unsigned int dim, const KinematicsSolverPtr& kinematics_solver) 
  : ompl::base::RealVectorStateSpace(dim), kinematics_solver_(kinematics_solver) {
}

TaskSpace::~TaskSpace() {
}

void TaskSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const {
  const auto from_vector = StateToEigenVectorXd(getDimension(), from);
  const auto to_vector = StateToEigenVectorXd(getDimension(), to);
  const auto configs = kinematics_solver_->solve_edge_ik_with_config(from_vector, to_vector);

  if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
    copyState(state, from);
    return;
  }
  EigenPath path(configs);
  EigenVectorXdToState(path.GetConfigAt(t), state);
}

double TaskSpace::distance(const ompl::base::State *from, const ompl::base::State *to) const {
  const auto from_vector = StateToEigenVectorXd(getDimension(), from);
  const auto maybe_from_tcp = kinematics_solver_->solve_fk(from_vector);
  if(!maybe_from_tcp.has_value()) {
    return Inf;
  }
  const auto from_tcp = maybe_from_tcp.value();

  const auto to_vector = StateToEigenVectorXd(getDimension(), to);
  const auto maybe_to_tcp = kinematics_solver_->solve_fk(to_vector);
  if(!maybe_to_tcp.has_value()) {
    return Inf;
  }
  const auto to_tcp = maybe_to_tcp.value();
  return (from_tcp - to_tcp).norm();
}
