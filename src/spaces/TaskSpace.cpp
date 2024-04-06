#include "spaces/TaskSpace.hpp"

#include "EigenPath.hpp"
#include "Common.hpp"

TaskSpace::TaskSpace(const RobotPtr& robot) 
  : ompl::base::RealVectorStateSpace(robot->GetSkeleton()->getNumDofs()), robot_(robot) {
  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());

  auto numDofs = robot->GetSkeleton()->getNumDofs();
  ompl::base::RealVectorBounds bounds(numDofs);
  auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k =0; k< numDofs; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  setBounds(bounds);
}

TaskSpace::~TaskSpace() {
}

void TaskSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const {
  const auto from_vector = robot_->StateToEigen(from);
  const auto to_vector = robot_->StateToEigen(to);
  //std::cout << "Interpolate " << t << std::endl;
  const auto configs = kinematics_solver_->solve_edge_ik_with_config(from_vector, to_vector);

  if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
    copyState(state, from);
    return;
  }
  EigenPath path(configs);
  robot_->EigenToState(path.GetConfigAt(t), state);
}

double TaskSpace::distance(const ompl::base::State *from, const ompl::base::State *to) const {
  const auto from_vector = robot_->StateToEigen(from);
  const auto maybe_from_tcp = kinematics_solver_->solve_fk(from_vector);
  if(!maybe_from_tcp.has_value()) {
    return Inf;
  }
  const auto from_tcp = maybe_from_tcp.value();

  const auto to_vector = robot_->StateToEigen(to);
  const auto maybe_to_tcp = kinematics_solver_->solve_fk(to_vector);
  if(!maybe_to_tcp.has_value()) {
    return Inf;
  }
  const auto to_tcp = maybe_to_tcp.value();
  return (from_tcp - to_tcp).norm();
}
