#include "TaskSpaceMobile.hpp"

#include "EigenPath.hpp"
#include "Common.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SO2StateSpace.h>

TaskSpaceMobile::TaskSpaceMobile(const RobotPtr& robot) 
  : robot_(robot) {
  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());

  auto numDofs = robot->GetSkeleton()->getNumDofs();
  setName("MobileManipulator" + getName());
  auto R2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);
  auto SO2 = std::make_shared<ompl::base::SO2StateSpace>();
  auto N = numDofs-3;
  auto RN = std::make_shared<ompl::base::RealVectorStateSpace>(N);
  addSubspace(R2, 1.0);
  addSubspace(SO2, 0.5);
  addSubspace(RN, 1.0);

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds SE2
  ////////////////////////////////////////////////////////////////////////////////
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  ompl::base::RealVectorBounds boundsSE2(2);
  for(size_t k =0; k< 2; k++) {
    boundsSE2.setLow(k, lb[k]);
    boundsSE2.setHigh(k, ub[k]);
  }
  R2->setBounds(boundsSE2);
  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::RealVectorBounds bounds(N);
  for(size_t k =0; k< N; k++) {
    bounds.setLow(k, lb[k+3]);
    bounds.setHigh(k, ub[k+3]);
  }
  RN->setBounds(bounds);
  lock();
}

TaskSpaceMobile::~TaskSpaceMobile() {
}

void TaskSpaceMobile::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const {
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

double TaskSpaceMobile::distance(const ompl::base::State *from, const ompl::base::State *to) const {
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
  return Distance(from_tcp, to_tcp);
}
