#include "spaces/TaskSpaceMobileTimeBased.hpp"

#include "EigenPath.hpp"
#include "Common.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/TimeStateSpace.h>

TaskSpaceMobileTimeBased::TaskSpaceMobileTimeBased(const RobotPtr& robot, double vMax, double tMax) 
  : robot_(robot), vMax_(vMax) {

  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());

  auto numDofs = robot->GetSkeleton()->getNumDofs();
  setName("MobileManipulatorTimeBased" + getName());
  auto R2 = std::make_shared<ompl::base::RealVectorStateSpace>(2);
  auto SO2 = std::make_shared<ompl::base::SO2StateSpace>();
  auto N = numDofs-3;
  auto RN = std::make_shared<ompl::base::RealVectorStateSpace>(N);
  auto T = std::make_shared<ompl::base::TimeStateSpace>();
  addSubspace(R2, 1.0);
  addSubspace(SO2, 0.5);
  addSubspace(RN, 1.0);
  addSubspace(T, 1.0);
  
  T->setBounds(0.0, tMax);
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

TaskSpaceMobileTimeBased::~TaskSpaceMobileTimeBased() {
}

double TaskSpaceMobileTimeBased::getVMax() const {
  return vMax_;
}

//void TaskSpaceMobileTimeBased::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const {
//  std::cout << "interpolate: " << t << std::endl;
//  return interpolate(from, to, t, state);
//  //const auto from_vector = robot_->StateToEigen(from);
//  //const auto to_vector = robot_->StateToEigen(to);
//  ////std::cout << "Interpolate " << t << std::endl;
//  //const auto configs = kinematics_solver_->solve_edge_ik_with_config(from_vector, to_vector);

//  //if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
//  //  copyState(state, from);
//  //  return;
//  //}
//  //EigenPath path(configs);
//  //robot_->EigenToState(path.GetConfigAt(t), state);

////   state->as<StateType>()->position =
////     from->as<StateType>()->position + (to->as<StateType>()->position - from->as<StateType>()->position) * t;

//}

double TaskSpaceMobileTimeBased::distance(const ompl::base::State *from, const ompl::base::State *to) const {
  const auto from_vector = robot_->StateToEigen(from);
  const auto to_vector = robot_->StateToEigen(to);

  if(!IsReachableInTime(from_vector, to_vector, vMax_)) {
    return Inf;
  }

  const auto maybe_from_tcp = kinematics_solver_->solve_fk(from_vector);
  if(!maybe_from_tcp.has_value()) {
    return Inf;
  }
  const auto from_tcp = maybe_from_tcp.value();

  const auto maybe_to_tcp = kinematics_solver_->solve_fk(to_vector);
  if(!maybe_to_tcp.has_value()) {
    return Inf;
  }
  const auto to_tcp = maybe_to_tcp.value();

  const auto deltaSpace = Distance(from_tcp, to_tcp);
  return deltaSpace;
}

bool TaskSpaceMobileTimeBased::isMetricSpace() const
{
  return false;
}
