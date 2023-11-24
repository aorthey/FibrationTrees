#include "TaskStateSpace.hpp"

TaskStateSpace::TaskStateSpace(unsigned int dim, const KinematicsSolverPtr& kinematics_solver) 
  : ompl::base::RealVectorStateSpace(dim), kinematics_solver_(kinematics_solver) {
}

TaskStateSpace::~TaskStateSpace() {
}

void TaskStateSpace::interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const {
  const auto from_vector = StateToEigenVectorXd(getDimension(), from);
  const auto to_vector = StateToEigenVectorXd(getDimension(), to);
  const auto maybe_v1 = kinematics_solver_->solve_fk(from_vector);
  const auto maybe_v2 = kinematics_solver_->solve_fk(to_vector);

  if(!maybe_v1.has_value() || !maybe_v2.has_value()) {
    copyState(state, from);
    return;
  }
  const auto v1 = maybe_v1.value();
  const auto v2 = maybe_v2.value();

  const auto vt = v1 + t*(v2 - v1);

  const auto configs = kinematics_solver_->solve_edge_ik(from_vector, vt);
  if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
    copyState(state, from);
    return;
  }

  EigenVectorXdToState(configs.back(), state);
}

bool TaskSpaceMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  const auto from_vector = StateToEigenVectorXd(si_->getStateDimension(), s1);
  const auto to_vector = StateToEigenVectorXd(si_->getStateDimension(), s2);

  const auto maybe_goal_frame = kinematics_solver_->solve_fk(to_vector);

  if(!maybe_goal_frame.has_value()) {
    return false;
  }

  const auto goal_frame = maybe_goal_frame.value();
  const auto configs = kinematics_solver_->solve_edge_ik(from_vector, goal_frame);

  if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
    return false;
  }

  for(const auto& config : configs) {
    EigenVectorXdToState(config, tmpState_);
    if(!si_->isValid(tmpState_)) {
      return false;
    }
  }
  return true;
}

bool TaskSpaceMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  const auto from_vector = StateToEigenVectorXd(si_->getStateDimension(), s1);
  const auto to_vector = StateToEigenVectorXd(si_->getStateDimension(), s2);

  const auto maybe_goal_frame = kinematics_solver_->solve_fk(to_vector);

  si_->copyState(lastValidState_, s1);
  if(!maybe_goal_frame.has_value()) {
    if(lastValid.first != nullptr) {
      si_->copyState(lastValid.first, lastValidState_);
      lastValid.second = 0.0f;
    }
    return false;
  }

  const auto goal_frame = maybe_goal_frame.value();
  const auto configs = kinematics_solver_->solve_edge_ik(from_vector, goal_frame);
  if(!kinematics_solver_->lastSolveWasSuccessful()) {
    return false;
  }

  const float d_all = si_->distance(s1, s2);
  for(const auto& config : configs) {
    EigenVectorXdToState(config, tmpState_);
    if(!si_->isValid(tmpState_)) {
      if(lastValid.first != nullptr) {
        si_->copyState(lastValid.first, lastValidState_);
        lastValid.second = si_->distance(s1, lastValidState_) / d_all;
      }
      si_->freeState(lastValidState_);
      return false;
    }
    si_->copyState(lastValidState_, tmpState_);
  }
  return true;
}
