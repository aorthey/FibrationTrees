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
  // const auto maybe_v1 = kinematics_solver_->solve_fk(from_vector);
  const auto maybe_v2 = kinematics_solver_->solve_fk(to_vector);

  if(!maybe_v2.has_value()) {
    copyState(state, from);
    return;
  }
  // const auto v1 = maybe_v1.value();
  const auto v2 = maybe_v2.value();

  // const auto vt = v1 + t*(v2 - v1);

  // std::cout << "SOVLE FROM " << from_vector << " TO " << to_vector << std::endl;
  // std::cout << "SOVLE FROM " << v1 << " TO " << v2 << std::endl;
  const auto configs = kinematics_solver_->solve_edge_ik(from_vector, v2);

  if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
    // std::cout << "Empty towards " << v2 << std::endl;
    copyState(state, from);
    return;
  }
  EigenPath path(configs);
  // std::cout << "Found path at " << t << " :" << path.GetConfigAt(t) << " config size is " << configs.size() << std::endl;
  EigenVectorXdToState(path.GetConfigAt(t), state);
}

TaskSpaceMotionValidator::TaskSpaceMotionValidator(
    ompl::base::SpaceInformation *si, const KinematicsSolverPtr& kinematics_solver) 
  : ompl::base::MotionValidator(si), kinematics_solver_(kinematics_solver)
{
  tmpState_ = si->allocState();
  lastValidState_ = si->allocState();
}
TaskSpaceMotionValidator::TaskSpaceMotionValidator(
    const ompl::base::SpaceInformationPtr &si, const KinematicsSolverPtr& kinematics_solver) 
  : ompl::base::MotionValidator(si), kinematics_solver_(kinematics_solver)
{
  tmpState_ = si->allocState();
  lastValidState_ = si->allocState();
}
TaskSpaceMotionValidator::~TaskSpaceMotionValidator() {
  si_->freeState(tmpState_);
  si_->freeState(lastValidState_);
}

bool TaskSpaceMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  std::pair<ompl::base::State *, double> lastValid;
  lastValid.first = nullptr;
  lastValid.second = 0.0f;
  return checkMotion(s1, s2, lastValid);
  // const auto from_vector = StateToEigenVectorXd(si_->getStateDimension(), s1);
  // const auto to_vector = StateToEigenVectorXd(si_->getStateDimension(), s2);

  // const auto maybe_goal_frame = kinematics_solver_->solve_fk(to_vector);

  // if(!maybe_goal_frame.has_value()) {
  //   return false;
  // }

  // const auto goal_frame = maybe_goal_frame.value();
  // const auto configs = kinematics_solver_->solve_edge_ik(from_vector, goal_frame);

  // if(!kinematics_solver_->lastSolveWasSuccessful() || configs.empty()) {
  //   return false;
  // }

  // EigenPath path(configs);
  // for(float d = 0; d < 1.0; d+= 0.01) {
  //   EigenVectorXdToState(path.GetConfigAt(d), tmpState_);
  //   if(!si_->isValid(tmpState_)) {
  //     return false;
  //   }
  // }
  // // for(const auto& config : configs) {
  // //   EigenVectorXdToState(config, tmpState_);
  // //   if(!si_->isValid(tmpState_)) {
  // //     return false;
  // //   }
  // // }
  // return true;
}

bool TaskSpaceMotionValidator::FillLastStateOnNoProgressAndReturn(
    const ompl::base::State *state, std::pair<ompl::base::State *, double> &lastValid) const {
  if(lastValid.first != nullptr) {
    si_->copyState(lastValid.first, state);
    lastValid.second = 0.0f;
  }
  return false;
}

bool TaskSpaceMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  const auto from_vector = StateToEigenVectorXd(si_->getStateDimension(), s1);
  const auto to_vector = StateToEigenVectorXd(si_->getStateDimension(), s2);

  if((to_vector - from_vector).norm() < 1e-6) {
    return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  }
  ////////////////////////////////////////////////////////////////////////////////
  //Check goal frame
  ////////////////////////////////////////////////////////////////////////////////
  const auto maybe_goal_frame = kinematics_solver_->solve_fk(to_vector);
  if(!maybe_goal_frame.has_value()) {
    return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  }
  const auto goal_frame = maybe_goal_frame.value();

  if(!maybe_goal_frame.has_value()) {
    return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  }

  ////////////////////////////////////////////////////////////////////////////////
  //Check start frame
  ////////////////////////////////////////////////////////////////////////////////
  const auto maybe_tcp_start = kinematics_solver_->solve_fk(from_vector);
  if(!maybe_tcp_start.has_value()) {
    return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  }
  const auto start_frame = maybe_tcp_start.value();
  if((start_frame - goal_frame).norm() < 1e-3) {
    return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  }

  ////////////////////////////////////////////////////////////////////////////////
  //Solve edge ik
  ////////////////////////////////////////////////////////////////////////////////
  const auto configs = kinematics_solver_->solve_edge_ik(from_vector, goal_frame);
  if(configs.empty()) {
    return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  }
  if(!kinematics_solver_->lastSolveWasSuccessful()) {
    EigenVectorXdToState(configs.back(), lastValidState_);
    return FillLastStateOnNoProgressAndReturn(lastValidState_, lastValid);
  }

  si_->copyState(lastValidState_, s1);
  EigenPath path(configs);
  float L = path.GetLength();
  for(float d = 0; d < L; d+= 0.01) {
    EigenVectorXdToState(path.GetConfigAt(d/L), tmpState_);
    if(!si_->isValid(tmpState_)) {
      if(lastValid.first != nullptr) {
        si_->copyState(lastValid.first, lastValidState_);
        lastValid.second = d/L;
      }
      return false;
    }
    si_->copyState(lastValidState_, tmpState_);
  }

  if((configs.back() - to_vector).norm() < 1e-3) {
    return true;
  }
  // return true;
  //TODO: Return true again for oob cases
  if(lastValid.first != nullptr) {
    si_->copyState(lastValid.first, lastValidState_);
    lastValid.second = L;
  }
  return false;
}

