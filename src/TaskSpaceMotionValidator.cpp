#include "TaskSpaceMotionValidator.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"

TaskSpaceMotionValidator::TaskSpaceMotionValidator(
    ompl::base::SpaceInformation *si, const KinematicsSolverPtr& kinematics_solver) 
  : ompl::base::DiscreteMotionValidator(si), kinematics_solver_(kinematics_solver)
{
  tmpState_ = si->allocState();
  lastValidState_ = si->allocState();
}
TaskSpaceMotionValidator::TaskSpaceMotionValidator(
    const ompl::base::SpaceInformationPtr &si, const KinematicsSolverPtr& kinematics_solver) 
  : ompl::base::DiscreteMotionValidator(si), kinematics_solver_(kinematics_solver)
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
  //Use interpolate here
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

  if(lastValid.first != nullptr) {
    si_->copyState(lastValid.first, lastValidState_);
    lastValid.second = L;
  }
  return false;
}


TaskSpaceMultiRobotMotionValidator::TaskSpaceMultiRobotMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor)
  : ompl::base::DiscreteMotionValidator(factor)
{
  for(const auto& child : factor->getChildren()) {
    motion_validators_.push_back(child->getMotionValidator());
  }
  auto space = factor->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    // lastValids_.push_back({std::make_pair(space->getSubspace(k)->allocState(), 0.0f)});
    lastValids_.push_back(space->getSubspace(k)->allocState());
  }
  if(space->getSubspaceCount() != motion_validators_.size()) {
    OMPL_ERROR("Number of subspaces and number of motion validators does not match up (%d != %d).", 
        space->getSubspaceCount(), motion_validators_.size());
    throw "InvalidNumber";
  }
  tmpStateOnTotalSpace_ = factor->allocState();
}

TaskSpaceMultiRobotMotionValidator::~TaskSpaceMultiRobotMotionValidator() {
  auto space = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    space->getSubspace(k)->freeState(lastValids_.at(k));
  }
  si_->freeState(tmpStateOnTotalSpace_);
}

bool TaskSpaceMultiRobotMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  auto lastValid = std::make_pair(tmpStateOnTotalSpace_, 0.0);
  return TaskSpaceMultiRobotMotionValidator::checkMotion(s1, s2, lastValid);
}

bool TaskSpaceMultiRobotMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  if(lastValid.first == nullptr) {
    OMPL_ERROR("Last valid state is required.");
    throw "NYI";
  }

  auto s1_compound = s1->as<ompl::base::CompoundState>();
  auto s2_compound = s2->as<ompl::base::CompoundState>();

  auto space = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  bool all_subspaces_are_valid = true;

  ////////////////////////////////////////////////////////////////////////////////
  // Propagate states forward on each subspace until collision or failure
  ////////////////////////////////////////////////////////////////////////////////
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    auto spacek = space->getSubspace(k);
    auto s1_k = s1_compound->operator[](k);
    auto s2_k = s2_compound->operator[](k);

    std::pair<ompl::base::State *, double> lastValidSpaceK;
    lastValidSpaceK.first = lastValids_.at(k);
    lastValidSpaceK.second = 0.0;

    //Check motion for component subspace and return in lastValidSpaceK
    if(!motion_validators_.at(k)->checkMotion(s1_k, s2_k, lastValidSpaceK)) {
      all_subspaces_are_valid = false;
    }
    spacek->copyState(lastValid.first->as<ompl::base::CompoundState>()->operator[](k), lastValidSpaceK.first);
    lastValid.second += lastValidSpaceK.second;
    OMPL_INFORM("Progress %f on %d", lastValidSpaceK.second, k);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Check that resulting states are valid for this factor
  ////////////////////////////////////////////////////////////////////////////////
  if(all_subspaces_are_valid || lastValid.second > 0.0) {
    const auto last_state = (all_subspaces_are_valid ? s2 : lastValid.first);
    //TODO: Needs to forward propagate, not direct interpolation
    if(!ompl::base::DiscreteMotionValidator::checkMotion(s1, last_state, lastValid)) {
      all_subspaces_are_valid = false;
    } else {
      std::cout << "Verified motion." << std::endl;
    }
  }

  OMPL_INFORM("Total %f", lastValid.second);
  return all_subspaces_are_valid;
}

