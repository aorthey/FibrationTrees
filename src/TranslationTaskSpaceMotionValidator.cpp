#include "TranslationTaskSpaceMotionValidator.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"

TranslationTaskSpaceMotionValidator::TranslationTaskSpaceMotionValidator(
    const ompl::base::SpaceInformationPtr &si, const RobotPtr& robot) 
  : ompl::multilevel::TaskSpaceMotionValidator(si), robot_(robot)
{
  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  tmpState_ = si->allocState();
  lastValidState_ = si->allocState();
}

TranslationTaskSpaceMotionValidator::~TranslationTaskSpaceMotionValidator() {
  si_->freeState(tmpState_);
  si_->freeState(lastValidState_);
}

bool TranslationTaskSpaceMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  OMPL_ERROR("NYI. Please use propagateMotion.");
  return TaskSpaceMotionValidator::checkMotion(s1, s2);
  // std::pair<ompl::base::State *, double> lastValid;
  // lastValid.first = nullptr;
  // lastValid.second = 0.0f;
  // return checkMotion(s1, s2, lastValid);
}

// bool TranslationTaskSpaceMotionValidator::FillLastStateOnNoProgressAndReturn(
//     const ompl::base::State *state, std::pair<ompl::base::State *, double> &lastValid) const {
//   if(lastValid.first != nullptr) {
//     si_->copyState(lastValid.first, state);
//     lastValid.second = 0.0f;
//   }
//   return false;
// }

bool TranslationTaskSpaceMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  OMPL_ERROR("NYI. Please use propagateMotion.");
  return TaskSpaceMotionValidator::checkMotion(s1, s2, lastValid);
  //const auto from_vector = robot_->StateToEigen(s1);
  //const auto to_vector = robot_->StateToEigen(s2);

  //////////////////////////////////////////////////////////////////////////////////
  ////Solve edge ik
  //////////////////////////////////////////////////////////////////////////////////
  //const auto configs = kinematics_solver_->solve_edge_ik_with_config(from_vector, to_vector);
  //if(configs.empty()) {
  //  return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  //}
  //if(!kinematics_solver_->lastSolveWasSuccessful()) {
  //  robot_->EigenToState(configs.back(), lastValidState_);
  //  return FillLastStateOnNoProgressAndReturn(lastValidState_, lastValid);
  //}
  //std::cout << "Interpolated " << configs.size() << " states." << std::endl;
  //std::cout << "First config: " << configs.front() << std::endl;
  //std::cout << "Last  config: " << configs.back() << std::endl;

  //si_->copyState(lastValidState_, s1);
  //EigenPath path(configs);
  //float L = path.GetLength();
  //std::cout << "Path length: " << L << std::endl;
  //for(float d = 0; d < L; d+= kDeltaCollisionCheckStepSize) {
  //  robot_->EigenToState(path.GetConfigAt(d/L), tmpState_);
  //  if(!si_->isValid(tmpState_)) {
  //    std::cout << "Invalid at " << d/L << std::endl;
  //    if(lastValid.first != nullptr) {
  //      si_->copyState(lastValid.first, lastValidState_);
  //      lastValid.second = si_->distance(s1, lastValid.first);
  //    }
  //    return false;
  //  }
  //  si_->copyState(lastValidState_, tmpState_);
  //}

  //if((configs.back() - to_vector).norm() < 1e-3) {
  //  return true;
  //}

  //if(lastValid.first != nullptr) {
  //  si_->copyState(lastValid.first, lastValidState_);
  //  lastValid.second = si_->distance(s1, lastValid.first);
  //}
  //return false;
}

std::vector<ompl::base::State*> TranslationTaskSpaceMotionValidator::propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {

  std::vector<ompl::base::State*> result;
  ////////////////////////////////////////////////////////////////////////////////
  //Solve edge ik
  ////////////////////////////////////////////////////////////////////////////////
  const auto from_vector = robot_->StateToEigen(s1);
  const auto to_vector = robot_->StateToEigen(s2);
  const auto configs = kinematics_solver_->solve_edge_ik_with_config(from_vector, to_vector);
  if(configs.empty()) {
    OMPL_WARN("Solver found no configs.");
    return result;
  }

  ////////////////////////////////////////////////////////////////////////////////
  //Check validities
  ////////////////////////////////////////////////////////////////////////////////
  auto last_config = from_vector;
  for(const auto& config : configs) {
    robot_->EigenToState(config, tmpState_);
    if(!si_->isValid(tmpState_)) {
      return result;
    }
    //Do not add too many states, only every X spaced
    if( Distance(config, last_config) < kMinimumSpacing) {
      continue;
    }
    auto state = si_->allocState();
    si_->copyState(state, tmpState_);
    result.push_back(state);

    last_config = config;
  }
  return result;
}
