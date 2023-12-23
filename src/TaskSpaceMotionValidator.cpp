#include "TaskSpaceMotionValidator.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"

const float kDeltaCollisionCheckStepSize = 0.005;

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

  ////////////////////////////////////////////////////////////////////////////////
  //Solve edge ik
  ////////////////////////////////////////////////////////////////////////////////
  //Use interpolate here
  const auto configs = kinematics_solver_->solve_edge_ik_with_config(from_vector, to_vector);
  if(configs.empty()) {
    return FillLastStateOnNoProgressAndReturn(s1, lastValid);
  }
  if(!kinematics_solver_->lastSolveWasSuccessful()) {
    EigenVectorXdToState(configs.back(), lastValidState_);
    return FillLastStateOnNoProgressAndReturn(lastValidState_, lastValid);
  }
  // std::cout << "Interpolated " << configs.size() << " states." << std::endl;
  // std::cout << "First config: " << configs.front().format(CommaFmt) << std::endl;
  // std::cout << "Last  config: " << configs.back().format(CommaFmt) << std::endl;

  si_->copyState(lastValidState_, s1);
  EigenPath path(configs);
  float L = path.GetLength();
  // std::cout << "Path length: " << L << std::endl;
  for(float d = 0; d < L; d+= kDeltaCollisionCheckStepSize) {
    EigenVectorXdToState(path.GetConfigAt(d/L), tmpState_);
    if(!si_->isValid(tmpState_)) {
      // std::cout << "Invalid at " << d/L << std::endl;
      if(lastValid.first != nullptr) {
        si_->copyState(lastValid.first, lastValidState_);
        lastValid.second = si_->distance(s1, lastValid.first);
      }
      return false;
    }
    si_->copyState(lastValidState_, tmpState_);
  }

  if((configs.back() - to_vector).norm() < 1e-3) {
    std::cout << "#####################################################" << std::endl;
    std::cout << "SUCCESS SOLVE EDGE IK WITH CONFIG" << std::endl;
    std::cout << "#####################################################" << std::endl;
    return true;
  }

  if(lastValid.first != nullptr) {
    si_->copyState(lastValid.first, lastValidState_);
    lastValid.second = si_->distance(s1, lastValid.first);
  }
  return false;
}
