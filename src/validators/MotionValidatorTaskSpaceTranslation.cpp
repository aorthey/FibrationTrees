#include "validators/MotionValidatorTaskSpaceTranslation.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"

MotionValidatorTaskSpaceTranslation::MotionValidatorTaskSpaceTranslation(
    const ompl::base::SpaceInformationPtr &si, const RobotPtr& robot) 
  : ompl::multilevel::TaskSpaceMotionValidator(si), robot_(robot)
{
  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  tmpState_ = si->allocState();
  lastValidState_ = si->allocState();
}

MotionValidatorTaskSpaceTranslation::~MotionValidatorTaskSpaceTranslation() {
  si_->freeState(tmpState_);
  si_->freeState(lastValidState_);
}

//bool MotionValidatorTaskSpaceTranslation::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
bool MotionValidatorTaskSpaceTranslation::checkMotion(const ompl::base::State *, const ompl::base::State *) const {
  // OMPL_ERROR("NYI. Please use propagateMotion.");
  // throw std::domain_error("You are not allowed to use checkMotion");
  //return TaskSpaceMotionValidator::checkMotion(s1, s2);
  return true;
}

//bool MotionValidatorTaskSpaceTranslation::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
bool MotionValidatorTaskSpaceTranslation::checkMotion(const ompl::base::State *, const ompl::base::State *, std::pair<ompl::base::State *, double> &) const {
  // OMPL_ERROR("NYI. Please use propagateMotion.");
  // throw std::domain_error("You are not allowed to use checkMotion");
  //return TaskSpaceMotionValidator::checkMotion(s1, s2, lastValid);
  return true;
}

std::vector<ompl::base::State*> MotionValidatorTaskSpaceTranslation::propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {

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
