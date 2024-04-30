#include "validators/MotionValidatorTimeBased.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"
#include "spaces/TaskSpaceMobileTimeBased.hpp"

MotionValidatorTimeBased::MotionValidatorTimeBased(
    const ompl::base::SpaceInformationPtr &si, const RobotPtr& robot, double vMax)
  : MotionValidatorTaskSpaceTranslation(si, robot), vMax_(vMax)
{
}

MotionValidatorTimeBased::~MotionValidatorTimeBased() {
}

bool MotionValidatorTimeBased::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  const auto s1_vector = robot_->StateToEigen(s1);
  const auto s2_vector = robot_->StateToEigen(s2);

  if(!IsReachableInTime(s1_vector, s2_vector, vMax_)) {
    return false;
  }
  return TaskSpaceMotionValidator::checkMotion(s1, s2);
}

bool MotionValidatorTimeBased::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  const auto s1_vector = robot_->StateToEigen(s1);
  const auto s2_vector = robot_->StateToEigen(s2);

  if(!IsReachableInTime(s1_vector, s2_vector, vMax_)) {
    lastValid.second = 0.0;
    return false;
  }
  return TaskSpaceMotionValidator::checkMotion(s1, s2);
}

float ComputeTime(const StateXd& start, const StateXd& current, float vMax) {
  auto dx = Distance(start, current);
  return start.time + dx / vMax;
}

std::vector<ompl::base::State*> MotionValidatorTimeBased::propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {

  std::vector<ompl::base::State*> result;
  ////////////////////////////////////////////////////////////////////////////////
  //Solve edge ik
  ////////////////////////////////////////////////////////////////////////////////
  const auto from_vector = robot_->StateToEigen(s1);
  const auto to_vector = robot_->StateToEigen(s2);

  if(!IsReachableInTime(from_vector, to_vector, vMax_)) {
    return result;
  }
  //auto deltaTime = to_vector.time - from_vector.time;
  //if(deltaTime <= 0) {
  //  std::cout << "[Rejected]: Negative time:" << deltaTime << std::endl;
  //  //Cannot propagate backwards in time
  //  return result;
  //}

  //auto deltaSpace = Distance(from_vector, to_vector);
  //if (deltaSpace / vMax_ > deltaTime + Epsilon) {
  //  //Cannot physically reach goal
  //  std::cout << "[Rejected]: Not enough time:" << deltaTime << " > " << deltaSpace / vMax_ << std::endl;
  //  return result;
  //}

  auto configs = kinematics_solver_->solve_edge_ik_with_config(from_vector, to_vector);
  if(configs.empty()) {
    OMPL_WARN("Solver found no configs.");
    return result;
  }
  for(auto& config : configs) {
    config.time = ComputeTime(from_vector, config, vMax_);
  }
  ////////////////////////////////////////////////////////////////////////////////
  //Check validities
  ////////////////////////////////////////////////////////////////////////////////
  auto last_config = from_vector;
  //std::cout << "Edge IK found " << configs.size() << " states." << std::endl;
  for(const auto& config : configs) {
    robot_->EigenToState(config, tmpState_);
    if(!si_->isValid(tmpState_)) {
      //std::cout << "Invalid state" << std::endl;
      //si_->printState(tmpState_);
      return result;
    }
    //Do not add too many states, only every X spaced
    if(Distance(config, last_config) < kMinimumSpacing) {
      continue;
    }
    auto state = si_->allocState();
    si_->copyState(state, tmpState_);
    result.push_back(state);

    last_config = config;
  }
  return result;
}
