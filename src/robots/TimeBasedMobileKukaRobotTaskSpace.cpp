#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include "validators/MotionValidatorTaskSpaceTranslation.hpp"
#include "validators/MotionValidatorTimeBased.hpp"
#include "KinematicsSolver.hpp"
#include "spaces/TaskSpaceMobileTimeBased.hpp"
#include "spaces/TaskSpace.hpp"
#include "samplers/TaskSpaceSampler.hpp"
#include "Common.hpp"

TimeBasedMobileKukaRobotTaskSpace::TimeBasedMobileKukaRobotTaskSpace(float vMax, float tMax) :
  vMax_(vMax), tMax_(tMax) {}

dart::dynamics::SkeletonPtr TimeBasedMobileKukaRobotTaskSpace::MakeSkeleton(const YAML::Node& node) {
  if(node["tcp_lower_limits"]) {
    const auto lower_limits = node["tcp_lower_limits"].as<std::vector<double>>();
    if(lower_limits.size() != 3) {
      throw std::domain_error("Tcp limits size must be 3 (x, y, z)");
    }
    if(!node["tcp_upper_limits"]) {
      throw std::domain_error("Tcp limits requires both lower and upper limits (missing upper limits)");
    }
    const auto upper_limits = node["tcp_upper_limits"].as<std::vector<double>>();
    if(upper_limits.size() != 3) {
      throw std::domain_error("Tcp limits size must be 3 (x, y, z)");
    }
    tcp_limits_ = std::make_pair(MakeState3d(lower_limits), MakeState3d(upper_limits));
  }
  return MobileKukaRobotTaskSpace::MakeSkeleton(node);
}

ompl::multilevel::FactoredSpaceInformationPtr TimeBasedMobileKukaRobotTaskSpace::MakeSpaceInformation(const RobotPtr& robot) {
  ompl::base::StateSpacePtr space_time(new TaskSpaceMobileTimeBased(robot, vMax_, tMax_));

  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space_time);
  if(tcp_limits_.has_value()) {
    const auto task_space_limits = std::make_pair(tcp_limits_.value().first, tcp_limits_.value().second);
    factor->getStateSpace()->setStateSamplerAllocator(
          std::bind(&allocateTaskSpaceSampler, robot, task_space_limits));
  }
  return factor;
}

float TimeBasedMobileKukaRobotTaskSpace::GetVMax() const {
  return vMax_;
}

float TimeBasedMobileKukaRobotTaskSpace::GetTMax() const {
  return tMax_;
}

void TimeBasedMobileKukaRobotTaskSpace::SetVMax(float vMax) {
  vMax_ = vMax;
}

void TimeBasedMobileKukaRobotTaskSpace::SetTMax(float tMax) {
  tMax_ = tMax;
}

void TimeBasedMobileKukaRobotTaskSpace::SetSpaceInformationFromRobot(const RobotPtr& robot, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  ompl::base::StateSpacePtr space_time(new ompl::base::SpaceTimeStateSpace(robot->GetSpaceInformation()->getStateSpace(), vMax_));
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space_time);
  auto motion_validator = MakeMotionValidator(factor, robot);
  factor->setMotionValidator(motion_validator);
  SetSpaceInformation(factor);
  auto collision_checker = MakeCollisionChecker(factor, world, obstacles);
  SetCollisionChecker(collision_checker);
}

ompl::base::MotionValidatorPtr TimeBasedMobileKukaRobotTaskSpace::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
  return std::make_shared<MotionValidatorTimeBased>(factor, robot, vMax_);
}

float TimeBasedMobileKukaRobotTaskSpace::StateToTime(const ompl::base::State* state) const {
  return state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(3)->position;
}
void TimeBasedMobileKukaRobotTaskSpace::TimeToState(const float time, ompl::base::State* state) const {
  state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(3)->position = time;

  if(time < 0.0) {
    state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(3)->position = 0.0;
  }
  if(time > GetTMax()) {
    OMPL_WARN("Time is out of bounds");
    state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(3)->position = GetTMax();
  }
}

StateXd TimeBasedMobileKukaRobotTaskSpace::StateToEigen(const ompl::base::State* state) const {
  auto N = GetDimension();
  Eigen::VectorXd v(N-1);

  const auto *cstate = state->as<ompl::base::CompoundState>();
  const auto *state_R2 = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const auto *state_SO2 = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
  const auto *state_RN = cstate->as<ompl::base::RealVectorStateSpace::StateType>(2);
  const auto *state_T = cstate->as<ompl::base::TimeStateSpace::StateType>(3);

  v[0] = state_R2->values[0];
  v[1] = state_R2->values[1];
  v[2] = state_SO2->value;

  for (unsigned int k = 0; k < N-4; k++)
  {
      v[k + 3] = state_RN->values[k];
  }

  auto eigen_state = MakeState(v);
  eigen_state.time = state_T->position;
  return eigen_state;
}

void TimeBasedMobileKukaRobotTaskSpace::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.configuration.size();

  auto *cstate = state->as<ompl::base::CompoundState>();
  auto *state_R2 = cstate->as<ompl::base::RealVectorStateSpace::StateType>(0);
  auto *state_SO2 = cstate->as<ompl::base::SO2StateSpace::StateType>(1);
  auto *state_RN = cstate->as<ompl::base::RealVectorStateSpace::StateType>(2);
  auto *state_T = cstate->as<ompl::base::TimeStateSpace::StateType>(3);

  state_R2->values[0] = v.configuration[0];
  state_R2->values[1] = v.configuration[1];
  state_SO2->value = v.configuration[2];

  for (unsigned int k = 0; k < N-3; k++)
  {
      state_RN->values[k] = v.configuration[k + 3];
  }

  state_T->position = v.time;
}
