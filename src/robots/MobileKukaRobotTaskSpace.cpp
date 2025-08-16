#include "robots/MobileKukaRobotTaskSpace.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "validators/MotionValidatorTaskSpaceTranslation.hpp"
#include "KinematicsSolver.hpp"
#include "samplers/TaskSpaceSampler.hpp"
#include "spaces/TaskSpaceMobile.hpp"
#include "spaces/TaskSpace.hpp"
#include "ToString.hpp"

dart::dynamics::SkeletonPtr MobileKukaRobotTaskSpace::MakeSkeleton(const YAML::Node& node) {
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
    OMPL_INFORM("Setting tcp limits for time based task space: [%s, %s]", 
        ToString(tcp_limits_.value().first).c_str(),
        ToString(tcp_limits_.value().second).c_str());
  }
  return MobileKukaRobot::MakeSkeleton(node);
}

ompl::multilevel::FactoredSpaceInformationPtr MobileKukaRobotTaskSpace::MakeSpaceInformation(const RobotPtr& robot) {
  ompl::base::StateSpacePtr space(new TaskSpaceMobile(robot));
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  if(tcp_limits_.has_value()) {
    OMPL_INFORM("Setting tcp limits for time based task space: [%s, %s]", 
        ToString(tcp_limits_.value().first).c_str(),
        ToString(tcp_limits_.value().second).c_str());
    const auto task_space_limits = std::make_pair(tcp_limits_.value().first, tcp_limits_.value().second);
    factor->getStateSpace()->setStateSamplerAllocator(
          std::bind(&allocateTaskSpaceSampler, robot, task_space_limits));
  }
  return factor;
}

ompl::base::MotionValidatorPtr MobileKukaRobotTaskSpace::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
  return std::make_shared<MotionValidatorTaskSpaceTranslation>(factor, robot);
}

StateXd MobileKukaRobotTaskSpace::StateToEigen(const ompl::base::State* state) const {
  auto N = GetDimension();
  Eigen::VectorXd v(N);
  const auto *state_R2 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1);
  const auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(2);

  v[0] = state_R2->values[0];
  v[1] = state_R2->values[1];
  v[2] = state_SO2->value;

  for (unsigned int k = 0; k < N-3; k++)
  {
      v[k + 3] = state_RN->values[k];
  }
  return MakeState(v);
}

void MobileKukaRobotTaskSpace::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.size();
  auto *state_R2 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1);
  auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(2);

  state_R2->values[0] = v[0];
  state_R2->values[1] = v[1];
  state_SO2->value = v[2];

  for (unsigned int k = 0; k < N-3; k++)
  {
      state_RN->values[k] = v[k + 3];
  }
}

std::vector<State3d> MobileKukaRobotTaskSpace::GetFK(const StateXd& config) const {
  return Robot::GetFK(config);
}
