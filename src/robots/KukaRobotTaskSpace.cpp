#include "robots/KukaRobotTaskSpace.hpp"
#include "validators/MotionValidatorTaskSpaceTranslation.hpp"

#include "spaces/TaskSpace.hpp"
#include "samplers/TaskSpaceSampler.hpp"
#include "KinematicsSolver.hpp"
#include "ToString.hpp"

dart::dynamics::SkeletonPtr KukaRobotTaskSpace::MakeSkeleton(const YAML::Node& node) {
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
  return KukaRobot::MakeSkeleton(node);
}

ompl::multilevel::FactoredSpaceInformationPtr KukaRobotTaskSpace::MakeSpaceInformation(const RobotPtr& robot) {
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  ompl::base::StateSpacePtr space(new TaskSpace(robot));
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

ompl::base::MotionValidatorPtr KukaRobotTaskSpace::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
  return std::make_shared<MotionValidatorTaskSpaceTranslation>(factor, robot);
}
