#include "robots/KukaRobotTaskSpace.hpp"
#include "validators/MotionValidatorTaskSpaceTranslation.hpp"

#include "spaces/TaskSpace.hpp"
#include "KinematicsSolver.hpp"

ompl::multilevel::FactoredSpaceInformationPtr KukaRobotTaskSpace::MakeSpaceInformation(const RobotPtr& robot) {
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  ompl::base::StateSpacePtr space(new TaskSpace(robot));
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}

ompl::base::MotionValidatorPtr KukaRobotTaskSpace::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
  return std::make_shared<MotionValidatorTaskSpaceTranslation>(factor, robot);
}
