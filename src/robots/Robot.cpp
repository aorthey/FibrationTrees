#include "robots/Robot.hpp"

size_t Robot::GetDimension() const {
  return factor_->getStateDimension();
}

const std::shared_ptr<dart::dynamics::Skeleton>& Robot::GetSkeleton() {
  return skeleton_;
}

const std::shared_ptr<ompl::multilevel::FactoredSpaceInformation>& Robot::GetSpaceInformation() {
  return factor_;
}

const std::shared_ptr<CollisionChecker>& Robot::GetCollisionChecker() {
  return collision_checker_;
}

void Robot::SetSkeleton(const dart::dynamics::SkeletonPtr& skeleton) {
  skeleton_ = skeleton;
}

void Robot::SetSpaceInformation(const ompl::multilevel::FactoredSpaceInformationPtr& factor) {
  factor_ = factor;
}

void Robot::SetCollisionChecker(const CollisionCheckerPtr& collision_checker) {
  collision_checker_ = collision_checker;
}

bool Robot::IsValid(const ompl::base::State* state) const {
  auto config = this->StateToEigen(state);
  auto lb = skeleton_->getPositionLowerLimits();
  auto ub = skeleton_->getPositionUpperLimits();
  for(size_t k = 0; k < config.size(); k++) {
    if(config[k] < lb[k] || config[k] > ub[k] || config[k] != config[k]) {
      OMPL_WARN("Out of limits: %f is not in [%f, %f] (index %d).", config[k], lb[k], ub[k], k);
      return false;
    }
  }
  //Check collisions
  skeleton_->setConfiguration(config);
  return !collision_checker_->IsInCollision();
}

CollisionCheckerPtr Robot::MakeCollisionChecker(
    const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const dart::simulation::WorldPtr& world,
    const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {

  std::vector<dart::dynamics::SkeletonPtr> collision_group_robot = {GetSkeleton()};
  auto collision_checker = std::make_shared<CollisionChecker>(world, collision_group_robot, obstacles);

  auto func = [&](const ompl::base::State* state) -> bool
  {
    return IsValid(state);
  };
  factor->setStateValidityChecker(func);
  factor->setStateValidityCheckingResolution(0.001);
  return collision_checker;
}

ompl::base::MotionValidatorPtr Robot::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
  return factor->getMotionValidator();
}
