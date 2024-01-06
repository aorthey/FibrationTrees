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
  // std::cout << "IsValid" << std::endl;
  // factor_->printSettings(std::cout);
  // factor_->printState(state);
  auto config = this->StateToEigen(state);
  auto lb = skeleton_->getPositionLowerLimits();
  auto ub = skeleton_->getPositionUpperLimits();
  for(size_t k = 0; k < config.size(); k++) {
    if(config[k] < lb[k] || config[k] > ub[k] || config[k] != config[k]) {
      OMPL_WARN("Out of limits: %f < %f < %f (index %d).", lb[k], ub[k], config[k], k);
      return false;
    }
  }
  if(!collision_checker_) {
    return true;
  }
  //Check collisions
  skeleton_->setConfiguration(config);
  return !collision_checker_->IsInCollision();
}
