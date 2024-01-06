#include "CollisionChecker.hpp"

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "OmplHelper.hpp"
#include "robots/Robot.hpp"
#include "Common.hpp"

CollisionChecker::CollisionChecker(const dart::simulation::WorldPtr& world, 
    const std::vector<dart::dynamics::SkeletonPtr>& group1, 
    const std::vector<dart::dynamics::SkeletonPtr>& group2) 
  : world_(world), group1_(group1), group2_(group2) {
  auto N = world->getNumSkeletons();
  for(size_t k =0; k<N; k++) {
    const auto& skeleton = world->getSkeleton(k);
    for(const auto& body_node : skeleton->getBodyNodes()) {
      auto visualShapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
      for(auto& visual_node : visualShapeNodes) {
        default_colors_[visual_node] = visual_node->getVisualAspect()->getColor();
      }
    }
  }
}

bool CollisionChecker::IsInCollision() {
  return IsInCollision(world_);
}

bool CollisionChecker::IsInCollision(const dart::simulation::WorldPtr& world) {
  auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();

  for(const auto& rhs : group1_) {
    for(const auto& lhs : group2_) {
      auto rhsGroup = collisionEngine->createCollisionGroup(rhs.get());
      auto lhsGroup = collisionEngine->createCollisionGroup(lhs.get());

      dart::collision::CollisionOption option;
      dart::collision::CollisionResult result;
      bool collision = collisionEngine->collide(rhsGroup.get(), lhsGroup.get(), option, &result);

      if(collision) {
        return true;
      }
    }
  }
  return false;
}

void CollisionChecker::PrintCollisionInfo() {
  auto collisionEngine = world_->getConstraintSolver()->getCollisionDetector();

  std::string collision_info;
  std::string delim;
  for(const auto& rhs : group1_) {
    for(const auto& lhs : group2_) {
      auto rhsGroup = collisionEngine->createCollisionGroup(rhs.get());
      auto lhsGroup = collisionEngine->createCollisionGroup(lhs.get());

      dart::collision::CollisionOption option;
      dart::collision::CollisionResult result;
      bool collision = collisionEngine->collide(rhsGroup.get(), lhsGroup.get(), option, &result);

      if(collision) {
        for(const auto& body : result.getCollidingBodyNodes()) {
          collision_info += delim + body->getName();
          delim = ", ";
        }
      }
    }
  }
  OMPL_DEBUG("Colliding bodies: %s", collision_info.c_str());
}

MultiCollisionChecker::MultiCollisionChecker(const dart::simulation::WorldPtr& world, 
      const std::vector<CollisionCheckerPtr>& collision_checkers) 
  : CollisionChecker(world, {}, {}), collision_checkers_(collision_checkers)
{
}

bool MultiCollisionChecker::IsInCollision(const dart::simulation::WorldPtr& world) {
  for(const auto& collision_checker : collision_checkers_) {
    if(collision_checker->IsInCollision(world)) {
      return true;
    }
  }
  return false;
}

DartWorldCollisionChecker::DartWorldCollisionChecker(const ompl::base::SpaceInformationPtr si, 
    const dart::simulation::WorldPtr& world,
    const dart::dynamics::SkeletonPtr& skeleton,
    const CollisionCheckerPtr& collision_checker)
  : ompl::base::StateValidityChecker(si), world_(world), skeleton_(skeleton), collision_checker_(collision_checker)
{
}

bool DartWorldCollisionChecker::isValid(const ompl::base::State *state) const
{
  auto config = StateToEigenVectorXd(si_, state);
  //Check joint limits
  auto lb = skeleton_->getPositionLowerLimits();
  auto ub = skeleton_->getPositionUpperLimits();
  for(size_t k = 0; k < config.size(); k++) {
    if(config[k] < lb[k] || config[k] > ub[k] || config[k] != config[k]) {
      OMPL_WARN("Out of limits: %f < %f < %f (index %d).", lb[k], ub[k], config[k], k);
      return false;
    }
  }
  //Check collisions
  skeleton_->setConfiguration(config);
  return !collision_checker_->IsInCollision(world_);
}

RobotToObstaclesCollisionChecker::RobotToObstaclesCollisionChecker(
    const dart::simulation::WorldPtr& world,
    const RobotPtr& robot,
    const CollisionCheckerPtr& collision_checker)
  : ompl::base::StateValidityChecker(robot->GetSpaceInformation()), world_(world), robot_(robot), collision_checker_(collision_checker)
{
}

bool RobotToObstaclesCollisionChecker::isValid(const ompl::base::State *state) const
{
  auto config = robot_->StateToEigen(state);
  auto lb = robot_->GetSkeleton()->getPositionLowerLimits();
  auto ub = robot_->GetSkeleton()->getPositionUpperLimits();
  for(size_t k = 0; k < config.size(); k++) {
    if(config[k] < lb[k] || config[k] > ub[k] || config[k] != config[k]) {
      OMPL_WARN("Out of limits: %f < %f < %f (index %d).", lb[k], ub[k], config[k], k);
      return false;
    }
  }
  //Check collisions
  robot_->GetSkeleton()->setConfiguration(config);
  return !collision_checker_->IsInCollision(world_);
}

////////////////////////////////////////////////////////////////////////////////
// DartMultiRobotCollisionChecker
////////////////////////////////////////////////////////////////////////////////

DartMultiRobotCollisionChecker::DartMultiRobotCollisionChecker(const ompl::base::SpaceInformationPtr& si, 
  const dart::simulation::WorldPtr& world,
  const std::unordered_map<std::string, dart::dynamics::SkeletonPtr>& skeletons,
  const CollisionCheckerPtr& collision_checker) 
  : ompl::base::StateValidityChecker(si), world_(world), skeletons_(skeletons), collision_checker_(collision_checker)
{
  auto children = static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si)->getChildren();
  for(const auto& child : children) {
    tmp_skeleton_states_.insert({child->getName(), child->allocState()});
  }
}

DartMultiRobotCollisionChecker::~DartMultiRobotCollisionChecker() {
  auto children = static_cast<ompl::multilevel::FactoredSpaceInformation*>(si_)->getChildren();
  for(const auto& child : children) {
    const auto& name = child->getName();
    auto it = tmp_skeleton_states_.find(name);
    if(it == tmp_skeleton_states_.end()) {
      continue;
    }
    child->freeState(it->second);
  }
}

bool DartMultiRobotCollisionChecker::isValid(const ompl::base::State *state) const {

  auto factor = static_cast<ompl::multilevel::FactoredSpaceInformation*>(si_);
  factor->project(state, tmp_skeleton_states_);

  for(const auto& child : factor->getChildren()) {
    const auto& name = child->getName();
    const auto& state = tmp_skeleton_states_.at(name);
    const auto config = StateToEigenVectorXd(child, state);
    const auto& skeleton = skeletons_.at(name);
    const auto lb = skeleton->getPositionLowerLimits();
    const auto ub = skeleton->getPositionUpperLimits();
    for(size_t k = 0; k < config.size(); k++) {
      if(config[k] < (lb[k] - 1e-5) || config[k] > (ub[k] + 1e-5) || config[k] != config[k]) {
        std::cout <<"Out of bounds: " << config[k] << " not in ["<< lb[k] << "," << ub[k] <<"]" << std::endl;

        return false;
      }
    }
    skeleton->setConfiguration(config);
  }
  return !collision_checker_->IsInCollision(world_);
}
