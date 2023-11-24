#include "CollisionChecker.hpp"

#include "OmplHelper.hpp"

CollisionChecker::CollisionChecker(const dart::simulation::WorldPtr& world, 
    const std::vector<dart::dynamics::SkeletonPtr>& group1, 
    const std::vector<dart::dynamics::SkeletonPtr>& group2) 
  : group1_(group1), group2_(group2) {
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

void CollisionChecker::ColorAllCollisionBodies(dart::simulation::WorldPtr world) {
  auto collisionEngine
      = world->getConstraintSolver()->getCollisionDetector();

  for(const auto& rhs : group1_) {
    for(const auto& lhs : group2_) {
      auto rhsGroup = collisionEngine->createCollisionGroup(rhs.get());
      auto lhsGroup = collisionEngine->createCollisionGroup(lhs.get());

      dart::collision::CollisionOption option;
      dart::collision::CollisionResult result;
      bool collision = collisionEngine->collide(rhsGroup.get(), lhsGroup.get(), option, &result);

      auto N = world->getNumSkeletons();

      for(size_t k =0; k<N; k++) {
        const auto& skeleton = world->getSkeleton(k);
        for(const auto& body_node : skeleton->getBodyNodes()) {
          if(!result.inCollision(body_node)) {
            continue;
          }
          auto shapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
          for(const auto& node : shapeNodes) {
            std::cout << "Set color : " << skeleton->getName() << ":" << body_node->getName() << ":" << node->getName() << std::endl;
            node->getVisualAspect()->setColor(kCollisionColor);
            node->getVisualAspect()->show();
          }
        }
      }
    }
  }
}

void CollisionChecker::ResetColors(const dart::simulation::WorldPtr& world) {
  auto N = world->getNumSkeletons();

  for(size_t k =0; k<N; k++) {
    const auto& skeleton = world->getSkeleton(k);
    for(const auto& body_node : skeleton->getBodyNodes()) {
      auto visualShapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
      for(auto& visual_node : visualShapeNodes) {
        auto node = default_colors_.find(visual_node);
        if(node == default_colors_.end()) {
          continue;
        }
        visual_node->getVisualAspect()->setColor(node->second);
      }
    }
  }
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
      return false;
    }
  }
  //Check collisions
  skeleton_->setConfiguration(config);
  return !collision_checker_->IsInCollision(world_);
}

DartTransformCollisionChecker::DartTransformCollisionChecker(const ompl::base::SpaceInformationPtr si, 
    const dart::simulation::WorldPtr& world,
    const dart::dynamics::SkeletonPtr& skeleton,
    const CollisionCheckerPtr& collision_checker)
  : DartWorldCollisionChecker(si, world, skeleton, collision_checker)
{
}

bool DartTransformCollisionChecker::isValid(const ompl::base::State *state) const
{
  auto config = StateToEigenVector3d(state);
  Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
  tf.translation() = config;

  for(const auto& body_node : skeleton_->getBodyNodes()) {
    body_node->getParentJoint()->setTransformFromParentBodyNode(tf);
  }
  return !collision_checker_->IsInCollision(world_);
}
