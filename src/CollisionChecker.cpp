#include "CollisionChecker.hpp"

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

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
        // for(const auto& body : result.getCollidingBodyNodes()) {
        //   OMPL_WARN("Collision body %s", body->getName().c_str());
        // }
        // auto N = world->getNumSkeletons();
        // for(size_t k =0; k<N; k++) {
        //   const auto& skeleton = world->getSkeleton(k);
        //   for(const auto& body_node : skeleton->getBodyNodes()) {
        //     if(!result.inCollision(body_node)) {
        //       continue;
        //     }
        //     OMPL_WARN("Collision body %s", skeleton->getName().c_str());
        //   }
        // }
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
            OMPL_ERROR("In Collision: %s--%s--%s", skeleton->getName().c_str(), body_node->getName().c_str(), node->getName().c_str());
            std::shared_ptr<dart::dynamics::MeshShape> mesh =
                std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
                        node->getShape());
            if(mesh) {
              node->getVisualAspect()->setColor(kCollisionColor);
              mesh->incrementVersion();
              mesh->refreshData();
            }
            auto properties(node->getVisualAspect()->getProperties());
            properties.mHidden = false;
            properties.mUseDefaultColor = false;
            // properties.mRGBA = kCollisionColor;
            node->getVisualAspect()->setProperties(properties);
            node->getVisualAspect()->notifyPropertiesUpdated();
            node->getVisualAspect()->incrementVersion();
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
        visual_node->getVisualAspect()->show();
      }
    }
  }
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

void MultiCollisionChecker::ColorAllCollisionBodies(dart::simulation::WorldPtr world) {
  for(const auto& collision_checker : collision_checkers_) {
    collision_checker->ColorAllCollisionBodies(world);
  }
}

void MultiCollisionChecker::ResetColors(const dart::simulation::WorldPtr& world) {
  for(const auto& collision_checker : collision_checkers_) {
    collision_checker->ResetColors(world);
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

// DartTransformCollisionChecker::DartTransformCollisionChecker(const ompl::base::SpaceInformationPtr si, 
//     const dart::simulation::WorldPtr& world,
//     const dart::dynamics::SkeletonPtr& skeleton,
//     const CollisionCheckerPtr& collision_checker)
//   : DartWorldCollisionChecker(si, world, skeleton, collision_checker)
// {
// }

// bool DartTransformCollisionChecker::isValid(const ompl::base::State *state) const
// {
//   auto config = StateToEigenVector3d(state);
//   Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
//   tf.translation() = config;

//   for(const auto& body_node : skeleton_->getBodyNodes()) {
//     body_node->getParentJoint()->setTransformFromParentBodyNode(tf);
//   }
//   return !collision_checker_->IsInCollision(world_);
// }

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
