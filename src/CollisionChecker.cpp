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
  factor->printState(state);
  factor->project(state, tmp_skeleton_states_);

  for(const auto& child : factor->getChildren()) {
    const auto& name = child->getName();
    const auto it = tmp_skeleton_states_.find(name);
    if(it == tmp_skeleton_states_.end()) {
      OMPL_ERROR("Could not find factor %s", name.c_str());
      throw "FactorNotExisting";
    }
    const auto config = StateToEigenVectorXd(child, it->second);
    const auto sit = skeletons_.find(name);
    if(sit == skeletons_.end()) {
      OMPL_ERROR("Could not find factor %s in skeletons.", name.c_str());
      throw "FactorNotExisting";
    }
    const auto& skeleton = sit->second;
    const auto lb = skeleton->getPositionLowerLimits();
    const auto ub = skeleton->getPositionUpperLimits();
    for(size_t k = 0; k < config.size(); k++) {
      if(config[k] < lb[k] || config[k] > ub[k] || config[k] != config[k]) {
        std::cout <<"Out of bounds: " << config[k] << " not in ["<< lb[k] << "," << ub[k] <<"]" << std::endl;

        return false;
      }
    }
    std::cout <<"Set config for " << name << " to "<< config << std::endl;
    skeleton->setConfiguration(config);
  }


  return !collision_checker_->IsInCollision(world_);
}
