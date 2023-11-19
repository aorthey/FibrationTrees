#include "dart/dart.hpp"

#include "ompl/util/ClassForward.h"

OMPL_CLASS_FORWARD(CollisionChecker);

class CollisionChecker {

 public:
  explicit CollisionChecker(const std::vector<dart::dynamics::SkeletonPtr>& group1, const std::vector<dart::dynamics::SkeletonPtr>& group2) :
    group1_(group1),
    group2_(group2) {
  }

  bool IsInCollision(const dart::simulation::WorldPtr& world) {
    auto collisionEngine
        = world->getConstraintSolver()->getCollisionDetector();

    for(const auto& rhs : group1_) {
      for(const auto& lhs : group2_) {
        auto rhsGroup = collisionEngine->createCollisionGroup(rhs.get());
        auto lhsGroup = collisionEngine->createCollisionGroup(lhs.get());

        dart::collision::CollisionOption option;
        dart::collision::CollisionResult result;
        bool collision = collisionEngine->collide(rhsGroup.get(), lhsGroup.get(), option, &result);

        for(const auto& body_node : result.getCollidingBodyNodes()) {
          std::cout << "Colliding body: " << body_node->getName() << std::endl;
        }
        if(collision) {
          return true;
        }
      }
    }
    return false;
  }


  private:
    std::vector<dart::dynamics::SkeletonPtr> group1_;
    std::vector<dart::dynamics::SkeletonPtr> group2_;
};

