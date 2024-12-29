#include "CollisionChecker.hpp"

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "OmplHelper.hpp"
#include "DartHelper.hpp"
#include "robots/Robot.hpp"
#include "robots/MultiRobot.hpp"
#include "CollisionCheckerHelper.hpp"
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
  if(!collision_info.empty()) {
    OMPL_DEBUG("Colliding bodies: %s", collision_info.c_str());
  }
}

MultiCollisionChecker::MultiCollisionChecker(const dart::simulation::WorldPtr& world, 
      const std::vector<CollisionCheckerPtr>& collision_checkers) 
  : CollisionChecker(world, {}, {}), collision_checkers_(collision_checkers)
{
}

void MultiCollisionChecker::PrintCollisionInfo() {
  for(const auto& collision_checker : collision_checkers_) {
    collision_checker->PrintCollisionInfo();
  }
}

bool MultiCollisionChecker::IsInCollision(const dart::simulation::WorldPtr& world) {
  return std::any_of(collision_checkers_.begin(), collision_checkers_.end(), 
      [&](const CollisionCheckerPtr& collision_checker) {
        return collision_checker->IsInCollision(world);
      });
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
  auto config = robot_->StateToEigen(state).configuration;
  if(!robot_->IsValid(state)) {
    return false;
  }
  //Check collisions
  robot_->GetSkeleton()->setConfiguration(config);
  return !collision_checker_->IsInCollision(world_);
}

////////////////////////////////////////////////////////////////////////////////
// MultiRobotCollisionChecker
////////////////////////////////////////////////////////////////////////////////

MultiRobotCollisionChecker::MultiRobotCollisionChecker(const dart::simulation::WorldPtr& world,
  const std::shared_ptr<MultiRobot>& multi_robot) 
  : ompl::base::StateValidityChecker(multi_robot->GetSpaceInformation()), world_(world), multi_robot_(multi_robot) {

  std::vector<RobotPtr> robots = CollectAtomicRobots(multi_robot_);

  std::vector<CollisionCheckerPtr> pairwise_collision_checkers;
  for(size_t index1 = 0; index1 < robots.size(); index1++) {
    const auto& robot1 = robots.at(index1);
    //Add internal collision checker for each robot
    pairwise_collision_checkers.push_back(robot1->GetCollisionChecker());

    std::vector<dart::dynamics::SkeletonPtr> collision_robot1 = {robot1->GetSkeleton()};
    for(size_t index2 = index1 + 1; index2 < robots.size(); index2++) {
      const auto& robot2 = robots.at(index2);
      std::vector<dart::dynamics::SkeletonPtr> collision_robot2 = {robot2->GetSkeleton()};
      //Add robot robot collision checker
      auto collision_checker_robot_robot = std::make_shared<CollisionChecker>(world, collision_robot1, collision_robot2);
      pairwise_collision_checkers.push_back(collision_checker_robot_robot);
    }
  }

  collision_checker_ = std::make_shared<MultiCollisionChecker>(world, pairwise_collision_checkers);

  if(!si_->getStateSpace()->isCompound()) {
    throw std::runtime_error("MultiRobot requires compound space.");
  }
}

MultiRobotCollisionChecker::~MultiRobotCollisionChecker() {
}

bool MultiRobotCollisionChecker::isValid(const ompl::base::State *state) const {

  auto compound_state = state->as<ompl::base::CompoundState>();

  size_t index = 0;
  auto subrobots = multi_robot_->GetSubRobots();
  for(const auto& robot : subrobots) {
    const auto& name = robot->GetSpaceInformation()->getName();
    const auto eigen_state = robot->StateToEigen(compound_state->operator[](index));
    if(!robot->HasValidJointLimits(eigen_state.configuration)) {
      return false;
    }
    robot->SetConfiguration(eigen_state);
    index++;
  }
  return !collision_checker_->IsInCollision(world_);
}

CollisionCheckerPtr MultiRobotCollisionChecker::GetCollisionChecker() const {
  return collision_checker_;
}
