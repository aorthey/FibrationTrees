#pragma once

#include <optional>
#include "Robot.hpp"

template <typename RobotType>
class RobotFactory {
  public:
    RobotFactory() = default;

    std::shared_ptr<RobotType> Create() {
      auto robot = std::make_shared<RobotType>();

      auto skeleton = robot->MakeSkeleton();
      robot->SetSkeleton(skeleton);

      if(!world_.has_value()) {
        OMPL_ERROR("Requires world");
        throw "RequiresWorld";
      }
      if(!obstacles_.has_value()) {
        OMPL_ERROR("Requires obstacles");
        throw "RequiresObstacles";
      }

      auto factor = robot->MakeSpaceInformation(skeleton);
      robot->SetSpaceInformation(factor);

      robot->MakeCollisionChecker(factor, world_.value(), obstacles_.value());
      //auto validity_checker = robot->MakeValidityChecker(world_, obstacles_);
      //factor->setStateValidityChecker(validity_checker);
      //

      // if(world_.has_value() && obstacles_.has_value()) {
      //   // robot->MakeCollisionChecker(factor, world_.value(), obtacles_.value());
      //   std::vector<dart::dynamics::SkeletonPtr> collision_group_robot = {robot->GetSkeleton()};
      //   auto collision_checker = std::make_shared<CollisionChecker>(world_.value(), collision_group_robot, obstacles_.value());
      //   robot->SetCollisionChecker(collision_checker);

      //   auto func = [robot](const ompl::base::State* state) -> bool
      //   {
      //     return robot->IsValid(state);
      //   };
      //   robot->GetSpaceInformation()->setStateValidityChecker(func);
      // } else {
      //   auto validity_checker = std::make_shared<ompl::base::AllValidStateValidityChecker>(factor);
      //   robot->GetSpaceInformation()->setStateValidityChecker(validity_checker);
      //   OMPL_WARN("No collision_checker set.");
      // }
      return robot;
    }
  
    void SetWorld(const dart::simulation::WorldPtr& world) {
      world_ = world;
    }
    void SetObstacles(const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
      obstacles_ = obstacles;
    }
  private:
    std::optional<std::vector<dart::dynamics::SkeletonPtr>> obstacles_;
    std::optional<dart::simulation::WorldPtr> world_;
};


template <typename RobotType>
std::shared_ptr<RobotType> MakeRobot(const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  RobotFactory<RobotType> robot_factory;
  robot_factory.SetWorld(world);
  robot_factory.SetObstacles(obstacles);
  auto robot = robot_factory.Create();
  world->addSkeleton(robot->GetSkeleton());
  return robot;
}

template <typename RobotType>
std::shared_ptr<RobotType> MakeRobot(const dart::simulation::WorldPtr& world, const dart::dynamics::SkeletonPtr& obstacle) {
  return MakeRobot<RobotType>(world, std::vector{obstacle});
}

template <typename RobotType>
std::shared_ptr<RobotType> MakeRobot(const dart::simulation::WorldPtr& world) {
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  return MakeRobot<RobotType>(world, obstacles);
}
template <typename RobotType>
std::shared_ptr<RobotType> MakeRobot() {
  dart::simulation::WorldPtr world(new dart::simulation::World);
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  return MakeRobot<RobotType>(world, obstacles);
}
