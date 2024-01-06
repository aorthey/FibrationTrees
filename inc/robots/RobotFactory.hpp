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
      auto factor = robot->MakeSpaceInformation(skeleton);
      robot->SetSpaceInformation(factor);
      if(world_.has_value() && obstacles_.has_value()) {
        std::vector<dart::dynamics::SkeletonPtr> collision_group_robot = {robot->GetSkeleton()};
        auto collision_checker = std::make_shared<CollisionChecker>(world_.value(), collision_group_robot, obstacles_.value());
        robot->SetCollisionChecker(collision_checker);

        auto func = [robot](const ompl::base::State* state) -> bool
        {
          return robot->IsValid(state);
        };
        robot->GetSpaceInformation()->setStateValidityChecker(func);
      } else {
        auto validity_checker = std::make_shared<ompl::base::AllValidStateValidityChecker>(factor);
        robot->GetSpaceInformation()->setStateValidityChecker(validity_checker);
        OMPL_WARN("No collision_checker set.");
      }
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
RobotPtr MakeRobot(const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  RobotFactory<RobotType> robot_factory;
  robot_factory.SetWorld(world);
  robot_factory.SetObstacles(obstacles);
  auto robot = robot_factory.Create();
  world->addSkeleton(robot->GetSkeleton());
  return robot;
}

template <typename RobotType>
RobotPtr MakeRobot(const dart::simulation::WorldPtr& world, const dart::dynamics::SkeletonPtr& obstacle) {
  return MakeRobot<RobotType>(world, std::vector{obstacle});
}
