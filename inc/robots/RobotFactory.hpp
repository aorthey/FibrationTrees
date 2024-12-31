#pragma once

#include <optional>
#include "Robot.hpp"
#include "yaml-cpp/yaml.h"

template <typename RobotType>
class RobotFactory {
  public:
    RobotFactory() = default;

    std::shared_ptr<RobotType> Create() {
      if(!world_.has_value()) {
        throw std::invalid_argument("Requires World");
      }
      if(!obstacles_.has_value()) {
        throw std::invalid_argument("Requires Obstacles");
      }
      if(!node_.has_value()) {
        throw std::invalid_argument("Requires YAML Node");
      }

      auto robot = std::make_shared<RobotType>();

      ////////////////////////////////////////////////////////////////////////////////
      // Make Skeleton
      ////////////////////////////////////////////////////////////////////////////////
      auto skeleton = robot->MakeSkeleton(node_.value());
      robot->SetSkeleton(skeleton);

      ////////////////////////////////////////////////////////////////////////////////
      // Make SpaceInformation
      ////////////////////////////////////////////////////////////////////////////////
      auto factor = robot->MakeSpaceInformation(robot);
      robot->SetSpaceInformation(factor);

      ////////////////////////////////////////////////////////////////////////////////
      // Make CollisionChecker
      ////////////////////////////////////////////////////////////////////////////////
      auto collision_checker = robot->MakeCollisionChecker(factor, world_.value(), obstacles_.value());
      robot->SetCollisionChecker(collision_checker);

      ////////////////////////////////////////////////////////////////////////////////
      // Make MotionValidator
      ////////////////////////////////////////////////////////////////////////////////
      auto motion_validator = robot->MakeMotionValidator(factor, robot);
      factor->setMotionValidator(motion_validator);

      return robot;
    }
  
    void SetWorld(const dart::simulation::WorldPtr& world) {
      world_ = world;
    }
    void SetObstacles(const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
      obstacles_ = obstacles;
    }
    void SetNode(const YAML::Node& node) {
      node_ = node;
    }
  private:
    std::optional<std::vector<dart::dynamics::SkeletonPtr>> obstacles_;
    std::optional<dart::simulation::WorldPtr> world_;
    std::optional<YAML::Node> node_;
};


template <typename RobotType>
std::shared_ptr<RobotType> MakeRobot(const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles, const YAML::Node& node) {
  RobotFactory<RobotType> robot_factory;
  robot_factory.SetWorld(world);
  robot_factory.SetObstacles(obstacles);
  robot_factory.SetNode(node);
  auto robot = robot_factory.Create();
  world->addSkeleton(robot->GetSkeleton());
  return robot;
}
template <typename RobotType>
std::shared_ptr<RobotType> MakeRobot(const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  return MakeRobot<RobotType>(world, obstacles, YAML::Node());
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
