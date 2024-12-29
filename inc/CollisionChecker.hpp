#pragma once

#include <dart/dart.hpp>

#include <ompl/util/ClassForward.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "State.hpp"

OMPL_CLASS_FORWARD(CollisionChecker);
OMPL_CLASS_FORWARD(Robot);
OMPL_CLASS_FORWARD(MultiRobot);

const State3d kCollisionColor = State3d(0.8, 0.3, 0.3);

////////////////////////////////////////////////////////////////////////////////
/// Collision Checker
////////////////////////////////////////////////////////////////////////////////

class CollisionChecker {

 public:
  explicit CollisionChecker(const dart::simulation::WorldPtr& world, 
      const std::vector<dart::dynamics::SkeletonPtr>& group1, 
      const std::vector<dart::dynamics::SkeletonPtr>& group2);

  virtual bool IsInCollision(const dart::simulation::WorldPtr& world);
  virtual bool IsInCollision();

  virtual void PrintCollisionInfo();

 private:
  dart::simulation::WorldPtr world_;
  std::vector<dart::dynamics::SkeletonPtr> group1_;
  std::vector<dart::dynamics::SkeletonPtr> group2_;
  std::unordered_map<const dart::dynamics::ShapeNode*, State3d> default_colors_;
};

class MultiCollisionChecker : public CollisionChecker {
 public:
  explicit MultiCollisionChecker(const dart::simulation::WorldPtr& world, 
      const std::vector<CollisionCheckerPtr>& collision_checkers);

  bool IsInCollision(const dart::simulation::WorldPtr& world);
  void PrintCollisionInfo() override;

 private:
  std::vector<CollisionCheckerPtr> collision_checkers_;
};


////////////////////////////////////////////////////////////////////////////////
/// State Validity Checker
////////////////////////////////////////////////////////////////////////////////

class RobotToObstaclesCollisionChecker : public ompl::base::StateValidityChecker
{
 public:
    RobotToObstaclesCollisionChecker(
      const dart::simulation::WorldPtr& world,
      const RobotPtr& robot,
      const CollisionCheckerPtr& collision_checker);

    ~RobotToObstaclesCollisionChecker() = default;

    bool isValid(const ompl::base::State *state) const override;

 protected:
  dart::simulation::WorldPtr world_;
  RobotPtr robot_;
  CollisionCheckerPtr collision_checker_;
};

class MultiRobotCollisionChecker : public ompl::base::StateValidityChecker
{
 public:
    MultiRobotCollisionChecker(const dart::simulation::WorldPtr& world,
      const std::shared_ptr<MultiRobot>& multi_robot);

    ~MultiRobotCollisionChecker();

    CollisionCheckerPtr GetCollisionChecker() const;

    bool isValid(const ompl::base::State *state) const override;

 protected:
    std::vector<RobotPtr> robots_;
    MultiRobotPtr multi_robot_;

    dart::simulation::WorldPtr world_;
    CollisionCheckerPtr collision_checker_;
};
