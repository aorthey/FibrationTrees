#pragma once

#include "dart/dart.hpp"

#include "ompl/util/ClassForward.h"
#include "ompl/base/StateValidityChecker.h"

OMPL_CLASS_FORWARD(CollisionChecker);
OMPL_CLASS_FORWARD(Robot);

const Eigen::Vector3d kCollisionColor = Eigen::Vector3d(0.8, 0.3, 0.3);

class CollisionChecker {

 public:
  explicit CollisionChecker(const dart::simulation::WorldPtr& world, 
      const std::vector<dart::dynamics::SkeletonPtr>& group1, 
      const std::vector<dart::dynamics::SkeletonPtr>& group2);

  virtual bool IsInCollision(const dart::simulation::WorldPtr& world);
  virtual bool IsInCollision();

  void PrintCollisionInfo();

 private:
  dart::simulation::WorldPtr world_;
  std::vector<dart::dynamics::SkeletonPtr> group1_;
  std::vector<dart::dynamics::SkeletonPtr> group2_;
  std::unordered_map<const dart::dynamics::ShapeNode*, Eigen::Vector3d> default_colors_;
};

class MultiCollisionChecker : public CollisionChecker {
 public:
  explicit MultiCollisionChecker(const dart::simulation::WorldPtr& world, 
      const std::vector<CollisionCheckerPtr>& collision_checkers);

  bool IsInCollision(const dart::simulation::WorldPtr& world);

 private:
  std::vector<CollisionCheckerPtr> collision_checkers_;
};

class DartWorldCollisionChecker : public ompl::base::StateValidityChecker
{
 public:
    DartWorldCollisionChecker(const ompl::base::SpaceInformationPtr si, 
        const dart::simulation::WorldPtr& world,
        const dart::dynamics::SkeletonPtr& skeleton,
        const CollisionCheckerPtr& collision_checker);

    ~DartWorldCollisionChecker() = default;

    bool isValid(const ompl::base::State *state) const override;

 protected:
  dart::dynamics::SkeletonPtr skeleton_;
  dart::simulation::WorldPtr world_;
  CollisionCheckerPtr collision_checker_;
};

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

// class DartTransformCollisionChecker : public DartWorldCollisionChecker
// {
//  public:
//     DartTransformCollisionChecker(const ompl::base::SpaceInformationPtr si, 
//         const dart::simulation::WorldPtr& world,
//         const dart::dynamics::SkeletonPtr& skeleton,
//         const CollisionCheckerPtr& collision_checker);

//     bool isValid(const ompl::base::State *state) const override;
// };

class DartMultiRobotCollisionChecker : public ompl::base::StateValidityChecker
{
 public:
    DartMultiRobotCollisionChecker(const ompl::base::SpaceInformationPtr& si, 
      const dart::simulation::WorldPtr& world,
      const std::unordered_map<std::string, dart::dynamics::SkeletonPtr>& manipulators,
      const CollisionCheckerPtr& collision_checker);

    ~DartMultiRobotCollisionChecker();

    bool isValid(const ompl::base::State *state) const override;

 protected:
    std::unordered_map<std::string, dart::dynamics::SkeletonPtr> skeletons_;
    std::unordered_map<std::string, ompl::base::State*> tmp_skeleton_states_;

    dart::simulation::WorldPtr world_;
    CollisionCheckerPtr collision_checker_;
};
