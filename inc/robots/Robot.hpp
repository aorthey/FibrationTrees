#pragma once

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "CollisionChecker.hpp"

class Robot {
  public:
    Robot() = default;

    virtual dart::dynamics::SkeletonPtr MakeSkeleton() = 0;
    virtual ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const dart::dynamics::SkeletonPtr& skeleton) = 0;

    virtual Eigen::VectorXd StateToEigen(const ompl::base::State* state) const = 0;
    virtual void EigenToState(const Eigen::VectorXd& v, ompl::base::State* state) const = 0;

    const std::shared_ptr<dart::dynamics::Skeleton>& GetSkeleton();
    const std::shared_ptr<ompl::multilevel::FactoredSpaceInformation>& GetSpaceInformation();
    const std::shared_ptr<CollisionChecker>& GetCollisionChecker();

    size_t GetDimension() const;

    void SetSkeleton(const dart::dynamics::SkeletonPtr& skeleton);
    void SetSpaceInformation(const ompl::multilevel::FactoredSpaceInformationPtr& factor);
    void SetCollisionChecker(const CollisionCheckerPtr& collision_checker);

    bool IsValid(const ompl::base::State* state) const;

  protected:
    dart::dynamics::SkeletonPtr skeleton_;
    ompl::multilevel::FactoredSpaceInformationPtr factor_;
    CollisionCheckerPtr collision_checker_;
};

typedef std::shared_ptr<Robot> RobotPtr;
