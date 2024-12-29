#pragma once

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/base/Path.h>

#include "CollisionChecker.hpp"
#include "EigenPath.hpp"
#include "State.hpp"

OMPL_CLASS_FORWARD(EigenPath);

class Robot {
  public:
    Robot() = default;

    virtual dart::dynamics::SkeletonPtr MakeSkeleton() = 0;
    virtual ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) = 0;
    virtual CollisionCheckerPtr MakeCollisionChecker(
        const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
        const dart::simulation::WorldPtr& world,
        const std::vector<dart::dynamics::SkeletonPtr>& obstacles);
    virtual ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot);

    virtual StateXd StateToEigen(const ompl::base::State* state) const = 0;
    virtual float StateToTime(const ompl::base::State* state) const;
    virtual void EigenToState(const StateXd& v, ompl::base::State* state) const = 0;
    virtual void TimeToState(const float time, ompl::base::State* state) const;

    const std::shared_ptr<dart::dynamics::Skeleton>& GetSkeleton();
    const std::shared_ptr<ompl::multilevel::FactoredSpaceInformation>& GetSpaceInformation();
    const std::shared_ptr<CollisionChecker>& GetCollisionChecker();

    size_t GetDimension() const;
    virtual bool IsMultiRobot() const;

    std::string GetName() const;
    virtual void SetConfiguration(const StateXd& config);

    void SetSkeleton(const dart::dynamics::SkeletonPtr& skeleton);
    void SetSpaceInformation(const ompl::multilevel::FactoredSpaceInformationPtr& factor);
    void SetCollisionChecker(const CollisionCheckerPtr& collision_checker);

    virtual bool IsValid(const ompl::base::State* state) const;

    //Returns a vector of State3d, one for each internal robot
    virtual std::vector<State3d> GetFK(const StateXd& config) const;

    std::vector<State3d> GetFK(const ompl::base::State* state) const;

    void Hide();
    void Show();

    void AddDynamicalObstacle(const std::pair<RobotPtr, ompl::base::PathPtr>& obstacle);

    void EnabledShowPath();
    void DisableShowPath();
    bool ShouldShowPath() const;

    void EnabledSmoothPath();
    void DisableSmoothPath();
    bool ShouldSmoothPath() const;

  protected:
    dart::dynamics::SkeletonPtr skeleton_;
    ompl::multilevel::FactoredSpaceInformationPtr factor_;
    CollisionCheckerPtr collision_checker_;
    bool smooth_path_{false};
    bool show_path_{false};

  private:
    std::vector<std::pair<RobotPtr, std::shared_ptr<EigenPath>>> dynamic_obstacles_;
};

typedef std::shared_ptr<Robot> RobotPtr;
