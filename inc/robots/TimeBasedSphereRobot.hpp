#pragma once

#include "robots/Robot.hpp"

class TimeBasedSphereRobot : public Robot {
  public:
    TimeBasedSphereRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;

    //void SetSpaceInformationFromRobot(const RobotPtr& robot,
    //const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);

    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;
    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;

    float StateToTime(const ompl::base::State* state) const override;
    void TimeToState(const float time, ompl::base::State* state) const override;

    void SetLimits(const std::pair<State3d, State3d>& limits);
};

