#pragma once

#include "robots/MobileKukaRobotTaskSpace.hpp"

class TimeBasedMobileKukaRobotTaskSpace : public MobileKukaRobotTaskSpace {
  public:
    TimeBasedMobileKukaRobotTaskSpace() = default;

    void SetSpaceInformationFromRobot(const RobotPtr& robot,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);

    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    // ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;
    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;
};

