#pragma once

#include "robots/MobileKukaRobot.hpp"

class MobileKukaRobotTaskSpace : public MobileKukaRobot {
  public:
    MobileKukaRobotTaskSpace() = default;

    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;

    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;
};

