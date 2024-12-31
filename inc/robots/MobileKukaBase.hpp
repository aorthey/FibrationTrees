#pragma once

#include "robots/Robot.hpp"

class MobileKukaBase : public Robot {
  public:
    MobileKukaBase() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;
};

