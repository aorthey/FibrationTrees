#pragma once

#include "robots/SE2Robot.hpp"

class MobileCarForklift : public SE2Robot {
  public:
    MobileCarForklift() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    // ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    // ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;

    // std::vector<State3d> GetFK(const StateXd& config) const override;

    // StateXd StateToEigen(const ompl::base::State* state) const override;
    // void EigenToState(const StateXd& v, ompl::base::State* state) const override;
};

