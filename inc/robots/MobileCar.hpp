#pragma once

#include "robots/MobileKukaBase.hpp"

class MobileCar : public MobileKukaBase {
  public:
    MobileCar() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;

    std::vector<State3d> GetFK(const StateXd& config) const override;
};

