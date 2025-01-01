#pragma once

#include "robots/EuclideanRobot.hpp"

class MobileKukaBase : public EuclideanRobot {
  public:
    MobileKukaBase() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;
};

