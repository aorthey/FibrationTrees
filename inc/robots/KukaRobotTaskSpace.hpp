#pragma once

#include "robots/KukaRobot.hpp"

class KukaRobotTaskSpace : public KukaRobot {
  public:
    KukaRobotTaskSpace() = default;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;
};
