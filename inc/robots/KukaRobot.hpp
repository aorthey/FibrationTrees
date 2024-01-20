#pragma once

#include "robots/EuclideanRobot.hpp"

class KukaRobot : public EuclideanRobot {
  public:
    KukaRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
};
