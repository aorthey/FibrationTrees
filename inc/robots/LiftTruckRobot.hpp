#pragma once

#include "robots/SE2Robot.hpp"
#include "State.hpp"

class LiftTruckRobot : public SE2Robot {
  public:
    LiftTruckRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
};

