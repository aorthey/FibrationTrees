#pragma once

#include "robots/SE2Robot.hpp"
#include "State.hpp"

class CubeRobot : public SE2Robot {
  public:
    CubeRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;

  private:
    double width_{1.0};
    double length_{1.0};
    double height_{1.0};
};

