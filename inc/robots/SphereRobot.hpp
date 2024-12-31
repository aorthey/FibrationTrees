#pragma once

#include "robots/EuclideanRobot.hpp"

class SphereRobot : public EuclideanRobot {
  public:
    SphereRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    void SetLimits(const std::pair<State3d, State3d>& limits);
};

