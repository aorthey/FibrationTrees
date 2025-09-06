#pragma once

#include "robots/SE2Robot.hpp"

class MobileCarForklift : public SE2Robot {
  public:
    MobileCarForklift() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
};

