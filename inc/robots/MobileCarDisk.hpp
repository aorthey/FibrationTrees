#pragma once

#include "robots/EuclideanRobot.hpp"

class MobileCarDisk : public EuclideanRobot {
  public:
    MobileCarDisk() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
};

