#pragma once

#include "robots/SphereRobot.hpp"

class ZeppelinInnerSphereRobot : public SphereRobot {
  public:
    ZeppelinInnerSphereRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
};
