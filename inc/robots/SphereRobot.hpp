#pragma once

#include "robots/EuclideanRobot.hpp"

class SphereRobot : public EuclideanRobot {
  public:
    SphereRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;
};

