#pragma once

#include "robots/EuclideanRobot.hpp"

class MobileKukaBase : public EuclideanRobot {
  public:
    MobileKukaBase() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;
};

