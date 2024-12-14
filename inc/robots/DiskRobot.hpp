#pragma once

#include "robots/EuclideanRobot.hpp"
#include "State.hpp"

class DiskRobot : public EuclideanRobot {
  public:
    DiskRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;
    void SetLimits(const std::pair<State2d, State2d>& limits);

  private:
    double radius_{1.0};
    double height_{0.3};
};

