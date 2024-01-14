#pragma once

#include "robots/EuclideanRobot.hpp"

class SphereRobot : public EuclideanRobot {
  public:
    SphereRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;
    void SetLimits(const std::pair<Eigen::Vector3d, Eigen::Vector3d>& limits);
};

