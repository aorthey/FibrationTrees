#pragma once

#pragma once

#include "robots/EuclideanRobot.hpp"

class ZeppelinRobot : public Robot {
  public:
    ZeppelinRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const dart::dynamics::SkeletonPtr& skeleton) override;

    Eigen::VectorXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const Eigen::VectorXd& v, ompl::base::State* state) const override;
};
