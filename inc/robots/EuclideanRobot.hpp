#pragma once

#include "robots/Robot.hpp"

//Robot which has state space R^N
class EuclideanRobot : public Robot {
  public:
    EuclideanRobot() = default;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const dart::dynamics::SkeletonPtr& skeleton) override;
    Eigen::VectorXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const Eigen::VectorXd& v, ompl::base::State* state) const override;
};

