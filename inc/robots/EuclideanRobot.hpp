#pragma once

#include "robots/Robot.hpp"

//Robot which has state space R^N
class EuclideanRobot : public Robot {
  public:
    EuclideanRobot() = default;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;
};

