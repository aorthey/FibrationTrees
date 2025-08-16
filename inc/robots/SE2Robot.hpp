#pragma once

#include "robots/EuclideanRobot.hpp"
#include "State.hpp"

class SE2Robot : public EuclideanRobot {
  public:
    SE2Robot() = default;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    void SetLimits(const std::pair<State2d, State2d>& limits);

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;
};

