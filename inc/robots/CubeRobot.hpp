#pragma once

#include "robots/EuclideanRobot.hpp"
#include "State.hpp"

class CubeRobot : public EuclideanRobot {
  public:
    CubeRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    void SetLimits(const std::pair<State2d, State2d>& limits);

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;

  private:
    double width_{1.0};
    double length_{1.0};
    double height_{1.0};
};

