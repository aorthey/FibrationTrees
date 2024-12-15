#pragma once

#include "robots/Robot.hpp"

class MobileKukaRobot : public Robot {
  public:
    MobileKukaRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;

    std::vector<State3d> GetFK(const StateXd& config) const override;
};

