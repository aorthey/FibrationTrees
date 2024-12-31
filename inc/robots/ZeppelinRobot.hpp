#pragma once

#pragma once

#include "robots/EuclideanRobot.hpp"

class ZeppelinRobot : public Robot {
  public:
    ZeppelinRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;
};
