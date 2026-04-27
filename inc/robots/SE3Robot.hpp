#pragma once

#include "robots/Robot.hpp"
#include "State.hpp"

class SE3Robot : public Robot {
  public:
    SE3Robot() = default;

    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    void SetLimits(const std::pair<State3d, State3d>& limits);

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;
  private:
    std::string urdf_filename_;
};


