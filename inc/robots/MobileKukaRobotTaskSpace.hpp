#pragma once

#include "robots/MobileKukaRobot.hpp"

class MobileKukaRobotTaskSpace : public MobileKukaRobot {
  public:
    MobileKukaRobotTaskSpace() = default;

    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;

    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;

    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;

    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;

    std::vector<State3d> GetFK(const StateXd& config) const override;

  protected:
    std::optional<std::pair<State3d, State3d>> tcp_limits_{std::nullopt};
};

