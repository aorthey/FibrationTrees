#pragma once

#include "robots/MobileKukaRobotTaskSpace.hpp"

const auto kDefaultVMax = 1.0;
const auto kDefaultTMax = 20.0;

class TimeBasedMobileKukaRobotTaskSpace : public MobileKukaRobotTaskSpace {
  public:
    TimeBasedMobileKukaRobotTaskSpace() = default;
    TimeBasedMobileKukaRobotTaskSpace(float vMax, float tMax);

    void SetSpaceInformationFromRobot(const RobotPtr& robot, const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles);
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& node) override;

    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    ompl::base::MotionValidatorPtr MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) override;
    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;

    float StateToTime(const ompl::base::State* state) const override;
    void TimeToState(const float time, ompl::base::State* state) const override;

    float GetVMax() const;
    float GetTMax() const;
    void SetVMax(float vMax);
    void SetTMax(float tMax);

  private:
    float vMax_{kDefaultVMax};
    float tMax_{kDefaultTMax};

    std::optional<std::pair<State3d, State3d>> tcp_limits_{std::nullopt};
};

