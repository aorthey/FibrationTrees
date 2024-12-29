#pragma once

#include "robots/Robot.hpp"

class MultiRobot : public Robot {
  public:
    MultiRobot(const std::vector<RobotPtr>& robots);
    ~MultiRobot();

    dart::dynamics::SkeletonPtr MakeSkeleton() override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;

    std::vector<State3d> GetFK(const StateXd& config) const override;
    void SetConfiguration(const StateXd& config) override;
    bool IsMultiRobot() const override;
    bool HasValidJointLimits(const Eigen::VectorXd& config) const override;

    std::vector<RobotPtr> GetSubRobots() const;

    static std::shared_ptr<MultiRobot> MakeMultiRobot(const std::vector<RobotPtr>& robots);
  private:
    std::vector<RobotPtr> robots_;
};
