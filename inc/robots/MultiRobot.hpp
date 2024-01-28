#pragma once

#include "robots/Robot.hpp"

class MultiRobot : public Robot {
  public:
    MultiRobot(const std::vector<RobotPtr>& robots);
    ~MultiRobot();

    dart::dynamics::SkeletonPtr MakeSkeleton() override;
    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    Eigen::VectorXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const Eigen::VectorXd& v, ompl::base::State* state) const override;

    std::vector<Eigen::Vector3d> GetFK(const Eigen::VectorXd& config) const override;
    void SetConfiguration(const Eigen::VectorXd& config) override;

    static std::shared_ptr<MultiRobot> MakeMultiRobot(const std::vector<RobotPtr>& robots);
  private:
    std::vector<RobotPtr> robots_;

};
