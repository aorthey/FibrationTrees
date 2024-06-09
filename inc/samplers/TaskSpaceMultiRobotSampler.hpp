#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include "KinematicsSolver.hpp"
#include "State.hpp"
#include "robots/Robot.hpp"

class TaskSpaceMultiRobotSampler : public ompl::base::StateSampler {
  public:
    TaskSpaceMultiRobotSampler(const RobotPtr& joint_robot, 
        const std::vector<RobotPtr>& robots, 
        const std::vector<std::pair<State3d, State3d>>& limits);

    void sampleUniform(ompl::base::State *state) override;
    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

  protected:
    RobotPtr joint_robot_;
    std::vector<RobotPtr> robots_;
    std::vector<std::pair<State3d, State3d>> limits_;
    std::vector<KinematicsSolverPtr> kinematics_solvers_;
    size_t dimension_{0};
};

ompl::base::StateSamplerPtr allocateTaskSpaceMultiRobotSampler(const RobotPtr& robot, 
    const std::vector<RobotPtr>& robots, const std::vector<std::pair<State3d, State3d>>& limits);
