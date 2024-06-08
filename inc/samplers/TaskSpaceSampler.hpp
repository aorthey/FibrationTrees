#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include "KinematicsSolver.hpp"
#include "State.hpp"
#include "robots/Robot.hpp"

class TaskSpaceSampler : public ompl::base::StateSampler {
  public:
    TaskSpaceSampler(const RobotPtr& robot, const std::pair<State3d, State3d>& limits);
    void sampleUniform(ompl::base::State *state) override;
    void sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) override;
    void sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) override;

  protected:
    KinematicsSolverPtr kinematics_solver_;
    std::pair<State3d, State3d> limits_;
    RobotPtr robot_;
};

ompl::base::StateSamplerPtr allocateTaskSpaceSampler(const RobotPtr& robot, const std::pair<State3d, State3d>& limits);
