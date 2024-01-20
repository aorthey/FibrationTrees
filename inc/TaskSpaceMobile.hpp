#pragma once

#include <ompl/base/StateSpace.h>

#include "robots/Robot.hpp"
#include "KinematicsSolver.hpp"

OMPL_CLASS_FORWARD(TaskSpace);

class TaskSpaceMobile : public ompl::base::CompoundStateSpace {
 public:

  explicit TaskSpaceMobile(const RobotPtr& robot);

  ~TaskSpaceMobile();

  void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

  double distance(const ompl::base::State *from, const ompl::base::State *to) const override;

 private:
  KinematicsSolverPtr kinematics_solver_;
  RobotPtr robot_;
};
