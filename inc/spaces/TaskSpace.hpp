#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "projections/ProjectionTaskSpace.hpp"
#include "OmplHelper.hpp"
#include "robots/Robot.hpp"

OMPL_CLASS_FORWARD(TaskSpace);

class TaskSpace : public ompl::base::RealVectorStateSpace {
 public:

  explicit TaskSpace(const RobotPtr& robot);

  ~TaskSpace();

  void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

  double distance(const ompl::base::State *from, const ompl::base::State *to) const override;

 private:
  KinematicsSolverPtr kinematics_solver_;
  RobotPtr robot_;
};
