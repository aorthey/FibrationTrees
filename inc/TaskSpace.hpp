#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "TaskSpaceProjection.hpp"
#include "OmplHelper.hpp"

OMPL_CLASS_FORWARD(TaskSpace);

class TaskSpace : public ompl::base::RealVectorStateSpace {
 public:

  explicit TaskSpace(unsigned int dim, const KinematicsSolverPtr& kinematics_solver);

  ~TaskSpace();

  void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

  double distance(const ompl::base::State *from, const ompl::base::State *to) const override;

 private:
  KinematicsSolverPtr kinematics_solver_;
};
