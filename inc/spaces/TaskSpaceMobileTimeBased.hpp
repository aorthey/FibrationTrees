#pragma once

#include <ompl/base/StateSpace.h>

#include "robots/Robot.hpp"
#include "spaces/TaskSpaceMobile.hpp"

OMPL_CLASS_FORWARD(TaskSpaceMobileTimeBased);

class TaskSpaceMobileTimeBased : public ompl::base::CompoundStateSpace {
 public:

  explicit TaskSpaceMobileTimeBased(const RobotPtr& robot, double vMax, double tMax);

  ~TaskSpaceMobileTimeBased();

  //void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

  double distance(const ompl::base::State *from, const ompl::base::State *to) const override;

  bool isMetricSpace() const override;

  double getVMax() const;

 protected:
  /** \brief The maximum velocity of the space. */
  double vMax_;

  /** \brief The epsilon for time distance calculation. */
  double eps_ = std::numeric_limits<float>::epsilon();

  KinematicsSolverPtr kinematics_solver_;
  RobotPtr robot_;
};
