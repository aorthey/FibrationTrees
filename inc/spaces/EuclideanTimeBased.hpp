#pragma once

#include <ompl/base/StateSpace.h>

#include "robots/Robot.hpp"

OMPL_CLASS_FORWARD(EuclideanTimeBased);

class EuclideanTimeBased : public ompl::base::CompoundStateSpace {
 public:

  explicit EuclideanTimeBased(const RobotPtr& robot, double tMax);

  ~EuclideanTimeBased();

  bool isMetricSpace() const override;
};
