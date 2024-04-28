#pragma once

#include <ompl/base/StateSpace.h>

#include "robots/Robot.hpp"

OMPL_CLASS_FORWARD(EuclideanTimeBased);

class EuclideanTimeBased : public ompl::base::CompoundStateSpace {
 public:

  explicit EuclideanTimeBased(const RobotPtr& robot, double tMax, double vMax);

  ~EuclideanTimeBased();

  double distance(const ompl::base::State *from, const ompl::base::State *to) const override;

  bool isMetricSpace() const override;

 private:
  RobotPtr robot_;
  double vMax_;
};
