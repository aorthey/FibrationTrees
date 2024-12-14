#pragma once

#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "EigenPath.hpp"
#include <ompl/base/Path.h>

class TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints : public TimeBasedMobileKukaRobotTaskSpace {
  public:
    TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints() = default;
};

