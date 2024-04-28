#pragma once

#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "EigenPath.hpp"
#include <ompl/base/Path.h>

class TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints : public TimeBasedMobileKukaRobotTaskSpace {
  public:
    TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints() = default;

    bool IsValid(const ompl::base::State* state) const override;

    void AddDynamicalObstacle(const std::pair<RobotPtr, ompl::base::PathPtr>& obstacle);

  private:
    std::vector<std::pair<RobotPtr, std::shared_ptr<EigenPath>>> obstacles_;
};

