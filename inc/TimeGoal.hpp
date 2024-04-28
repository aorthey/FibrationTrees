#pragma once

#include <ompl/base/goals/GoalState.h>

#include "robots/Robot.hpp"

class TimeGoal : public ompl::base::GoalState
{
public:
    TimeGoal(const RobotPtr &robot, float vMax, const ompl::base::State *start, const ompl::base::State *goal);

    ~TimeGoal() {};

    void sampleGoal(ompl::base::State *state) const override;
    unsigned int maxSampleCount() const override;
    double distanceGoal(const ompl::base::State *state) const override;

private:
    RobotPtr robot_;
    float vMax_{0.0};
    float minTime_{0.0};
    float maxTime_{0.0};
};
