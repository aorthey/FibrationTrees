#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/multilevel/datastructures/Projection.h>

class TaskSpaceGoal : public ompl::base::GoalState {

  public:
    TaskSpaceGoal(const ompl::base::SpaceInformationPtr& si, ompl::base::State* state, const ompl::multilevel::ProjectionPtr& projection);
    ~TaskSpaceGoal();
    void sampleGoal(ompl::base::State *state) const override;
    unsigned int maxSampleCount() const override;
    double distanceGoal(const ompl::base::State *state) const override;

  private:
    ompl::multilevel::ProjectionPtr projection_;
    ompl::base::State* baseState_;
    ompl::base::State* tmpBaseState_;

};

