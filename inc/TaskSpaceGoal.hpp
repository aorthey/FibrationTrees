#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/multilevel/datastructures/Projection.h>

class TaskSpaceGoal : public ompl::base::GoalState {

  public:
    //si : Input state space
    //state : State on state space si
    //projection: From state space to a base space
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

class TaskSpaceGoalBaseState : public ompl::base::GoalSampleableRegion {

  public:
    //si : Input base space
    //state : State on base space
    //projection: From state space to a base space
    TaskSpaceGoalBaseState(const ompl::base::SpaceInformationPtr& si, ompl::base::State* baseState, const ompl::multilevel::ProjectionPtr& projection);
    ~TaskSpaceGoalBaseState();
    void sampleGoal(ompl::base::State *state) const override;
    unsigned int maxSampleCount() const override;
    double distanceGoal(const ompl::base::State *state) const override;

  private:
    ompl::multilevel::ProjectionPtr projection_;
    ompl::base::State* baseState_;
    ompl::base::State* tmpBaseState_;

};

