#include "TaskSpaceGoal.hpp"

TaskSpaceGoal::TaskSpaceGoal(const ompl::base::SpaceInformationPtr& si, ompl::base::State* state, const ompl::multilevel::ProjectionPtr& projection) 
  : ompl::base::GoalState(si), projection_(projection) {
  tmpBaseState_ = projection_->getBase()->allocState();
  baseState_ = projection_->getBase()->allocState();
  projection_->project(state, baseState_);
  setState(state);
}

TaskSpaceGoal::~TaskSpaceGoal() {
  projection_->getBase()->freeState(tmpBaseState_);
  projection_->getBase()->freeState(baseState_);
}

void TaskSpaceGoal::sampleGoal(ompl::base::State *state) const {
  projection_->lift(baseState_, state);
}

unsigned int TaskSpaceGoal::maxSampleCount() const {
  return 100;
}

double TaskSpaceGoal::distanceGoal(const ompl::base::State *state) const {
  projection_->project(state, tmpBaseState_);
  return projection_->getBase()->distance(tmpBaseState_, baseState_);
}
