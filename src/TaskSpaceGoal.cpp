#include "TaskSpaceGoal.hpp"

TaskSpaceGoal::TaskSpaceGoal(const ompl::base::SpaceInformationPtr& si, ompl::base::State* state, const ompl::multilevel::ProjectionPtr& projection) 
  : ompl::base::GoalState(si), projection_(projection) {
  if(projection_->getBundle()->getName() != si->getStateSpace()->getName()) {
    OMPL_ERROR("Space %s needs to be the parent of projection %s, but space %s is.", si->getStateSpace()->getName().c_str(), projection_->getTypeAsString().c_str(), projection_->getBundle()->getName().c_str());
    throw std::domain_error("Wrong space");
  }
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

TaskSpaceGoalBaseState::TaskSpaceGoalBaseState(const ompl::base::SpaceInformationPtr& si, ompl::base::State* baseState, const ompl::multilevel::ProjectionPtr& projection) 
  : ompl::base::GoalSampleableRegion(si), projection_(projection) {

  if(projection_->getBase()->getName() != si->getStateSpace()->getName()) {
    throw std::domain_error("Wrong space");
  }
  baseState_ = projection_->getBase()->allocState();
  tmpBaseState_ = projection_->getBase()->allocState();
  projection_->getBase()->copyState(baseState_, baseState);
}

TaskSpaceGoalBaseState::~TaskSpaceGoalBaseState() {
  projection_->getBase()->freeState(tmpBaseState_);
  projection_->getBase()->freeState(baseState_);
}

void TaskSpaceGoalBaseState::sampleGoal(ompl::base::State *state) const {
  projection_->lift(baseState_, state);
}

unsigned int TaskSpaceGoalBaseState::maxSampleCount() const {
  return 100;
}

double TaskSpaceGoalBaseState::distanceGoal(const ompl::base::State *state) const {
  projection_->project(state, tmpBaseState_);
  return projection_->getBase()->distance(tmpBaseState_, baseState_);
}
