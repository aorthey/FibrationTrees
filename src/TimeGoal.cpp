#include "TimeGoal.hpp"
#include "Common.hpp"

TimeGoal::TimeGoal(const RobotPtr &robot, float vMax, const ompl::base::State *start, const ompl::base::State *goal) :
  GoalState(robot->GetSpaceInformation()), robot_(robot), vMax_(vMax){

  auto s1 = robot_->StateToEigen(start);
  auto s2 = robot_->StateToEigen(goal);
  if(!IsReachableInTime(s1, s2, vMax)) {
    OMPL_ERROR("TimeGoal Not Reachable");
    throw "CannotReachInTime";
  }

  minTime_ = GetMinimumReachableTime(s1, s2, vMax);
  maxTime_ = s2.time;

  OMPL_INFORM("Create TimeGoal [%f, %f]", minTime_, maxTime_);

  setState(goal);
}

void TimeGoal::sampleGoal(ompl::base::State *state) const {
  si_->copyState(state, state_);
  auto time = dart::math::Random::uniform(minTime_, maxTime_);
  robot_->TimeToState(time, state);
}

unsigned int TimeGoal::maxSampleCount() const {
  return 100;
}

double TimeGoal::distanceGoal(const ompl::base::State *state) const {
  auto input = robot_->StateToEigen(state);
  auto desired = robot_->StateToEigen(state_);
  if(input.time < minTime_ || input.time > maxTime_) {
    return Inf;
  }
  float deltaSpace = Distance(input, desired);
  return deltaSpace;
}
