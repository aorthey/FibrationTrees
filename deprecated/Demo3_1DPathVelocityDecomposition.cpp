#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/DiscreteMotionValidator.h>

#include <ompl/base/goals/GoalState.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/TimeBasedProjection.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>

#include "Common.hpp"

#include <iostream>
#include <fstream>

using namespace ompl::base;
using namespace ompl::multilevel;

// Disk robot moving along a single line (1dof)

bool isStateValid(const ompl::base::State *state)
{
    // extract the space component of the state and cast it to what we expect
    const auto pos = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0)->values[0];

    // extract the time component of the state and cast it to what we expect
    const auto t = state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;

    // check validity of state defined by pos & t (e.g. check if constraints are satisfied)...
    if(pos > 0.4 && pos < 0.6) {
      if (t > 4 && t < 6) {
        return false;
      }
    }

    // return a value that is always true
    return t >= 0 && pos < std::numeric_limits<double>::infinity();
}

class SpaceTimeMotionValidator : public ompl::base::DiscreteMotionValidator {

public:
    explicit SpaceTimeMotionValidator(const ompl::base::SpaceInformationPtr &si) : DiscreteMotionValidator(si),
      vMax_(si_->getStateSpace().get()->as<ompl::base::SpaceTimeStateSpace>()->getVMax()),
      stateSpace_(si_->getStateSpace().get()) {};

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override
    {
        // assume motion starts in a valid configuration, so s1 is valid
        if (!si_->isValid(s2)) {
            invalid_++;
            return false;
        }

        // check if motion is forward in time and is not exceeding the speed limit
        auto *space = stateSpace_->as<ompl::base::SpaceTimeStateSpace>();
        auto deltaPos = space->distanceSpace(s1, s2);
        auto deltaT = s2->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position -
                      s1->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position;

        if (!(deltaT > 0 && deltaPos / deltaT <= vMax_)) {
            invalid_++;
            return false;
        }
        auto success = DiscreteMotionValidator::checkMotion(s1, s2);
        return success;
    }

    bool checkMotion(const ompl::base::State* s1, const ompl::base::State* s2,
                     std::pair<ompl::base::State *, double>& lastValid) const override
    {
        lastValid.second = 0.0;
        return checkMotion(s1, s2);
    }

private:
    double vMax_; // maximum velocity
    ompl::base::StateSpace *stateSpace_; // the animation state space for distance calculation
};

const float kLowerBound{0};
const float kUpperBound{+1};
const float kVMax = 0.2f;
const float kMaximumTime = 10.0f;

const size_t kMaximumIterations = 1000;

int main()
{
  const size_t kDimension = 1;
  auto space = std::make_shared<RealVectorStateSpace>(1);
  space->setBounds(kLowerBound, kUpperBound);

  auto space_time = std::make_shared<SpaceTimeStateSpace>(space, kVMax);
  space_time->setTimeBounds(0.0, kMaximumTime);
  if(!space_time->getTimeComponent()->isBounded()) {
    OMPL_ERROR("Unbounded");
    exit(0);
  }

  auto factor(std::make_shared<FactoredSpaceInformation>(space_time));
  factor->setStateValidityChecker([](const ompl::base::State *state) { return isStateValid(state); });
  factor->setMotionValidator(std::make_shared<SpaceTimeMotionValidator>(factor));

  auto child(std::make_shared<FactoredSpaceInformation>(space));
  child->setStateValidityChecker([](const ompl::base::State *state) { return true;});

  auto projection = std::make_shared<ompl::multilevel::Projection_TimeBased>(factor->getStateSpace(), child->getStateSpace());
  ReturnOnFalse(factor->addChild(child, projection), 1);

  ScopedState<> goal_state(space_time);
  goal_state[0] = +1;
  goal_state[1] = +kMaximumTime;

  auto goal = std::make_shared<GoalState>(factor);
  goal->setState(goal_state);
  goal->setThreshold(0.01);

  ScopedState<> start(space_time);
  start[0] = 0.0;

  ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoal(goal);

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();

  ompl::base::IterationTerminationCondition itc(kMaximumIterations);
  auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

  PlannerStatus solved = planner->solve(ptc);

  if(pdef->hasApproximateSolution()) {
    auto path = pdef->getSolutionPath();
    path->print(std::cout);
  }
  return 0;
}
