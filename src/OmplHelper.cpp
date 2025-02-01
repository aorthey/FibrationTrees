#include "OmplHelper.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

bool SampleValidLift(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::SpaceInformationPtr& si, 
    const size_t max_iterations, const ompl::base::State *xBase, ompl::base::State *xBundle) {
  size_t iterations=0;
  while(iterations++ < max_iterations) {
    projection->lift(xBase, xBundle);
    if(si->getStateValidityChecker()->isValid(xBundle)) {
      return true;
    }
  }
  return false;
}

std::optional<ompl::base::State*> ComputeValidIKState(const ompl::base::SpaceInformationPtr& si, 
    const ompl::multilevel::ProjectionPtr& projection, const State3d& point) {

  ompl::base::State *task_state = projection->getBase()->allocState();
  double *angles = task_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  angles[0] = point[0];
  angles[1] = point[1];
  angles[2] = point[2];

  ompl::base::State *state = si->allocState();
  if(!SampleValidLift(projection, si, kMaxResampleIteration, task_state, state)) {
    return std::nullopt;
  }

  return state;
}

std::optional<ompl::base::State*> ComputeValidTotalState(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const std::unordered_map<std::string, ompl::base::State*>& leaf_node_states, size_t max_resample_iterations) {
  ompl::base::State *state = factor->allocState();

  OMPL_INFORM("Trying to find valid total state for %d iterations.", max_resample_iterations);
  size_t samples = 0;
  while(samples++ < max_resample_iterations) {
    factor->liftLeafStates(leaf_node_states, state);
    if(factor->isValid(state)) {
      return state;
    }
  }

  OMPL_ERROR("Invalid total state after %d iterations.", samples - 1);
  factor->freeState(state);
  return std::nullopt;
}

ompl::base::State* AllocStateFromEigen(const RobotPtr& robot, const StateXd& v) {
  auto state = robot->GetSpaceInformation()->allocState();
  robot->EigenToState(v, state);
  return state;
}

ompl::base::PlannerTerminationCondition TimeOrSolutionPtc(const ompl::base::ProblemDefinitionPtr &pdef, double timeout) {
  auto solution_ptc = ompl::base::exactSolnPlannerTerminationCondition(pdef);
  auto timed_ptc = ompl::base::timedPlannerTerminationCondition(timeout);
  return ompl::base::plannerOrTerminationCondition(solution_ptc, timed_ptc);
}

std::vector<ompl::base::State*> MakeInterpolatedPathSegment(const ompl::base::SpaceInformation* si, const ompl::base::State *s1, const ompl::base::State *s2)  {
  std::vector<ompl::base::State*> result;
  const unsigned int count = si->getStateSpace()->validSegmentCount(s1, s2);
  si->getMotionStates(s1, s2, result, count, true, true);
  return result;
}

std::vector<ompl::base::State*> MakeInterpolatedPathSegment(const ompl::base::SpaceInformationPtr &si, const ompl::base::State *s1, const ompl::base::State *s2)  {
  return MakeInterpolatedPathSegment(si.get(), s1, s2);
}
