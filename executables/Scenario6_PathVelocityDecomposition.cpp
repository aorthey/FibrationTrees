#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>

#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <iostream>
#include <fstream>

using namespace ompl::base;
using namespace ompl::multilevel;

const float kLowerBound{-1};
const float kUpperBound{+1};
const size_t kMaximumIterations = 50;
int main()
{
  const size_t kDimension = 2;
  auto space = std::make_shared<RealVectorStateSpace>(2 * kDimension);
  space->setBounds(kLowerBound, kUpperBound);

  auto factor(std::make_shared<FactoredSpaceInformation>(space));

  ScopedState<> start(space);
  start[0] = -1;
  start[1] = -1;
  start[2] = 0.0;
  start[3] = 0.0;
  ScopedState<> goal(space);
  goal[0] = +1;
  goal[1] = +1;
  goal[2] = 0.0;
  goal[3] = 0.0;

  ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
  pdef->setStartAndGoalStates(start, goal);

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();

  ompl::base::IterationTerminationCondition itc(kMaximumIterations);
  auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

  PlannerStatus solved = planner->solve(ptc);

  if(solved == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION) {
    auto path = pdef->getSolutionPath();
    path->print(std::cout);

    // auto geom_path = path->as<ompl::geometric::PathGeometric>();
    // auto path_simplifier = std::make_shared<ompl::geometric::PathSimplifier>(factor);
    // path_simplifier->simplifyMax(*geom_path);
    // path_simplifier->smoothBSpline(*geom_path);
  }
  return 0;
}
