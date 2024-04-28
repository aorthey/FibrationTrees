#pragma once

#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/base/ProblemDefinition.h>

ompl::base::PlannerTerminationCondition TimeOrSolutionTerminationCondition(const ompl::base::ProblemDefinitionPtr& pdef, const double timeout) {
  return ompl::base::plannerOrTerminationCondition(
         ompl::base::exactSolnPlannerTerminationCondition(pdef),
          ompl::base::plannerAndTerminationCondition(
             ompl::base::timedPlannerTerminationCondition(timeout),
             ompl::base::IterationTerminationCondition(1u)
            )
     );
}
