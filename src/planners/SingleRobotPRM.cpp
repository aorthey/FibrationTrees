#include "planners/SingleRobotPRM.hpp"

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>


namespace ompl
{
namespace geometric
{

SingleRobotPRM::SingleRobotPRM(const base::SpaceInformationPtr &si)
  : geometric::PRM(si)
{
}

ompl::base::State* SingleRobotPRM::stateFromVertex(const Vertex& v)
{
    return stateProperty_[v];
}

void SingleRobotPRM::clear() 
{
    PRM::clear();
    g_.clear();
    addedNewSolution_ = false;
}

base::PlannerStatus SingleRobotPRM::solve(const base::PlannerTerminationCondition &ptc) {
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    // Add the valid start states as milestones
    while (const base::State *st = pis_.nextStart())
        startM_.push_back(addMilestone(si_->cloneState(st)));

    if (startM_.empty())
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    // Ensure there is at least one valid goal state
    if (goal->maxSampleCount() > goalM_.size() || goalM_.empty())
    {
        const base::State *st = goalM_.empty() ? pis_.nextGoal(ptc) : pis_.nextGoal();
        if (st != nullptr)
            goalM_.push_back(addMilestone(si_->cloneState(st)));

        if (goalM_.empty())
        {
            OMPL_ERROR("%s: Unable to find any valid goal states", getName().c_str());
            return base::PlannerStatus::INVALID_GOAL;
        }
    }

    unsigned long int nrStartStates = boost::num_vertices(g_);
    OMPL_INFORM("%s: Starting planning with %lu states already in datastructure", getName().c_str(), nrStartStates);

    constructRoadmap(ptc);

    base::PathPtr sol;

    ompl::base::IterationTerminationCondition itc(1);
    checkForSolution(itc, sol);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), boost::num_vertices(g_) - nrStartStates);

    if (sol)
    {
        base::PlannerSolution psol(sol);
        psol.setPlannerName(getName());
        pdef_->addSolutionPath(psol);
    }

    return sol ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;

}

}
}


