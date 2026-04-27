#ifndef OMPL_GEOMETRIC_PLANNERS_MULTIROBOT_DISCRETE_PLANNER_
#define OMPL_GEOMETRIC_PLANNERS_MULTIROBOT_DISCRETE_PLANNER_

#include <ompl/base/Planner.h>
#include <vector>
#include <memory>

#include "planners/SingleRobotPRM.hpp"

namespace ompl
{
namespace geometric
{

/** Simple multi-robot planner:
 *  1. Builds one PRM roadmap per robot (on each subspace).
 *  2. Performs basic path coordination on the discrete product graph.
 */
using Vertex = PRM::Vertex;

class DiscreteRRT : public base::Planner
{
public:
    DiscreteRRT(const base::SpaceInformationPtr &si);

    ~DiscreteRRT() override;

    void clear() override;

    void setup() override;

    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc) override;
    void setProblemDefinition(const base::ProblemDefinitionPtr &pdef) override;

    void setRoadmapBuildTime(double t);
    double getRoadmapBuildTime() const;

protected:
    void freeMemory();

    // Build individual PRM roadmaps (one per robot)
    bool buildSingleRobotRoadmaps(const base::PlannerTerminationCondition &ptc);

    bool coordinateRoadmaps(const base::PlannerTerminationCondition &ptc, std::vector<base::State *> &solutionStates) const;

    std::vector<ompl::base::ProblemDefinitionPtr> computeChildrenProblemDefinition(const ompl::base::ProblemDefinitionPtr& pdef);

    class CompositeMotion
    {
    public:
        CompositeMotion() = delete;
        CompositeMotion(const base::SpaceInformationPtr &si) : state(si->allocState())
        {
        }

        CompositeMotion(const CompositeMotion& other,
                        const base::SpaceInformationPtr &si)
            : vertices(other.vertices),
              parent(other.parent)
        {
            if (other.state)
            {
                state = si->allocState();
                si->copyState(state, other.state);
            }
        }
        ~CompositeMotion() = default;

        base::State *state{nullptr};
        std::vector<Vertex> vertices; //substate vertices
        CompositeMotion *parent{nullptr};
    };

    void PrintMotion(const CompositeMotion *m) const;

private:

    double roadmapBuildTime_ = 1.0;          // seconds per roadmap

    size_t num_robots_{0};

    std::vector<std::shared_ptr<SingleRobotPRM>> singleRobotPRMs_;
    std::vector<ompl::base::ProblemDefinitionPtr> singleRobotPdefs_;
    std::vector<SingleRobotPRM::Graph> singleRobotGraphs_;

    // Simple nearest-neighbor for coordination (you can improve this later)

    std::shared_ptr<NearestNeighbors<CompositeMotion *>> nn_;
    base::StateSamplerPtr sampler_;
    double goalBias_{.05};
    double maxDistance_{0.};
    CompositeMotion *lastGoalMotion_{nullptr};
    RNG rng_;

    CompositeMotion* oracle(const CompositeMotion* nearest_motion, const base::State* q_rand) const;

    std::unordered_map<std::string, base::State*> q_best_substates_;
    std::unordered_map<std::string, base::State*> q_rand_substates_;
};


//TODO
//[ ] Collision checking on composite space should be between robots only
//[ ] Oracle
//[ ] Get vertex for a state and its neighbors

}  // namespace geometric
}  // namespace ompl

#endif
