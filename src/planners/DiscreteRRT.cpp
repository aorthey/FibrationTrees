#include "planners/DiscreteRRT.hpp"
#include <ompl/util/Exception.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>

#include <ompl/base/GoalTypes.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/ProblemDefinitionHelper.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <ompl/base/ScopedState.h>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <cmath>

#include "planners/SingleRobotPRM.hpp"

namespace ompl
{
    namespace magic
    {
        const size_t kMaxNearestNeighbors = 10;
    }
}
namespace ompl
{
namespace geometric
{

// Compute angle at q_near between q_rand and q_candidate
double computeAngle(const base::State* q_near, const base::State* q_rand,
                    const base::State* q_candidate, const base::SpaceInformationPtr& si) {

    // Use the actual metric of the space (L∞ in your case)
    double d_to_rand = si->distance(q_near, q_rand);
    if (d_to_rand < 1e-9) {
        //OMPL_WARN("Distance from near to rand is very small (%f). Returning 0.", d_to_rand);
        return 0.0;
    }

    double d_to_cand = si->distance(q_near, q_candidate);
    if (d_to_cand < 1e-9) {
        return M_PI;  // degenerate case: q_candidate == near
    }

    double d_cand_to_rand = si->distance(q_candidate, q_rand);

    // === Option 1: Simple and commonly used in high-dim / non-Euclidean dRRT variants ===
    // Just return how aligned the directions are by comparing triangle inequality
    // (closer to 0 = better aligned, closer to π = opposite)
    double alignment = (d_to_cand + d_to_rand - d_cand_to_rand) / (2.0 * std::min(d_to_cand, d_to_rand));
    alignment = std::max(0.0, std::min(1.0, alignment));   // clamp to [0,1]

    // Convert to pseudo-angle in [0, π]
    return M_PI * (1.0 - alignment);
}

DiscreteRRT::DiscreteRRT(const base::SpaceInformationPtr &si)
  : base::Planner(si, "DiscreteRRT")
{
    specs_.approximateSolutions = false;
    specs_.optimizingPaths = false;
    specs_.multithreaded = false;
    specs_.directed = true;

    Planner::declareParam<double>("roadmap_build_time", this, &DiscreteRRT::setRoadmapBuildTime, &DiscreteRRT::getRoadmapBuildTime, "0.:1.:1000.");

    auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);
    q_best_substates_ = factor->allocChildStates();
    q_rand_substates_ = factor->allocChildStates();
}

DiscreteRRT::~DiscreteRRT()
{
    auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);
    factor->freeChildStates(q_rand_substates_);
    factor->freeChildStates(q_best_substates_);
    freeMemory();
}

void DiscreteRRT::setRoadmapBuildTime(double t) 
{ 
    roadmapBuildTime_ = t;
}

double DiscreteRRT::getRoadmapBuildTime() const
{ 
    return roadmapBuildTime_;
}

void DiscreteRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (nn_)
    {
        nn_->clear();
    }
    lastGoalMotion_ = nullptr;

    for (auto &prm : singleRobotPRMs_)
    {
        prm->clear();
    }
    // for (auto &graph : singleRobotGraphs_)
    // {
    //     graph.clear();
    // }

    // for(auto& problem_definition : singleRobotPdefs_) {
    //     problem_definition->clearSolutionPaths();
    // }
}

void DiscreteRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<CompositeMotion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void DiscreteRRT::setup()
{
    Planner::setup();

    auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);

    if (!factor->hasChildren())
    {
      throw std::runtime_error("Factor has no children.");
    }

    auto children = factor->getChildren();
    if (children.size() == 0)
    {
      throw std::runtime_error("Factor has no children.");
    }

    num_robots_ = children.size();
    OMPL_INFORM("Planning with %d robots.", num_robots_);
    singleRobotPRMs_.resize(num_robots_);
    singleRobotGraphs_.resize(num_robots_);
    singleRobotPdefs_.resize(num_robots_);

    for (unsigned int i = 0; i < num_robots_; ++i)
    {
        auto subSI = children.at(i);
        subSI->setup();
        singleRobotPRMs_.at(i) = std::make_shared<SingleRobotPRM>(subSI);
        singleRobotPRMs_.at(i)->setName("SingleRobotPRM" + std::to_string(i));
        singleRobotPRMs_.at(i)->setMaxNearestNeighbors(ompl::magic::kMaxNearestNeighbors);
    }
    for (auto &prm : singleRobotPRMs_)
    {
        prm->setup();
    }

    if (!nn_)
    {
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<CompositeMotion *>(this));
    }
    nn_->setDistanceFunction([this](const CompositeMotion *a, const CompositeMotion *b) { return si_->distance(a->state, b->state); });

}

void DiscreteRRT::setProblemDefinition(const base::ProblemDefinitionPtr &pdef) {
  Planner::setProblemDefinition(pdef);

  const auto root = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);

  OMPL_INFORM("Set problem definition for factor %s", root->getName().c_str());

  auto problem_definitions_per_factor = computeProblemDefinitions(root, pdef, 0.1); //TODO: verify the real value for goal threshold

  if(root->getTotalNumChildren() != problem_definitions_per_factor.size() - 1) {
      OMPL_ERROR("Found different size of problem definitions: %d vs %d", 
          root->getTotalNumChildren(), problem_definitions_per_factor.size());

      throw std::runtime_error("Found different size of problem definitions.");
  }

  for(const auto& child : root->getChildren())
  {
      auto name = child->getName();
      auto pdef_iterator = problem_definitions_per_factor.find(name);
      if(pdef_iterator == problem_definitions_per_factor.end()) {
          throw std::runtime_error("Cannot find problem definition for factor " + name);
      }
      singleRobotPdefs_.push_back(pdef_iterator->second);
  }
}

void DiscreteRRT::PrintMotion(const CompositeMotion *m) const
{
    auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);
    factor->printState(m->state);
    for(unsigned int i = 0; i < m->vertices.size(); ++i)
    {
        OMPL_DEBUG("Vertex %d: %d", i, m->vertices.at(i));
    }
}

DiscreteRRT::CompositeMotion* DiscreteRRT::oracle(const DiscreteRRT::CompositeMotion* nearest_motion, const base::State* q_rand) const
{
    auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(si_);
    auto children = factor->getChildren();

    factor->project(q_rand, q_rand_substates_);

    if(q_rand_substates_.size() != num_robots_)
    {
        throw std::runtime_error("Invalid num substates: " + q_rand_substates_.size());
    }

    auto *best_motion = new CompositeMotion(si_);
    best_motion->vertices.resize(num_robots_);
    std::fill(best_motion->vertices.begin(), best_motion->vertices.end(), 0);

    for (unsigned int index = 0; index < num_robots_; ++index)
    {
        const auto& child = children.at(index);
        const auto& name = child->getName();
        const auto& graph = singleRobotGraphs_.at(index);
        const auto& prm = singleRobotPRMs_.at(index);

        auto v = nearest_motion->vertices.at(index);
        // auto num_neighbors = boost::degree(v, graph);
        // auto num_vertices = boost::num_vertices(graph);

        const auto& q_near_local = prm->stateFromVertex(v);
        const auto& q_rand_local = q_rand_substates_.at(name);

        auto adj = boost::adjacent_vertices(v, graph);

        double best_angle = std::numeric_limits<double>::infinity();
        child->copyState(q_best_substates_.at(name), q_near_local);

        for (auto vi = adj.first; vi != adj.second; ++vi)
        {
            const Vertex& neighbor = *vi;
            if(neighbor == v) {
              continue;
            }
            const auto q_candidate = prm->stateFromVertex(neighbor);
            auto angle = computeAngle(q_near_local, q_rand_local, q_candidate, child);
            if(angle < best_angle) {
                best_angle = angle;
                child->copyState(q_best_substates_.at(name), q_candidate);
                best_motion->vertices.at(index) = neighbor;
            }
        }
        //OMPL_DEBUG("Best angle for %d/%d (%s): %f", index, num_robots_, name.c_str(), best_angle);
    }
    factor->lift(q_best_substates_, best_motion->state);

    return best_motion;
}

bool DiscreteRRT::buildSingleRobotRoadmaps(const base::PlannerTerminationCondition &ptc)
{
    for (unsigned int i = 0; i < num_robots_; ++i)
    {
        auto prm = singleRobotPRMs_.at(i);
        prm->setProblemDefinition(singleRobotPdefs_.at(i));
        OMPL_INFORM("Plan roadmap for single-robot %d/%d (timeout %.2f seconds).", i + 1, num_robots_, roadmapBuildTime_);

        auto singleRobotPtc = base::plannerOrTerminationCondition(
                            ptc, base::timedPlannerTerminationCondition(roadmapBuildTime_));

        auto status = prm->solve(singleRobotPtc);

        if(!status) {
            OMPL_ERROR("Could not find a single-robot solution on level %d/%d", i + 1, num_robots_);
            return false;
        }

        OMPL_INFORM("Found single-robot solution on level %d/%d (Found %d vertices).", i + 1, num_robots_, prm->milestoneCount());

        if(ptc) {
            return false;
        }
    }
    return true;
}

base::PlannerStatus DiscreteRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    OMPL_INFORM("Using roadmapBuildTime of %.2f seconds", getRoadmapBuildTime());

    if(!buildSingleRobotRoadmaps(ptc)) 
    {
        return base::PlannerStatus::TIMEOUT;
    }

    checkValidity();

    ////////////////////////////////////////////////////////////////////////////////
    /// Get start state
    ////////////////////////////////////////////////////////////////////////////////
    while (const base::State *st = pis_.nextStart())
    {
        std::vector<Vertex> vertices; 
        for (unsigned int i = 0; i < num_robots_; ++i)
        {
            vertices.push_back(0);
        }
        auto *motion = new CompositeMotion(si_);
        motion->vertices = vertices;
        si_->copyState(motion->state, st);
        nn_->add(motion);
        break;
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }
    ////////////////////////////////////////////////////////////////////////////////
    /// Get goal state
    ////////////////////////////////////////////////////////////////////////////////
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_motion = new CompositeMotion(si_);
    while (const base::State *st = pis_.nextGoal(ptc))
    {
        if(st != nullptr) 
        {
            si_->copyState(goal_motion->state, st);
            break;
        }
    }

    if(goal_motion->state == nullptr) {
        OMPL_ERROR("%s: goal state is null", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    ////////////////////////////////////////////////////////////////////////////////
    /// Init
    ////////////////////////////////////////////////////////////////////////////////
    if (!sampler_)
    {
        sampler_ = si_->allocStateSampler();
    }

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    CompositeMotion *solution = nullptr;
    auto *random_motion = new CompositeMotion(si_);

    unsigned int iterations = 1;
    while (!ptc)
    {
        //////////////////////////////////////////////////////////////////////////////// 
        // Step (0) Compute N
        //////////////////////////////////////////////////////////////////////////////// 
        const unsigned int N = pow(2.0, iterations);
        const unsigned int K = iterations;
        
        //////////////////////////////////////////////////////////////////////////////// 
        // Step (1) Sample random state
        //////////////////////////////////////////////////////////////////////////////// 
        // if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample())
        //     goal_s->sampleGoal(random_motion->state);
        // else
        //     sampler_->sampleUniform(random_motion->state);
        unsigned int subiterations = 0;
        while (!ptc && subiterations < N)
        {
            sampler_->sampleUniform(random_motion->state);

            //////////////////////////////////////////////////////////////////////////////// 
            // Step (2) Find closest state in tree
            //////////////////////////////////////////////////////////////////////////////// 
            CompositeMotion *nearest_motion = nn_->nearest(random_motion);

            //////////////////////////////////////////////////////////////////////////////// 
            // Step (3) Find vertex q_new in graph, closest to qrand according to
            // oracle
            //////////////////////////////////////////////////////////////////////////////// 
            random_motion = oracle(nearest_motion, random_motion->state);

            //////////////////////////////////////////////////////////////////////////////// 
            // Step (4) Add vertex and edge to tree if feasible
            //////////////////////////////////////////////////////////////////////////////// 

            if (si_->checkMotion(nearest_motion->state, random_motion->state))
            {
                auto *motion = new CompositeMotion(si_);
                si_->copyState(motion->state, random_motion->state);
                motion->parent = nearest_motion;
                motion->vertices = random_motion->vertices;
                if(motion->vertices.size() == 0) {
                  throw std::runtime_error("Motion has no vertices.");
                }
                nn_->add(motion);

                double dist = 0.0;
                bool sat = goal->isSatisfied(motion->state, &dist);
                if (sat)
                {
                    solution = motion;
                    break;
                }
            }
            subiterations++;
        }
        //////////////////////////////////////////////////////////////////////////////// 
        // Step (5) Connect to target
        //////////////////////////////////////////////////////////////////////////////// 

        std::vector<CompositeMotion *> neighbors;
        nn_->nearestK(goal_motion, K, neighbors);
        bool foundSolution = false;
        for(const auto& neighbor : neighbors) 
        {
            //TODO: Implement local connector VdBerg et al.
            if (si_->checkMotion(neighbor->state, goal_motion->state))
            {
                OMPL_INFORM("%s: Found direct solution from tree to goal.", getName().c_str());
                auto *motion = new CompositeMotion(si_);
                si_->copyState(motion->state, goal_motion->state);
                motion->parent = neighbor;
                //motion->vertices = neighbor_motion->vertices;
                nn_->add(motion);
                solution = motion;
                foundSolution = true;
                break;
            }
            if(ptc) {
                break;
            }
        }
        if(foundSolution)
        {
          break;
        }

        iterations++;
    }

    bool solved = false;
    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<CompositeMotion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        pdef_->addSolutionPath(path);
        solved = true;
    }

    if (random_motion->state != nullptr)
        si_->freeState(random_motion->state);
    delete random_motion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, false};
}
}
}
