#ifndef OMPL_GEOMETRIC_PLANNERS_SINGLEROBOTPRM__
#define OMPL_GEOMETRIC_PLANNERS_SINGLEROBOTPRM__

#include <ompl/base/Planner.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <vector>
#include <memory>
#include <unordered_map>

namespace ompl
{
namespace geometric
{

class SingleRobotPRM : public geometric::PRM
{
public:
    SingleRobotPRM(const base::SpaceInformationPtr &si);

    ~SingleRobotPRM() override = default;

    base::PlannerStatus solve(const base::PlannerTerminationCondition &ptc);

    void clear() override;

    ompl::base::State* stateFromVertex(const Vertex& v);
};

}  // namespace geometric
}  // namespace ompl

#endif

