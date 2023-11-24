#include "TaskSpaceProjection.hpp"

#include "dart/math/Random.hpp"
#include "KinematicsSolver.hpp"

ProjectionJointSpaceToR3::ProjectionJointSpaceToR3(const ompl::base::StateSpacePtr& bundle, 
    const ompl::base::StateSpacePtr& base, 
    const KinematicsSolverPtr& kinematics_solver)
  : ompl::multilevel::Projection(bundle, base), kinematics_solver_(kinematics_solver)
{
    type_ = ompl::multilevel::PROJECTION_TASK_SPACE;
    if(getBase()->getDimension() != 3) {
      throw "Not3dStateSpace";
    }

    defaultBundleReturnState_ = getBundle()->allocState();
    double *values = defaultBundleReturnState_->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (uint k = 0; k < getBundle()->getDimension(); k++)
    {
        values[k] = std::numeric_limits<double>::quiet_NaN();
    }
}
ProjectionJointSpaceToR3::~ProjectionJointSpaceToR3() {
  getBundle()->freeState(defaultBundleReturnState_);
}

void ProjectionJointSpaceToR3::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    double *xBundleValues = xBundle->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    Eigen::VectorXd config = Eigen::VectorXd::Zero(getBundle()->getDimension());
    for(size_t dim = 0; dim < getBundle()->getDimension(); dim++)
    {
      config[dim] = xBundleValues[dim];
    }
    const auto maybe_frame = kinematics_solver_->solve_fk(config);
    if(!maybe_frame.has_value()) {
      getBase()->copyState(xBase, defaultBaseReturnState_);
      return;
    }

    const auto& frame = maybe_frame.value();

    double *angles = xBase->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    angles[0] = frame[0];
    angles[1] = frame[1];
    angles[2] = frame[2];
}

void ProjectionJointSpaceToR3::lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
{
    double *xBaseValues = xBase->as<ompl::base::RealVectorStateSpace::StateType>()->values;

    Eigen::Vector3d frame;
    frame[0] = xBaseValues[0];
    frame[1] = xBaseValues[1];
    frame[2] = xBaseValues[2];

    auto maybe_result = kinematics_solver_->solve_ik(frame);
    if(!maybe_result.has_value()) {
      getBundle()->copyState(xBundle, defaultBundleReturnState_);
      return;
    }
    auto result = maybe_result.value();

    double *angles = xBundle->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    for (uint k = 0; k < getBundle()->getDimension(); k++)
    {
        angles[k] = result[k];
    }

    auto tmpState = getBase()->allocState();
    project(xBundle, tmpState);
    OMPL_WARN("Base -> Bundle -> Base");
    getBase()->printState(xBase);
    getBundle()->printState(xBundle);
    getBase()->printState(tmpState);
    getBase()->freeState(tmpState);
}
