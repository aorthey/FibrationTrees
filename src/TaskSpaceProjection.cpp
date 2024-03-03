#include "TaskSpaceProjection.hpp"

#include <ompl/base/SpaceInformation.h>

#include "dart/math/Random.hpp"
#include "KinematicsSolver.hpp"

TaskSpaceProjection::TaskSpaceProjection(const ompl::base::SpaceInformationPtr& bundle, 
    const ompl::base::SpaceInformationPtr& base, 
    const RobotPtr& robot)
  : ompl::multilevel::Projection(bundle->getStateSpace(), base->getStateSpace()), robot_(robot) {

  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  type_ = ompl::multilevel::PROJECTION_TASK_SPACE;
  if(getBase()->getDimension() != 3) {
    throw "Not3dStateSpace";
  }

  //Create default return states
  defaultBundleReturnState_ = getBundle()->allocState();
  auto nan_config = MakeState(Eigen::VectorXd::Constant(getBundle()->getDimension(), std::numeric_limits<float>::quiet_NaN()));
  robot->EigenToState(nan_config, defaultBundleReturnState_);

  defaultBaseReturnState_ = getBase()->allocState();
  double *base_values = defaultBaseReturnState_->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for (uint k = 0; k < getBase()->getDimension(); k++)
  {
      base_values[k] = std::numeric_limits<double>::quiet_NaN();
  }
}
TaskSpaceProjection::~TaskSpaceProjection() {
  getBundle()->freeState(defaultBundleReturnState_);
  getBase()->freeState(defaultBaseReturnState_);
}

void TaskSpaceProjection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const
{
    auto config = robot_->StateToEigen(xBundle);
    const auto maybe_frame = kinematics_solver_->solve_fk(config);
    if(!maybe_frame.has_value()) {
      getBase()->copyState(xBase, defaultBaseReturnState_);
      return;
    }

    const auto& frame = maybe_frame.value();

    double *values = xBase->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    values[0] = frame[0];
    values[1] = frame[1];
    values[2] = frame[2];
}

void TaskSpaceProjection::lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const
{
    double *xBaseValues = xBase->as<ompl::base::RealVectorStateSpace::StateType>()->values;
    State3d frame;
    frame[0] = xBaseValues[0];
    frame[1] = xBaseValues[1];
    frame[2] = xBaseValues[2];

    auto maybe_result = kinematics_solver_->solve_ik(frame);
    if(!maybe_result.has_value()) {
      getBundle()->copyState(xBundle, defaultBundleReturnState_);
      return;
    }
    auto config = maybe_result.value();

    robot_->EigenToState(config, xBundle);
}
