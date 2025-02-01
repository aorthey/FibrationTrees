#pragma once

#include <dart/dart.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>

#include "KinematicsSolver.hpp"
#include "robots/Robot.hpp"

OMPL_CLASS_FORWARD(TaskSpaceProjection);

class TaskSpaceProjection : public ompl::multilevel::Projection
{
 public:
  TaskSpaceProjection(const ompl::base::SpaceInformationPtr& bundle, const ompl::base::SpaceInformationPtr& base, const RobotPtr& robot);
  ~TaskSpaceProjection();

  void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const;
  void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const;
 private:
  RobotPtr robot_;
  KinematicsSolverPtr kinematics_solver_;
  ompl::base::State* defaultBaseReturnState_;
  ompl::base::State* defaultBundleReturnState_;
};
