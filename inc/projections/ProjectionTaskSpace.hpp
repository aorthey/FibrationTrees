#pragma once

#include <dart/dart.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>

#include "KinematicsSolver.hpp"
#include "robots/Robot.hpp"

OMPL_CLASS_FORWARD(ProjectionTaskSpace);

class ProjectionTaskSpace : public ompl::multilevel::Projection
{
 public:
  ProjectionTaskSpace(const ompl::base::SpaceInformationPtr& bundle, const ompl::base::SpaceInformationPtr& base, const RobotPtr& robot);
  ~ProjectionTaskSpace();

  void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const;
  void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const;
 private:
  RobotPtr robot_;
  KinematicsSolverPtr kinematics_solver_;
  ompl::base::State* defaultBaseReturnState_;
  ompl::base::State* defaultBundleReturnState_;
};
