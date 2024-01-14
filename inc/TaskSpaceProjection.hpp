#pragma once

#include <dart/dart.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/Projection.h>

#include "KinematicsSolver.hpp"

OMPL_CLASS_FORWARD(ProjectionJointSpaceToR3);

class ProjectionJointSpaceToR3 : public ompl::multilevel::Projection
{
 public:
  ProjectionJointSpaceToR3(const ompl::base::SpaceInformationPtr& bundle, const ompl::base::SpaceInformationPtr& base, const KinematicsSolverPtr& kinematics_solver);
  ProjectionJointSpaceToR3(const ompl::base::StateSpacePtr& bundle, const ompl::base::StateSpacePtr& base, const KinematicsSolverPtr& kinematics_solver);
  ~ProjectionJointSpaceToR3();

  void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const;
  void lift(const ompl::base::State *xBase, ompl::base::State *xBundle) const;
 private:
  KinematicsSolverPtr kinematics_solver_;
  ompl::base::State* defaultBaseReturnState_;
  ompl::base::State* defaultBundleReturnState_;
};
