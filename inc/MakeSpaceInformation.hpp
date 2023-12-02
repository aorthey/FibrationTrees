#pragma once

#include <dart/dart.hpp>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "CollisionChecker.hpp"
#include "TaskSpace.hpp"

////////////////////////////////////////////////////////////////////////////////
// SpaceInformation Makers
////////////////////////////////////////////////////////////////////////////////
ompl::multilevel::FactoredSpaceInformationPtr MakeTaskSpaceInformation(
  const dart::dynamics::SkeletonPtr& manipulator,
  const dart::simulation::WorldPtr& world,
  const KinematicsSolverPtr& kinematics_solver,
  const CollisionCheckerPtr& collision_checker) {
  auto numDofs = manipulator->getNumDofs();
  ompl::base::StateSpacePtr space(new TaskSpace(numDofs, kinematics_solver));
  ompl::base::RealVectorBounds bounds(numDofs);
  auto lb = manipulator->getPositionLowerLimits();
  auto ub = manipulator->getPositionUpperLimits();
  for(size_t k =0; k< numDofs; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));
  factor->setStateValidityChecker(std::make_shared<DartWorldCollisionChecker>(factor, world, manipulator, collision_checker));
  factor->setStateValidityCheckingResolution(0.001);
  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<TaskSpaceMotionValidator>(factor, kinematics_solver);
  factor->setMotionValidator(motion_validator);

  return factor;
}

ompl::multilevel::FactoredSpaceInformationPtr Make3DPointSpaceInformation(
  const dart::dynamics::SkeletonPtr& point,
  const dart::simulation::WorldPtr& world,
  const CollisionCheckerPtr& collision_checker) {
  auto numDofsPoint = 3;
  ompl::base::StateSpacePtr spaceR3(new ompl::base::RealVectorStateSpace(numDofsPoint));
  ompl::base::RealVectorBounds boundsWorkspace(numDofsPoint);
  boundsWorkspace.setLow(0, +0.38);
  boundsWorkspace.setHigh(0, +0.42);
  boundsWorkspace.setLow(1, -2);
  boundsWorkspace.setHigh(1, +2);
  boundsWorkspace.setLow(2, 0.0);
  boundsWorkspace.setHigh(2, 2.0);
  spaceR3->as<ompl::base::RealVectorStateSpace>()->setBounds(boundsWorkspace);

  auto child(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(spaceR3));
  child->setStateValidityChecker(std::make_shared<DartTransformCollisionChecker>(child, world, point, collision_checker));
  child->setStateValidityCheckingResolution(0.001);
  return child;
}

ompl::multilevel::FactoredSpaceInformationPtr MakeMultiRobotSpaceInformation(
  const dart::simulation::WorldPtr& world,
  const std::vector<dart::dynamics::SkeletonPtr>& manipulators,
  const std::vector<KinematicsSolverPtr>& kinematics_solvers,
  const CollisionCheckerPtr& collision_checkers) {

  auto numDofs = std::accumulate(manipulators.begin(), manipulators.end(), 0u, 
      [](size_t dofs, const auto& manipulator){
        return dofs + manipulator->getNumDofs();
      });

  ompl::base::StateSpacePtr space(new ompl::base::RealVectorStateSpace(numDofs));
  ompl::base::RealVectorBounds bounds(numDofs);

  auto dofs = 0u;
  for(const auto& manipulator : manipulators) {
    auto lb = manipulator->getPositionLowerLimits();
    auto ub = manipulator->getPositionUpperLimits();
    for(size_t k = 0; k < lb.size(); k++) {
      bounds.setLow(k + dofs, lb[k]);
      bounds.setHigh(k + dofs, ub[k]);
    }
    dofs += lb.size();
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));

  return factor;
}

