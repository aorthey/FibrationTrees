#pragma once

#include <dart/dart.hpp>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "CollisionChecker.hpp"
#include "TaskSpace.hpp"
#include "TranslationTaskSpaceMotionValidator.hpp"

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
  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<TranslationTaskSpaceMotionValidator>(factor, kinematics_solver);
  factor->setMotionValidator(motion_validator);

  return factor;
}

ompl::multilevel::FactoredSpaceInformationPtr Make3DPointSpaceInformation(
  const dart::dynamics::SkeletonPtr& point,
  const dart::simulation::WorldPtr& world,
  const CollisionCheckerPtr& collision_checker,
  const std::pair<Eigen::Vector3d, Eigen::Vector3d>& limits) {
  auto numDofsPoint = 3;
  ompl::base::StateSpacePtr spaceR3(new ompl::base::RealVectorStateSpace(numDofsPoint));
  ompl::base::RealVectorBounds boundsWorkspace(numDofsPoint);

  const auto lb = limits.first;
  const auto ub = limits.second;
  for(size_t k = 0; k < 3; k++) {
    boundsWorkspace.setLow(k, lb[k]);
    boundsWorkspace.setHigh(k, ub[k]);
  }
  spaceR3->as<ompl::base::RealVectorStateSpace>()->setBounds(boundsWorkspace);

  auto child(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(spaceR3));
  child->setStateValidityChecker(std::make_shared<DartWorldCollisionChecker>(child, world, point, collision_checker));
  child->setStateValidityCheckingResolution(0.001);
  return child;
}

ompl::multilevel::FactoredSpaceInformationPtr MakeMultiRobotSpaceInformation(
  const std::vector<ompl::base::StateSpacePtr>& task_spaces) {
  auto space = std::make_shared<ompl::base::CompoundStateSpace>(task_spaces, std::vector<double>(task_spaces.size(), 1.0f));

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));
  return factor;
}


