#include "robots/KukaRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "spaces/TaskSpace.hpp"
#include "KinematicsSolver.hpp"
#include "FilePath.hpp"

dart::dynamics::SkeletonPtr KukaRobot::MakeSkeleton(const YAML::Node& /*node*/) {
  const auto urdf_name = GetDataFolder() + "robots/kuka_lwr/kuka_endeffector.urdf";

  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FIXED;
  loader.setOptions(options);

  dart::dynamics::SkeletonPtr manipulator
    = loader.parseSkeleton(urdf_name);

  static int count = 0;
  manipulator->setName("manipulator_"+std::to_string(count++));
  manipulator->setMobile(false);
  manipulator->setSelfCollisionCheck(true);
  manipulator->setAdjacentBodyCheck(true);

  //Disable friction. This was causing an assert error in ContactConstraint.cpp
  //in dartsim
  for(const auto& body_node : manipulator->getBodyNodes()) {
    auto nodes = body_node->getShapeNodesWith<dart::dynamics::DynamicsAspect>();
    for(const auto& node : nodes) {
      node->getDynamicsAspect()->setFrictionCoeff(0.0);
      node->getDynamicsAspect()->setPrimaryFrictionCoeff(0.0);
      node->getDynamicsAspect()->setSecondaryFrictionCoeff(0.0);
    }
  }
  return manipulator;
}

ompl::multilevel::FactoredSpaceInformationPtr KukaRobot::MakeSpaceInformation(const RobotPtr& robot) {
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  auto numDofs = robot->GetSkeleton()->getNumDofs();
  auto space = std::make_shared<ompl::base::RealVectorStateSpace>(numDofs);
  ompl::base::RealVectorBounds bounds(numDofs);
  auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k =0; k< numDofs; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}
