#pragma once

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

dart::dynamics::SkeletonPtr createKukaSkeleton(const std::string& urdf_name =
  "/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka_endeffector.urdf") {
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
