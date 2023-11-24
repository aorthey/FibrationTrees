#pragma once

#include <dart/dart.hpp>

dart::dynamics::SkeletonPtr createKukaSkeleton(const std::string& urdf_name =
  "/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf") {
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr manipulator
    = loader.parseSkeleton(urdf_name);

  manipulator->setName("manipulator");
  manipulator->setMobile(false);
  manipulator->setSelfCollisionCheck(false);
  manipulator->setAdjacentBodyCheck(true);
  std::vector<size_t> indices = {0,1,2,3,4,5};
  Eigen::VectorXd zero = Eigen::VectorXd::Zero(6);
  manipulator->setPositionLowerLimits(indices, zero);
  manipulator->setPositionUpperLimits(indices, zero);
  return manipulator;
}
