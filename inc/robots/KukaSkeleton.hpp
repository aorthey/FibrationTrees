#pragma once

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

dart::dynamics::SkeletonPtr createKukaSkeleton(const std::string& urdf_name =
  // "/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf") {
  "/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka_endeffector.urdf") {
  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FIXED;
  loader.setOptions(options);

  dart::dynamics::SkeletonPtr manipulator
    = loader.parseSkeleton(urdf_name);

  manipulator->setName("manipulator");
  manipulator->setMobile(false);
  manipulator->setSelfCollisionCheck(true);
  manipulator->setAdjacentBodyCheck(true);
  return manipulator;
}
