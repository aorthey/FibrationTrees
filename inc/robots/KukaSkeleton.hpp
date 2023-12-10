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

  //Turn manipulator grey
  // for(const auto& body_node : manipulator->getBodyNodes()) {
  //   auto shapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
  //   for(const auto& node : shapeNodes) {
  //     std::shared_ptr<dart::dynamics::MeshShape> mesh =
  //         std::dynamic_pointer_cast<dart::dynamics::MeshShape>(
  //                 node->getShape());
  //     const Eigen::Vector4d c(0.8,0.8,0.8,1);
  //     if(mesh) {
  //       mesh->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
  //       mesh->setAlphaMode(dart::dynamics::MeshShape::SHAPE_ALPHA);
  //     }
  //     node->getVisualAspect()->setColor(c);
  //   }
  // }
  return manipulator;
}
