#pragma once

#include "dart/dart.hpp"
#include "dart/gui/osg/osg.hpp"
#include "dart/utils/urdf/urdf.hpp"
#include "dart/math/Random.hpp"

const Eigen::Vector3d kPathColor = Eigen::Vector3d(0.3, 0.8, 0.3);
const float kPathSphereSize = 0.01;
const float kPathLineWidth = 5.0;

void PrintSkeletonInfo(const dart::dynamics::SkeletonPtr& skeleton);
Eigen::VectorXd GetRandomPosition(const dart::dynamics::SkeletonPtr& skeleton);
dart::dynamics::SimpleFramePtr createSphereFrame(const Eigen::Vector3d& position, const float radius = kPathSphereSize, const Eigen::Vector3d& color = kPathColor);
dart::dynamics::SimpleFramePtr createLineSegmentFrame(const Eigen::Vector3d& s1, const Eigen::Vector3d& s2, const Eigen::Vector3d& color = kPathColor, float line_width= kPathLineWidth);
dart::dynamics::SkeletonPtr createFloor();
dart::dynamics::SkeletonPtr createCylinder(const Eigen::Vector3d& position, float radius, float height);
dart::dynamics::SkeletonPtr createSphere(const Eigen::Vector3d& position, float radius);
