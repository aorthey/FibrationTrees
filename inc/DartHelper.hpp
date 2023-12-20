#pragma once

#include "dart/dart.hpp"
#include "dart/gui/osg/osg.hpp"
#include "dart/utils/urdf/urdf.hpp"
#include "dart/math/Random.hpp"

const Eigen::Vector3d kPathColor = Eigen::Vector3d(0.3, 0.8, 0.3);
const float kPathSphereSize = 0.01;
const float kDefaultLineWidth = 5.0;

const Eigen::Vector3d color_red = Eigen::Vector3d(0.8, 0.1, 0.1);
const Eigen::Vector3d color_red_light = Eigen::Vector3d(1.0, 0.4, 0.4);

const Eigen::Vector3d color_green = Eigen::Vector3d(0.1, 0.8, 0.1);
const Eigen::Vector3d color_green_light = Eigen::Vector3d(0.4, 1.0, 0.4);

const Eigen::Vector3d color_blue = Eigen::Vector3d(0.1, 0.1, 0.8);
const Eigen::Vector3d color_blue_light = Eigen::Vector3d(0.4, 0.4, 1.0);

void PrintSkeletonInfo(const dart::dynamics::SkeletonPtr& skeleton);
Eigen::VectorXd GetRandomPosition(const dart::dynamics::SkeletonPtr& skeleton);

////////////////////////////////////////////////////////////////////////////////
// Skeleton constructors
////////////////////////////////////////////////////////////////////////////////
dart::dynamics::SkeletonPtr createFloor();
dart::dynamics::SkeletonPtr createCylinder(const Eigen::Vector3d& position, float radius, float height);
dart::dynamics::SkeletonPtr createSphere(const Eigen::Vector3d& position, float radius);
dart::dynamics::SkeletonPtr createBox(const Eigen::Vector3d& position, float length_x, float length_y, float length_z);
dart::dynamics::SkeletonPtr createFromURDF(const std::string& urdf_name, const Eigen::Vector3d& position);

////////////////////////////////////////////////////////////////////////////////
// Simple frame constructors
////////////////////////////////////////////////////////////////////////////////
dart::dynamics::SimpleFramePtr createSphereFrame(const Eigen::Vector3d& position, const float radius = kPathSphereSize, const Eigen::Vector3d& color = kPathColor);
dart::dynamics::SimpleFramePtr createLineSegmentFrame(const Eigen::Vector3d& s1, const Eigen::Vector3d& s2, const Eigen::Vector3d& color = kPathColor, float line_width= kDefaultLineWidth);
dart::dynamics::SimpleFramePtr createLineSegmentFrame(const std::vector<Eigen::Vector3d>& vertices, const Eigen::Vector3d& color = kPathColor, float line_width= kDefaultLineWidth);
void addCoordinateFrameToWorld(const dart::simulation::WorldPtr& world);

////////////////////////////////////////////////////////////////////////////////
// Misc
////////////////////////////////////////////////////////////////////////////////
Eigen::Vector3d GetFK(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::VectorXd& config);
