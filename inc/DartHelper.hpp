#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/math/Random.hpp>

#include "State.hpp"

const State3d kPathColor = State3d(0.3, 0.8, 0.3);
const float kPathSphereSize = 0.01;
const float kDefaultLineWidth = 5.0;

const State3d color_red = State3d(0.8, 0.1, 0.1);
const State3d color_red_light = State3d(1.0, 0.4, 0.4);

const State3d color_green = State3d(0.1, 0.8, 0.1);
const State3d color_green_light = State3d(0.4, 1.0, 0.4);

const State3d color_blue = State3d(0.1, 0.1, 0.8);
const State3d color_blue_light = State3d(0.4, 0.4, 1.0);

void PrintSkeletonInfo(const dart::dynamics::SkeletonPtr& skeleton);
StateXd GetRandomPosition(const dart::dynamics::SkeletonPtr& skeleton);

////////////////////////////////////////////////////////////////////////////////
// Skeleton constructors
////////////////////////////////////////////////////////////////////////////////
dart::dynamics::SkeletonPtr createFloor(float z_position = 0.0);
dart::dynamics::SkeletonPtr createCylinder(const State3d& position, float radius, float height);
dart::dynamics::SkeletonPtr createSphere(float radius);
dart::dynamics::SkeletonPtr createBox(const State3d& position, float length_x, float length_y, float length_z);
dart::dynamics::SkeletonPtr createFromURDF(const std::string& urdf_name, const State3d& position);
void changeBodyColor(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::Vector4d& color);
void hide(const dart::dynamics::SkeletonPtr& skeleton);
void show(const dart::dynamics::SkeletonPtr& skeleton);

////////////////////////////////////////////////////////////////////////////////
// Simple frame constructors
////////////////////////////////////////////////////////////////////////////////
dart::dynamics::SimpleFramePtr createCylinderFrame(const StateXd& position, const State3d& rotationXYZ, const float radius, const float height, const State4d& color);
dart::dynamics::SimpleFramePtr createCylinderFrame(const State3d& position, const State3d& rotationXYZ, const float radius, const float height, const State4d& color);
dart::dynamics::SimpleFramePtr createSphereFrame(const StateXd& position, const float radius = kPathSphereSize, const State3d& color = kPathColor);
dart::dynamics::SimpleFramePtr createSphereFrame(const State3d& position, const float radius = kPathSphereSize, const State3d& color = kPathColor);
dart::dynamics::SimpleFramePtr createLineSegmentFrame(const State3d& s1, const State3d& s2, const State3d& color = kPathColor, float line_width= kDefaultLineWidth);
dart::dynamics::SimpleFramePtr createLineSegmentFrame(const std::vector<State3d>& vertices, const State3d& color = kPathColor, float line_width= kDefaultLineWidth);
void addCoordinateFrameToWorld(const dart::simulation::WorldPtr& world);
