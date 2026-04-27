#include "robots/SE3Robot.hpp"

#include <ompl/base/spaces/SE3StateSpace.h>
#include <Eigen/Geometry>

#include "DartHelper.hpp"
#include "yaml/SkeletonHelpers.hpp"
#include "FilePath.hpp"

dart::dynamics::SkeletonPtr SE3Robot::MakeSkeleton(const YAML::Node& node) 
{
  if(!node["urdf_filename"]) {
    throw std::domain_error("SE3Robot requires urdf_filename");
  }
  urdf_filename_ = node["urdf_filename"].as<std::string>();
  const std::string urdf_name = GetDataFolder() + urdf_filename_;

  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FIXED;
  loader.setOptions(options);

  dart::dynamics::SkeletonPtr skeleton
    = loader.parseSkeleton(urdf_name);

  static int count = 0;
  skeleton->setName(urdf_filename_+"_"+std::to_string(count++));
  skeleton->setMobile(false);
  skeleton->setSelfCollisionCheck(true);
  skeleton->setAdjacentBodyCheck(true);

  //Disable friction. This was causing an assert error in ContactConstraint.cpp
  //in dartsim
  for(const auto& body_node : skeleton->getBodyNodes()) {
    auto nodes = body_node->getShapeNodesWith<dart::dynamics::DynamicsAspect>();
    for(const auto& node : nodes) {
      node->getDynamicsAspect()->setFrictionCoeff(0.0);
      node->getDynamicsAspect()->setPrimaryFrictionCoeff(0.0);
      node->getDynamicsAspect()->setSecondaryFrictionCoeff(0.0);
    }
  }
  SetSkeletonLowerLimits(skeleton, node, 3u);
  SetSkeletonUpperLimits(skeleton, node, 3u);
  return skeleton;

}

void SE3Robot::SetLimits(const std::pair<State3d, State3d>& limits) {
  const auto& lb = limits.first;
  const auto& ub = limits.second;

  ompl::base::RealVectorBounds bounds(3);
  for(size_t k = 0; k < 3; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  GetSpaceInformation()->getStateSpace()->as<ompl::base::SE3StateSpace>()->setBounds(bounds);
  skeleton_->setPositionLowerLimits(lb);
  skeleton_->setPositionUpperLimits(ub);
}

ompl::multilevel::FactoredSpaceInformationPtr SE3Robot::MakeSpaceInformation(const RobotPtr& robot) {
  auto space = std::make_shared<ompl::base::SE3StateSpace>();

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::RealVectorBounds bounds(3);
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k = 0; k < 3; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->setBounds(bounds);
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}

StateXd SE3Robot::StateToEigen(const ompl::base::State* state) const {
  Eigen::VectorXd v(6);
  const auto *state_SE3 = state->as<ompl::base::SE3StateSpace::StateType>();
  const auto *state_SO3 = &state_SE3->rotation();

  v[0] = state_SE3->getX();
  v[1] = state_SE3->getY();
  v[2] = state_SE3->getZ();

  Eigen::Quaterniond quat(state_SO3->w, state_SO3->x, state_SO3->y, state_SO3->z);
  Eigen::Vector3d euler = quat.toRotationMatrix().eulerAngles(2, 1, 0);  // ZYX (yaw, pitch, roll)
  v[3] = euler[0];
  v[4] = euler[1];
  v[5] = euler[2];

  return MakeState(v);
}

void SE3Robot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto *state_SE3 = state->as<ompl::base::SE3StateSpace::StateType>();
  auto *state_SO3 = &state_SE3->rotation();

  state_SE3->setX(v.configuration[0]);
  state_SE3->setY(v.configuration[1]);
  state_SE3->setZ(v.configuration[2]);

  double yaw   = v.configuration[3];  // rotation around Z
  double pitch = v.configuration[4];  // rotation around Y
  double roll  = v.configuration[5];  // rotation around X
  Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

  Eigen::Quaterniond quat = yawAngle * pitchAngle * rollAngle;  // ZYX order

  state_SO3->x = quat.x();
  state_SO3->y = quat.y();
  state_SO3->z = quat.z();
  state_SO3->w = quat.w();
}
