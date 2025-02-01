#include "robots/Drone.hpp"

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "FilePath.hpp"
#include "yaml/SkeletonHelpers.hpp"

dart::dynamics::SkeletonPtr Drone::MakeSkeleton(const YAML::Node& node) {
  const std::string urdf_name = GetDataFolder() + "robots/dddk_with_joints.urdf";
  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FIXED;
  loader.setOptions(options);

  dart::dynamics::SkeletonPtr skeleton
    = loader.parseSkeleton(urdf_name);

  static int count = 0;
  skeleton->setName("drone_"+std::to_string(count++));
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

ompl::multilevel::FactoredSpaceInformationPtr Drone::MakeSpaceInformation(const RobotPtr& robot) {
  //Create translational component
  ompl::base::StateSpacePtr spaceR3(new ompl::base::RealVectorStateSpace(3));
  ompl::base::RealVectorBounds bounds(3);
  auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k = 0; k < 3; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  spaceR3->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  //Create rotational component around X and Y axis (Roll, Pitch)
  ompl::base::StateSpacePtr spaceR2(new ompl::base::RealVectorStateSpace(2));

  ompl::base::RealVectorBounds rotational_bounds(2);
  rotational_bounds.setLow(0, -0.57);
  rotational_bounds.setHigh(0, +0.57);
  rotational_bounds.setLow(1, -0.57);
  rotational_bounds.setHigh(1, +0.57);
  spaceR2->as<ompl::base::RealVectorStateSpace>()->setBounds(rotational_bounds);

  //Create rotational component around Z axis (Yaw)
  ompl::base::StateSpacePtr spaceSO2(new ompl::base::SO2StateSpace());

  ompl::base::StateSpacePtr space = spaceR3 + spaceR2 + spaceSO2;

  return std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
}

StateXd Drone::StateToEigen(const ompl::base::State* state) const {

  auto N = GetDimension();
  Eigen::VectorXd v(N);

  const auto *state_R3 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const auto *state_R2 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1);
  const auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(2);

  v[0] = state_R3->values[0];
  v[1] = state_R3->values[1];
  v[2] = state_R3->values[2];

  v[3] = state_R2->values[0];
  v[4] = state_R2->values[1];

  v[5] = state_SO2->value;


  return MakeState(v);
}

void Drone::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.size();
  auto *state_R3 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  auto *state_R2 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1);
  auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(2);

  state_R3->values[0] = v[0];
  state_R3->values[1] = v[1];
  state_R3->values[2] = v[2];

  state_R2->values[0] = v[3];
  state_R2->values[1] = v[4];

  state_SO2->value = v[5];
}

