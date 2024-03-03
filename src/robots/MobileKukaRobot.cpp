#include "robots/MobileKukaRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "KinematicsSolver.hpp"

dart::dynamics::SkeletonPtr MobileKukaRobot::MakeSkeleton() {
  const auto urdf_name = "/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka_endeffector_mobile.urdf";

  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FLOATING;
  loader.setOptions(options);

  dart::dynamics::SkeletonPtr manipulator
    = loader.parseSkeleton(urdf_name);

  static int count = 0;
  manipulator->setName("manipulator_"+std::to_string(count++));
  manipulator->setMobile(false);
  manipulator->setSelfCollisionCheck(true);
  manipulator->setAdjacentBodyCheck(true);

  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = State3d{0.0, 0.0, -0.2};
  manipulator->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(transform);

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

ompl::multilevel::FactoredSpaceInformationPtr MobileKukaRobot::MakeSpaceInformation(const RobotPtr& robot) {
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  auto numDofs = robot->GetSkeleton()->getNumDofs() - 3;

  auto spaceRN = std::make_shared<ompl::base::RealVectorStateSpace>(numDofs);
  auto spaceSE2 = std::make_shared<ompl::base::SE2StateSpace>();
  auto space = spaceSE2 + spaceRN;

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::RealVectorBounds bounds(numDofs);
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k =0; k< numDofs; k++) {
    bounds.setLow(k, lb[k+3]);
    bounds.setHigh(k, ub[k+3]);
  }
  spaceRN->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds SE2
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::RealVectorBounds boundsSE2(2);
  for(size_t k =0; k< 2; k++) {
    boundsSE2.setLow(k, lb[k]);
    boundsSE2.setHigh(k, ub[k]);
  }
  spaceSE2->setBounds(boundsSE2);

  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}

StateXd MobileKukaRobot::StateToEigen(const ompl::base::State* state) const {
  auto N = GetDimension();
  Eigen::VectorXd v(N);
  const auto *state_SE2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0);
  const auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1);

  v[0] = state_SE2->getX();
  v[1] = state_SE2->getY();
  v[2] = state_SE2->getYaw();
  for (unsigned int k = 0; k < N-3; k++)
  {
      v[k + 3] = state_RN->values[k];
  }
  return MakeState(v);
}

void MobileKukaRobot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.configuration.size();
  auto *state_SE2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SE2StateSpace::StateType>(0);
  auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(1);

  state_SE2->setX(v.configuration[0]);
  state_SE2->setY(v.configuration[1]);
  state_SE2->setYaw(v.configuration[2]);

  for (unsigned int k = 0; k < N-3; k++)
  {
      state_RN->values[k] = v.configuration[k + 3];
  }
}
