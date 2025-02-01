#include "robots/MobileKukaRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "KinematicsSolver.hpp"
#include "FilePath.hpp"
#include "validators/DefaultMotionValidator.hpp"
#include "yaml/SkeletonHelpers.hpp"

dart::dynamics::SkeletonPtr MobileKukaRobot::MakeSkeleton(const YAML::Node& node) {
  const auto urdf_name = GetDataFolder() + "robots/kuka_lwr/kuka_endeffector_mobile.urdf";

  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FLOATING;
  loader.setOptions(options);

  dart::dynamics::SkeletonPtr skeleton
    = loader.parseSkeleton(urdf_name);

  static int count = 0;
  skeleton->setName("manipulator_"+std::to_string(count++));
  skeleton->setMobile(false);
  skeleton->setSelfCollisionCheck(true);
  skeleton->setAdjacentBodyCheck(true);

  Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
  transform.translation() = State3d{0.0, 0.0, -0.2};
  skeleton->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(transform);

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

  SetSkeletonLowerLimits(skeleton, node, 2u);
  SetSkeletonUpperLimits(skeleton, node, 2u);
  return skeleton;
}

ompl::multilevel::FactoredSpaceInformationPtr MobileKukaRobot::MakeSpaceInformation(const RobotPtr& robot) {
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  auto numDofsAll = robot->GetSkeleton()->getNumDofs();
  auto numDofsManipulator = numDofsAll - 3;

  auto spaceRN = std::make_shared<ompl::base::RealVectorStateSpace>(numDofsManipulator);
  auto spaceSE2 = std::make_shared<ompl::base::SE2StateSpace>();
  auto space = spaceSE2 + spaceRN;

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::RealVectorBounds bounds(numDofsManipulator);
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k =0; k< numDofsManipulator; k++) {
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

std::vector<State3d> MobileKukaRobot::GetFK(const StateXd& config) const {
  auto endeffector = "base_link_robot";
  if(config.configuration != config.configuration) {
    OMPL_ERROR("Detected NaN value for FK config.");
    std::cout << "set config " << config << " for robot " << GetName() << std::endl;
    exit(0);
  }
  skeleton_->setConfiguration(config.configuration);
  return std::vector<State3d>({skeleton_->getBodyNode(endeffector)->getTransform().translation()});
}
