#include "robots/MobileCar.hpp"

#include <ompl/base/spaces/ReedsSheppStateSpace.h>

#include "FilePath.hpp"
#include "validators/MotionValidatorReedsShepp.hpp"
#include "yaml/SkeletonHelpers.hpp"

dart::dynamics::SkeletonPtr MobileCar::MakeSkeleton(const YAML::Node& node) {
  const auto urdf_name = GetDataFolder() + "robots/car.urdf";

  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FLOATING;
  loader.setOptions(options);

  dart::dynamics::SkeletonPtr skeleton
    = loader.parseSkeleton(urdf_name);

  static int count = 0;
  skeleton->setName("base_"+std::to_string(count++));
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

ompl::multilevel::FactoredSpaceInformationPtr MobileCar::MakeSpaceInformation(const RobotPtr& robot) {
  auto space = std::make_shared<ompl::base::ReedsSheppStateSpace>();

  ompl::base::RealVectorBounds bounds(2);
  const auto lb = robot->GetSkeleton()->getPositionLowerLimits();
  const auto ub = robot->GetSkeleton()->getPositionUpperLimits();
  for(size_t k =0; k<2; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->setBounds(bounds);
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}

ompl::base::MotionValidatorPtr MobileCar::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& /*robot*/) {
  return std::make_shared<MotionValidatorReedsShepp>(factor);
}

std::vector<State3d> MobileCar::GetFK(const StateXd& config) const {
  auto endeffector = "frontspoiler";
  skeleton_->setConfiguration(config.configuration);
  return std::vector<State3d>({skeleton_->getBodyNode(endeffector)->getTransform().translation()});
}

StateXd MobileCar::StateToEigen(const ompl::base::State* state) const {
  Eigen::VectorXd v(3);
  const auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  v[0] = state_SE2->getX();
  v[1] = state_SE2->getY();
  v[2] = state_SE2->getYaw();
  return MakeState(v);
}

void MobileCar::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  state_SE2->setX(v.configuration[0]);
  state_SE2->setY(v.configuration[1]);
  state_SE2->setYaw(v.configuration[2]);
}

