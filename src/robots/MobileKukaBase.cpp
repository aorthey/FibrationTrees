#include "robots/MobileKukaBase.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "FilePath.hpp"
#include "validators/DefaultMotionValidator.hpp"

dart::dynamics::SkeletonPtr MobileKukaBase::MakeSkeleton(const YAML::Node& /*node*/) {
  const auto urdf_name = GetDataFolder() + "robots/kuka_lwr/kuka_mobile_base_only.urdf";

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
  return skeleton;
}

ompl::multilevel::FactoredSpaceInformationPtr MobileKukaBase::MakeSpaceInformation(const RobotPtr& robot) {
  auto space = std::make_shared<ompl::base::SE2StateSpace>();

  ////////////////////////////////////////////////////////////////////////////////
  // Set Bounds RN
  ////////////////////////////////////////////////////////////////////////////////
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

StateXd MobileKukaBase::StateToEigen(const ompl::base::State* state) const {
  Eigen::VectorXd v(3);
  const auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  v[0] = state_SE2->getX();
  v[1] = state_SE2->getY();
  v[2] = state_SE2->getYaw();
  return MakeState(v);
}

void MobileKukaBase::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.configuration.size();
  auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  state_SE2->setX(v.configuration[0]);
  state_SE2->setY(v.configuration[1]);
  state_SE2->setYaw(v.configuration[2]);
}

ompl::base::MotionValidatorPtr MobileKukaBase::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& /*robot*/) {
  return std::make_shared<DefaultMotionValidator>(factor);
}
