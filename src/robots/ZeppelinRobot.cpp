#include "robots/ZeppelinRobot.hpp"

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorBounds.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "FilePath.hpp"

dart::dynamics::SkeletonPtr ZeppelinRobot::MakeSkeleton() {
  const std::string urdf_name = GetDataFolder() + "robots/zeppelin_with_joints.urdf";
  dart::utils::DartLoader loader;
  dart::utils::DartLoader::Options options;
  options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FIXED;
  loader.setOptions(options);
  dart::dynamics::SkeletonPtr manipulator
    = loader.parseSkeleton(urdf_name);

  static int count = 0;
  manipulator->setName("zeppelin_"+std::to_string(count++));
  manipulator->setMobile(true);
  manipulator->setSelfCollisionCheck(true);
  manipulator->setAdjacentBodyCheck(true);

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

ompl::multilevel::FactoredSpaceInformationPtr ZeppelinRobot::MakeSpaceInformation(const RobotPtr& robot) {
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

  //Create rotational component
  ompl::base::StateSpacePtr spaceSO2(new ompl::base::SO2StateSpace());

  ompl::base::StateSpacePtr space = spaceR3 + spaceSO2;

  return std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
}

StateXd ZeppelinRobot::StateToEigen(const ompl::base::State* state) const {
  auto N = GetDimension();
  Eigen::VectorXd v(N);
  const auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1);
  for (unsigned int k = 0; k < N-1; k++)
  {
      v[k] = state_RN->values[k];
      // if(v[k] != v[k]) {
      //   OMPL_ERROR("Detected NaN in StateToEigen");
      //   factor_->printState(state);
      //   throw "DetectedNaN";
      // }
  }
  v[N-1] = state_SO2->value;
  return MakeState(v);
}

void ZeppelinRobot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.size();
  auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1);
  for (unsigned int k = 0; k < N-1; k++)
  {
      state_RN->values[k] = v[k];
      // if(v[k] != v[k]) {
      //   OMPL_ERROR("Detected NaN in EigenToState");
      //   factor_->printState(state);
      //   throw "DetectedNaN";
      // }
  }
  state_SO2->value = v[N-1];
}

