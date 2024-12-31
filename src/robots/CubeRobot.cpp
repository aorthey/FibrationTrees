#include "robots/CubeRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>

#include "DartHelper.hpp"

dart::dynamics::SkeletonPtr CubeRobot::MakeSkeleton(const YAML::Node& node) {
  if(node["size"]) {
    const auto size = node["size"].as<std::vector<double>>();
    if(size.size() != 3) {
      throw std::domain_error("CubeRobot size must be 3 (width, length, height).");
    }
    width_ = size[0];
    length_ = size[1];
    height_ = size[2];
  }

  auto skeleton = createPlanarBox(State3d(0, 0, 0.5*height_), width_, length_, height_);

  if(node["lower_limits"]) {
    const auto lower_limits = node["lower_limits"].as<std::vector<double>>();
    if(lower_limits.size() != 2) {
      throw std::domain_error("CubeRobot lower_limits must be size 2 (x, y).");
    }
    skeleton->setPositionLowerLimits({0,1}, MakeState2d(lower_limits));
  }
  if(node["upper_limits"]) {
    const auto upper_limits = node["upper_limits"].as<std::vector<double>>();
    if(upper_limits.size() != 2) {
      throw std::domain_error("CubeRobot upper_limits must be size 2 (x, y).");
    }
    skeleton->setPositionUpperLimits({0,1}, MakeState2d(upper_limits));
  }

  return skeleton;
}

void CubeRobot::SetLimits(const std::pair<State2d, State2d>& limits) {
  const auto& lb = limits.first;
  const auto& ub = limits.second;

  ompl::base::RealVectorBounds bounds(2);
  for(size_t k =0; k< 2; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  GetSpaceInformation()->getStateSpace()->as<ompl::base::SE2StateSpace>()->setBounds(bounds);
  skeleton_->setPositionLowerLimits(lb);
  skeleton_->setPositionUpperLimits(ub);
}

ompl::multilevel::FactoredSpaceInformationPtr CubeRobot::MakeSpaceInformation(const RobotPtr& robot) {
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

StateXd CubeRobot::StateToEigen(const ompl::base::State* state) const {
  Eigen::VectorXd v(3);
  const auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  v[0] = state_SE2->getX();
  v[1] = state_SE2->getY();
  v[2] = state_SE2->getYaw();
  return MakeState(v);
}

void CubeRobot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto N = v.configuration.size();
  auto *state_SE2 = state->as<ompl::base::SE2StateSpace::StateType>();

  state_SE2->setX(v.configuration[0]);
  state_SE2->setY(v.configuration[1]);
  state_SE2->setYaw(v.configuration[2]);
}
