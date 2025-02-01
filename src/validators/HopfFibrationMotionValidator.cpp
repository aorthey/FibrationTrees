#include "validators/HopfFibrationMotionValidator.hpp"

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/special/SphereStateSpace.h>

HopfFibrationMotionValidator::HopfFibrationMotionValidator(const ompl::base::SpaceInformationPtr& bundle, const std::shared_ptr<HopfFibrationProjection>& hopf_fibration_projection) 
  : ompl::multilevel::TaskSpaceMotionValidator(bundle), hopf_fibration_projection_(hopf_fibration_projection) {
    baseState1_ = hopf_fibration_projection_->getBase()->allocState();
    baseState2_ = hopf_fibration_projection_->getBase()->allocState();
    baseState3_ = hopf_fibration_projection_->getBase()->allocState();
    fiberState1_ = hopf_fibration_projection_->getFiberSpace()->allocState();
    fiberState2_ = hopf_fibration_projection_->getFiberSpace()->allocState();
    fiberState3_ = hopf_fibration_projection_->getFiberSpace()->allocState();
}

HopfFibrationMotionValidator::~HopfFibrationMotionValidator() {
    hopf_fibration_projection_->getBase()->freeState(baseState3_);
    hopf_fibration_projection_->getBase()->freeState(baseState2_);
    hopf_fibration_projection_->getBase()->freeState(baseState1_);
    hopf_fibration_projection_->getFiberSpace()->freeState(fiberState3_);
    hopf_fibration_projection_->getFiberSpace()->freeState(fiberState2_);
    hopf_fibration_projection_->getFiberSpace()->freeState(fiberState1_);
}

bool HopfFibrationMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  throw std::runtime_error("Not yet implemented");
}

bool HopfFibrationMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  throw std::runtime_error("Not yet implemented");
}

bool HopfFibrationMotionValidator::MaybeRepairState(ompl::base::State *bundleState, const std::vector<ompl::base::State*> lastStates) const {
  if(lastStates.empty()) {
    return true;
  }
  const auto last_element = hopf_fibration_projection_->toVector(lastStates.back());
  const auto next_element = hopf_fibration_projection_->toVector(bundleState);
  const auto stereographic_distance = (next_element - last_element).norm();
  if(stereographic_distance > kStereographicDistanceThreshold) {
    auto state = bundleState->as<ompl::base::SO3StateSpace::StateType>();
    state->x = -state->x;
    state->y = -state->y;
    state->z = -state->z;
    state->w = -state->w;
    const auto new_next_element = hopf_fibration_projection_->toVector(bundleState);
    const auto new_stereographic_distance = (new_next_element - last_element).norm();
    if(new_stereographic_distance > kStereographicDistanceThreshold) {
      OMPL_ERROR("New stereographic_distance is %f (old was %f)", new_stereographic_distance, stereographic_distance);
      hopf_fibration_projection_->getBundle()->printState(bundleState);
      return false;
    }
  }
  return true;
}

std::vector<ompl::base::State*> HopfFibrationMotionValidator::propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {

  hopf_fibration_projection_->project(s1, baseState1_);
  hopf_fibration_projection_->project(s2, baseState2_);

  hopf_fibration_projection_->projectFiber(s1, fiberState1_);
  hopf_fibration_projection_->projectFiber(s2, fiberState2_);

  auto total_distance = hopf_fibration_projection_->getBase()->distance(baseState1_, baseState2_);

  std::vector<ompl::base::State*> states;

  auto startState = hopf_fibration_projection_->getBundle()->allocState();
  hopf_fibration_projection_->getBundle()->copyState(startState, s1);
  states.push_back(startState);

  double d = kEdgeEpsilon;

  while(d < total_distance) {
    double s = d / total_distance;
    hopf_fibration_projection_->getBase()->interpolate(baseState1_, baseState2_, s, baseState3_);
    hopf_fibration_projection_->getFiberSpace()->interpolate(fiberState1_, fiberState2_, s, fiberState3_);

    auto bundleState = hopf_fibration_projection_->getBundle()->allocState();
    hopf_fibration_projection_->lift(baseState3_, fiberState3_, bundleState);

    ////////////////////////////////////////////////////////////////////////////////
    //START DEBUG
    ////////////////////////////////////////////////////////////////////////////////
    auto baseState = hopf_fibration_projection_->getBase()->allocState();
    hopf_fibration_projection_->project(bundleState, baseState);
    const double desired_theta = baseState->as<ompl::base::SphereStateSpace::StateType>()->getTheta();
    const double desired_phi = baseState->as<ompl::base::SphereStateSpace::StateType>()->getPhi();
    const double result_theta = baseState3_->as<ompl::base::SphereStateSpace::StateType>()->getTheta();
    const double result_phi = baseState3_->as<ompl::base::SphereStateSpace::StateType>()->getPhi();
    if(abs(desired_theta-result_theta) > 1e-6
        || abs(desired_phi-result_phi) > 1e-6) {
      hopf_fibration_projection_->getBase()->printState(baseState, std::cout);
      hopf_fibration_projection_->getBase()->printState(baseState3_, std::cout);
      exit(0);
    }
    hopf_fibration_projection_->getBase()->freeState(baseState);
    ////////////////////////////////////////////////////////////////////////////////
    // END DEBUG
    ////////////////////////////////////////////////////////////////////////////////


    if(!MaybeRepairState(bundleState, states)) {
      return states;
    }

    states.push_back(bundleState);

    d+= kEdgeEpsilon;
  }
  auto bundleState = hopf_fibration_projection_->getBundle()->allocState();
  hopf_fibration_projection_->getBundle()->copyState(bundleState, s2);
  if(!MaybeRepairState(bundleState, states)) {
    return states;
  }
  states.push_back(bundleState);
  return states;
}
