#pragma once

#include <ompl/base/State.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>
#include "projections/HopfFibrationProjection.hpp"

//const auto kEdgeEpsilon = 0.05;
const auto kEdgeEpsilon = 0.001;
const auto kStereographicDistanceThreshold = 0.2;

class HopfFibrationMotionValidator : public ompl::multilevel::TaskSpaceMotionValidator
{
 public:
  HopfFibrationMotionValidator() = delete;
  explicit HopfFibrationMotionValidator(const ompl::base::SpaceInformationPtr& bundle, const std::shared_ptr<HopfFibrationProjection>& hopf_fibration_projection);
  virtual ~HopfFibrationMotionValidator() override;

  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

  std::vector<ompl::base::State*> propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

 protected:
  bool MaybeRepairState(ompl::base::State *bundleState, const std::vector<ompl::base::State*> lastStates) const;
  double StereographicDistance(const ompl::base::State *s1, const ompl::base::State *s2) const;

  std::shared_ptr<HopfFibrationProjection> hopf_fibration_projection_;

  ompl::base::State* baseState1_;
  ompl::base::State* baseState2_;
  ompl::base::State* baseState3_;

  ompl::base::State* fiberState1_;
  ompl::base::State* fiberState2_;
  ompl::base::State* fiberState3_;
};


