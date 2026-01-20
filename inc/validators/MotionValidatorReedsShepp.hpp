#pragma once

#include <ompl/base/State.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>

const float kMinimumSpacing = 0.05; //minimal distance between two joint configurations (was 0.25)

//class MotionValidatorReedsShepp : public ompl::multilevel::TaskSpaceMotionValidator
class MotionValidatorReedsShepp : public ompl::base::DiscreteMotionValidator
{
 public:
  MotionValidatorReedsShepp() = delete;
  explicit MotionValidatorReedsShepp(const ompl::base::SpaceInformationPtr &si);
  virtual ~MotionValidatorReedsShepp() override;

  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

  //std::vector<ompl::base::State*> propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

 protected:
  ompl::base::State* lastValidState_;

  std::shared_ptr<ompl::base::ReedsSheppMotionValidator> reeds_shepp_motion_validator_;
};

