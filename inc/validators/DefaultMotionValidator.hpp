#pragma once

#include <ompl/base/State.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>

const float kMinimumSpacing = 0.05; //minimal distance between two joint configurations (was 0.25)

class DefaultMotionValidator : public ompl::multilevel::TaskSpaceMotionValidator
{
 public:
  DefaultMotionValidator() = delete;
  explicit DefaultMotionValidator(const ompl::base::SpaceInformationPtr &si);
  virtual ~DefaultMotionValidator() override;

  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
  bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

  std::vector<ompl::base::State*> propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

 protected:
  ompl::base::State* lastValidState_;
};

