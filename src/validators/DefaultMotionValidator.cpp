#include "validators/DefaultMotionValidator.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"

DefaultMotionValidator::DefaultMotionValidator(
    const ompl::base::SpaceInformationPtr &si)
  : ompl::multilevel::TaskSpaceMotionValidator(si)
{
  lastValidState_ = si->allocState();
}

DefaultMotionValidator::~DefaultMotionValidator() {
  si_->freeState(lastValidState_);
}

bool DefaultMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  return TaskSpaceMotionValidator::checkMotion(s1, s2);
}

bool DefaultMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  return TaskSpaceMotionValidator::checkMotion(s1, s2, lastValid);
}

std::vector<ompl::base::State*> DefaultMotionValidator::propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  std::pair<ompl::base::State *, double> lastValid;
  lastValid.first = lastValidState_;
  lastValid.second = 0.0;

  std::vector<ompl::base::State*> result;
  if(TaskSpaceMotionValidator::checkMotion(s1, s2, lastValid)) {
    return MakeInterpolatedPathSegment(si_, s1, s2);
  }

  if(!(lastValid.second > 0.0)) {
    return result;
  }

  return MakeInterpolatedPathSegment(si_, s1, lastValidState_);
}

