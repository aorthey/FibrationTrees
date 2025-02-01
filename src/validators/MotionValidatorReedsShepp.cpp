#include "validators/MotionValidatorReedsShepp.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"

MotionValidatorReedsShepp::MotionValidatorReedsShepp(
    const ompl::base::SpaceInformationPtr &si)
  : ompl::multilevel::TaskSpaceMotionValidator(si)
  //: ompl::base::DiscreteMotionValidator(si)
{
  lastValidState_ = si->allocState();
  reeds_shepp_motion_validator_ = std::make_shared<ompl::base::ReedsSheppMotionValidator>(si);
}

MotionValidatorReedsShepp::~MotionValidatorReedsShepp() {
  si_->freeState(lastValidState_);
}

bool MotionValidatorReedsShepp::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  return reeds_shepp_motion_validator_->checkMotion(s1, s2);
}

bool MotionValidatorReedsShepp::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  return reeds_shepp_motion_validator_->checkMotion(s1, s2, lastValid);
}

std::vector<ompl::base::State*> MotionValidatorReedsShepp::propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  std::pair<ompl::base::State *, double> lastValid;
  lastValid.first = lastValidState_;
  lastValid.second = 0.0;

  if(reeds_shepp_motion_validator_->checkMotion(s1, s2, lastValid)) {
    return MakeInterpolatedPathSegment(si_, s1, s2);
  }

  if(!(lastValid.second > 0.0)) {
    std::vector<ompl::base::State*> result;
    return result;
  }

  return MakeInterpolatedPathSegment(si_, s1, lastValidState_);
}
