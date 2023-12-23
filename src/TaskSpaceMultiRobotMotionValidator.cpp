#include "TaskSpaceMultiRobotMotionValidator.hpp"

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"

TaskSpaceMultiRobotMotionValidator::TaskSpaceMultiRobotMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor)
  : ompl::base::DiscreteMotionValidator(factor)
{
  for(const auto& child : factor->getChildren()) {
    motion_validators_.push_back(child->getMotionValidator());
  }
  auto space = factor->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    // lastValids_.push_back({std::make_pair(space->getSubspace(k)->allocState(), 0.0f)});
    lastValids_.push_back(space->getSubspace(k)->allocState());
  }
  if(space->getSubspaceCount() != motion_validators_.size()) {
    OMPL_ERROR("Number of subspaces and number of motion validators does not match up (%d != %d).", 
        space->getSubspaceCount(), motion_validators_.size());
    throw "InvalidNumber";
  }
  tmpStateOnTotalSpace_ = factor->allocState();
}

TaskSpaceMultiRobotMotionValidator::~TaskSpaceMultiRobotMotionValidator() {
  auto space = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    space->getSubspace(k)->freeState(lastValids_.at(k));
  }
  si_->freeState(tmpStateOnTotalSpace_);
}

bool TaskSpaceMultiRobotMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  auto lastValid = std::make_pair(tmpStateOnTotalSpace_, 0.0);
  return TaskSpaceMultiRobotMotionValidator::checkMotion(s1, s2, lastValid);
}

bool TaskSpaceMultiRobotMotionValidator::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  if(lastValid.first == nullptr) {
    OMPL_ERROR("Last valid state is required.");
    throw "NYI";
  }

  auto s1_compound = s1->as<ompl::base::CompoundState>();
  auto s2_compound = s2->as<ompl::base::CompoundState>();

  auto space = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  bool all_subspaces_are_valid = true;

  ////////////////////////////////////////////////////////////////////////////////
  // Propagate states forward on each subspace until collision or failure
  ////////////////////////////////////////////////////////////////////////////////
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    auto spacek = space->getSubspace(k);
    auto s1_k = s1_compound->operator[](k);
    auto s2_k = s2_compound->operator[](k);

    std::pair<ompl::base::State *, double> lastValidSpaceK;
    lastValidSpaceK.first = lastValids_.at(k);
    lastValidSpaceK.second = 0.0;

    //Check motion for component subspace and return in lastValidSpaceK
    if(!motion_validators_.at(k)->checkMotion(s1_k, s2_k, lastValidSpaceK)) {
      all_subspaces_are_valid = false;
    }
    spacek->copyState(lastValid.first->as<ompl::base::CompoundState>()->operator[](k), lastValidSpaceK.first);
    lastValid.second += lastValidSpaceK.second;
    // OMPL_INFORM("Progress %f on %d", lastValidSpaceK.second, k);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Check that resulting states are valid for this factor
  ////////////////////////////////////////////////////////////////////////////////
  if(all_subspaces_are_valid || lastValid.second > 0.0) {
    const auto last_state = (all_subspaces_are_valid ? s2 : lastValid.first);
    //TODO: Needs to forward propagate, not direct interpolation
    if(!ompl::base::DiscreteMotionValidator::checkMotion(s1, last_state, lastValid)) {
      all_subspaces_are_valid = false;
    } else {
      // std::cout << "Verified motion." << std::endl;
    }
  }

  // OMPL_INFORM("Total %f", lastValid.second);
  return all_subspaces_are_valid;
}

