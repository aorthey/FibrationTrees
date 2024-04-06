#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"

#include "OmplHelper.hpp"
#include "Common.hpp"

bool kDebug = false;

MotionValidatorTaskSpaceMultiRobot::MotionValidatorTaskSpaceMultiRobot(const ompl::multilevel::FactoredSpaceInformationPtr& factor)
  : ompl::multilevel::TaskSpaceMotionValidator(factor)
{
  for(const auto& child : factor->getChildren()) {
    motion_validators_.push_back(child->getMotionValidator());
  }
  auto space = factor->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    lastValids_.push_back(space->getSubspace(k)->allocState());
  }
  if(space->getSubspaceCount() != motion_validators_.size()) {
    OMPL_ERROR("Number of subspaces and number of motion validators does not match up (%d != %d).", 
        space->getSubspaceCount(), motion_validators_.size());
    throw "InvalidNumber";
  }
  tmpStateOnTotalSpace_ = factor->allocState();
}

MotionValidatorTaskSpaceMultiRobot::~MotionValidatorTaskSpaceMultiRobot() {
  auto space = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    space->getSubspace(k)->freeState(lastValids_.at(k));
  }
  si_->freeState(tmpStateOnTotalSpace_);
}

bool MotionValidatorTaskSpaceMultiRobot::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  auto lastValid = std::make_pair(tmpStateOnTotalSpace_, 0.0);
  return MotionValidatorTaskSpaceMultiRobot::checkMotion(s1, s2, lastValid);
}

bool MotionValidatorTaskSpaceMultiRobot::checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const {
  OMPL_ERROR("Last valid state is required.");
  throw "NYI";
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
    //copyState: destination <- source
    // spacek->copyState(lastValid.first->as<ompl::base::CompoundState>()->operator[](k), lastValidSpaceK.first);
    spacek->copyState(lastValid.first->as<ompl::base::CompoundState>()->operator[](k), lastValids_.at(k));
    lastValid.second += lastValidSpaceK.second;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Check that resulting states are valid for this factor
  ////////////////////////////////////////////////////////////////////////////////
  //if(all_subspaces_are_valid || lastValid.second > 0.0) {
  //  const auto last_state = (all_subspaces_are_valid ? s2 : lastValid.first);
  //  //TODO: Needs to forward propagate, not direct interpolation
  //  if(!ompl::base::DiscreteMotionValidator::checkMotion(s1, last_state, lastValid)) {
  //    all_subspaces_are_valid = false;
  //  } else {
  //    std::cout << "Verified motion." << std::endl;
  //  }
  //}
  return all_subspaces_are_valid;
}

std::vector<ompl::base::State*> MotionValidatorTaskSpaceMultiRobot::propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const {
  auto s1_compound = s1->as<ompl::base::CompoundState>();
  auto s2_compound = s2->as<ompl::base::CompoundState>();
  auto space = si_->getStateSpace()->as<ompl::base::CompoundStateSpace>();

  ////////////////////////////////////////////////////////////////////////////////
  // Propagate states forward on each subspace until collision or failure
  ////////////////////////////////////////////////////////////////////////////////

  std::vector<std::vector<ompl::base::State*>> split_states;

  size_t max_number_states = 0;
  for(size_t k = 0; k < space->getSubspaceCount(); k++) {
    auto spacek = space->getSubspace(k);
    auto s1_k = s1_compound->operator[](k);
    auto s2_k = s2_compound->operator[](k);

    auto motion_validator = std::static_pointer_cast<TaskSpaceMotionValidator>(motion_validators_.at(k));
    auto states = motion_validator->propagateMotion(s1_k, s2_k);

    max_number_states = std::max(max_number_states, states.size());
    split_states.push_back(states);
    if(kDebug) {
      std::cout << "Space " << space->getSubspace(k)->getName() << " propagated " << states.size() << " states." << std::endl;
    }
  }
  if(kDebug) {
    std::cout << "Check validity on " << max_number_states << " states." << std::endl;
  }

  std::vector<ompl::base::State*> result;
  //Abort when no progress was made
  if(max_number_states == 0) {
    OMPL_DEBUG("No progress");
    return result;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Stitch individual states together into one path
  ////////////////////////////////////////////////////////////////////////////////
  for(size_t i = 0; i < max_number_states; i++) {
    auto state = space->allocState();

    for(size_t space_index = 0; space_index < space->getSubspaceCount(); space_index++) {
      auto spacek = space->getSubspace(space_index);
      auto statesk = split_states.at(space_index);

      auto N = statesk.size();

      //Choose next state on this space
      const ompl::base::State* sk;
      if( N <= 0) {
        sk = s1_compound->operator[](space_index);
      } else {
        if( i < N - 1) {
          sk = statesk.at(i);
        } else {
          sk = statesk.back();
        }
      }
      spacek->copyState(state->as<ompl::base::CompoundState>()->operator[](space_index), sk);
    }

    ////////////////////////////////////////////////////////////////////////////////
    // Check that resulting states are valid 
    ////////////////////////////////////////////////////////////////////////////////
    if(!si_->isValid(state)) {
      return result;
    }
    result.push_back(state);
  }

  return result;
}
