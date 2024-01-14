#include "OmplHelper.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/GoalState.h>

Eigen::Vector3d StateToEigenVector3d(const ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  Eigen::Vector3d v;
  for(size_t k = 0; k < 3; k++) {
    v[k] = state_R[k];
  }
  return v;
}

Eigen::Vector3d ProjectStateToEigenVector3d(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::State* state) {
  ompl::base::State *projected_state = projection->getBase()->allocState();
  projection->project(state, projected_state);
  double *state_R3 = projected_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  Eigen::Vector3d v;
  v << state_R3[0], state_R3[1], state_R3[2];
  projection->getBase()->freeState(projected_state);
  return v;
}

void EigenVector3dToState(const Eigen::Vector3d& v, ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for(size_t k = 0; k < 3; k++) {
    state_R[k] = v[k];
  }
}

void EigenVectorXdToState(const Eigen::VectorXd& v, ompl::base::State* state) {
  double *state_R = state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  for(size_t k = 0; k < v.size(); k++) {
    state_R[k] = v[k];
  }
}

bool SampleValidLift(const ompl::multilevel::ProjectionPtr& projection, const ompl::base::SpaceInformationPtr& si, 
    const size_t max_iterations, const ompl::base::State *xBase, ompl::base::State *xBundle) {
  int iterations=0;
  while(iterations++ < max_iterations) {
    projection->lift(xBase, xBundle);
    if(si->getStateValidityChecker()->isValid(xBundle)) {
      return true;
    }
  }
  return false;
}

ompl::geometric::PathGeometricPtr PathFromEigenVectors(const std::vector<Eigen::VectorXd>& configs,
    const ompl::base::SpaceInformationPtr& si) {
  std::vector<const ompl::base::State*> states;
  for(const auto& config : configs) {
    auto state = si->allocState();
    EigenVectorXdToState(config, state);
    states.push_back(state);
  }
  return std::make_shared<ompl::geometric::PathGeometric>(si, states);
}

const int kMaxResampleIteration = 100;
std::optional<ompl::base::State*> ComputeValidIKState(const ompl::base::SpaceInformationPtr& si, 
    const ompl::multilevel::ProjectionPtr& projection, const Eigen::Vector3d& point) {

  ompl::base::State *task_state = projection->getBase()->allocState();
  double *angles = task_state->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  angles[0] = point[0];
  angles[1] = point[1];
  angles[2] = point[2];

  ompl::base::State *state = si->allocState();
  if(!SampleValidLift(projection, si, kMaxResampleIteration, task_state, state)) {
    return std::nullopt;
  }

  return state;
}

std::optional<ompl::base::State*> ComputeValidTotalState(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const std::unordered_map<std::string, ompl::base::State*>& leaf_node_states) {
  ompl::base::State *state = factor->allocState();

  size_t samples = 0;
  while(samples++ < kMaxResampleIteration) {
    factor->liftLeafStates(leaf_node_states, state);
    if(factor->isValid(state)) {
      return state;
    }
  }

  OMPL_ERROR("Invalid total state after %d iterations.", samples);
  factor->freeState(state);
  return std::nullopt;
}

ompl::base::State* AllocStateFromEigen(const RobotPtr& robot, const Eigen::VectorXd& v) {
  auto state = robot->GetSpaceInformation()->allocState();
  robot->EigenToState(v, state);
  return state;
}

ompl::base::GoalPtr GoalFromEigen(const RobotPtr& robot, const Eigen::VectorXd& v, float threshold) {
  auto goal_region = std::make_shared<ompl::base::GoalState>(robot->GetSpaceInformation());
  auto state = AllocStateFromEigen(robot, v);
  robot->GetSpaceInformation()->printState(state);
  goal_region->setState(state);
  goal_region->setThreshold(threshold);
  return goal_region;
}

Eigen::VectorXd MakeEigen(std::initializer_list<double> const &init_values) {
  std::vector<double> values{init_values};
  // Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(values.data(), values.size());
  // return v;
  return MakeEigen(values);
}
Eigen::VectorXd MakeEigen(std::vector<double> values) {
  Eigen::VectorXd v = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(values.data(), values.size());
  return v;
}
