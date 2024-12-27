#include "robots/MultiRobot.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"
#include "DartHelper.hpp"

MultiRobot::MultiRobot(const std::vector<RobotPtr>& robots) 
  : robots_(robots) {



}

MultiRobot::~MultiRobot() {
}

dart::dynamics::SkeletonPtr MultiRobot::MakeSkeleton() {
  std::cout << "MakeSkeleton" << std::endl;
  return nullptr;
}

std::shared_ptr<MultiRobot> MultiRobot::MakeMultiRobot(const std::vector<RobotPtr>& robots) {
  auto robot = std::make_shared<MultiRobot>(robots);
  auto factor = robot->MakeSpaceInformation(nullptr);
  robot->SetSpaceInformation(factor);
  return robot;
}

ompl::multilevel::FactoredSpaceInformationPtr MultiRobot::MakeSpaceInformation(const RobotPtr& robot_input) {
  std::vector<ompl::base::StateSpacePtr> compound_spaces;
  for(const auto& robot : robots_) {
    compound_spaces.push_back(robot->GetSpaceInformation()->getStateSpace());
  }
  auto space = std::make_shared<ompl::base::CompoundStateSpace>(compound_spaces, std::vector<double>(compound_spaces.size(), 1.0f));

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));
  return factor;
}

typedef std::unordered_map<std::string, ompl::base::State*> SplitConfig;

StateXd MultiRobot::StateToEigen(const ompl::base::State* state) const {
  // auto child_states = factor_->allocChildStates();
  // std::vector<SplitConfig> configs;
  // factor_->project(state, child_states);

  // std::vector<Eigen::VectorXd> eigen_vectors;

  // int dimension = 0;
  // for(const auto& robot : robots_) {
  //   auto state = child_states.at(robot->GetSpaceInformation()->getName());
  //   auto eigen_vector = robot->StateToEigen(state).configuration;
  //   eigen_vectors.push_back(eigen_vector);
  //   dimension += eigen_vector.rows();
  // }

  // Eigen::VectorXd result(dimension);

  // int current_dimension = 0;
  // for(const auto& eigen_vector : eigen_vectors) {
  //   for(size_t k = current_dimension; k < current_dimension + eigen_vector.rows(); k++) {
  //     result[k] = eigen_vector[k-current_dimension];
  //   }
  //   current_dimension += eigen_vector.rows();
  // }
  // return MakeState(result);

  auto compound_state = state->as<ompl::base::CompoundState>();

  size_t Ndimension = factor_->getStateDimension();
  Eigen::VectorXd result(Ndimension);

  size_t subspace_index = 0;
  size_t current_dimension = 0;
  for(const auto& robot : robots_) {

    int Nrobot = robot->GetSpaceInformation()->getStateDimension();
    auto eigen_vector = robot->StateToEigen(compound_state->operator[](subspace_index)).configuration;


    for(size_t k = current_dimension; k < current_dimension + eigen_vector.rows(); k++) {
      result[k] = eigen_vector[k-current_dimension];
    }
    current_dimension = current_dimension + Nrobot;
    subspace_index++;
  }
  return MakeState(result);
}

void MultiRobot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  auto state_space = factor_->getStateSpace();

  if(!state_space->isCompound()) {
    throw std::domain_error("Not a compound space:" + state_space->getName());
  }

  auto compound_space = state_space->as<ompl::base::CompoundStateSpace>();
  if(compound_space->getSubspaceCount() != robots_.size()) {
    throw std::out_of_range("Number of subspaces is "
        + std::to_string(compound_space->getSubspaceCount()) 
        + " but " + std::to_string(robots_.size()) + " robots are given.");
  }
  
  size_t current_dimension = 0;
  size_t subspace_index = 0;

  auto compound_state = state->as<ompl::base::CompoundState>();
  for(const auto& robot : robots_) {
    int Nrobot = robot->GetSpaceInformation()->getStateDimension();

    auto eigen_vector = v.configuration.segment(current_dimension, Nrobot);

    robot->EigenToState(MakeState(eigen_vector), compound_state->operator[](subspace_index));

    current_dimension = current_dimension + Nrobot;
    subspace_index++;
  }
}

std::vector<State3d> MultiRobot::GetFK(const StateXd& config) const {
  auto state = factor_->allocState();
  EigenToState(config, state);

  auto child_states = factor_->allocChildStates();
  factor_->project(state, child_states);

  std::vector<State3d> tcps;
  for(const auto& robot : robots_) {
    for(const auto& child_state : child_states) {
      if(child_state.first == robot->GetSpaceInformation()->getName()) {
        auto frames = robot->GetFK(child_state.second);
        for(const auto& tcp : frames) {
          tcps.push_back(tcp);
        }
      }
    }
  }
  factor_->freeChildStates(child_states);
  factor_->freeState(state);
  return tcps;
}

void MultiRobot::SetConfiguration(const StateXd& config) {
  int current_dimension = 0;
  for(const auto& robot : robots_) {
    int Nrobot = robot->GetSpaceInformation()->getStateDimension();
    auto eigen_vector = config.configuration.segment(current_dimension, Nrobot);
    robot->SetConfiguration(MakeState(eigen_vector));
    current_dimension = current_dimension + Nrobot;
  }
}
