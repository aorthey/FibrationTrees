#include "robots/MultiRobot.hpp"
#include "TaskSpaceMultiRobotMotionValidator.hpp"
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
  std::cout << "MakeSpaceInformation" << std::endl;
  std::vector<ompl::base::StateSpacePtr> compound_spaces;
  for(const auto& robot : robots_) {
    compound_spaces.push_back(robot->GetSpaceInformation()->getStateSpace());
  }
  std::cout << "MakeSpaceInformation" << std::endl;
  auto space = std::make_shared<ompl::base::CompoundStateSpace>(compound_spaces, std::vector<double>(compound_spaces.size(), 1.0f));

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));
  return factor;
}

typedef std::unordered_map<std::string, ompl::base::State*> SplitConfig;

StateXd MultiRobot::StateToEigen(const ompl::base::State* state) const {
  auto child_states = factor_->allocChildStates();
  std::vector<SplitConfig> configs;
  factor_->project(state, child_states);

  std::vector<Eigen::VectorXd> eigen_vectors;

  int dimension = 0;
  for(const auto& robot : robots_) {
    auto state = child_states.at(robot->GetSpaceInformation()->getName());
    auto eigen_vector = robot->StateToEigen(state).configuration;
    eigen_vectors.push_back(eigen_vector);
    dimension += eigen_vector.rows();
  }

  Eigen::VectorXd result(dimension);

  int current_dimension = 0;
  for(const auto& eigen_vector : eigen_vectors) {
    for(size_t k = current_dimension; k < current_dimension + eigen_vector.rows(); k++) {
      result[k] = eigen_vector[k-current_dimension];
    }
    current_dimension += eigen_vector.rows();
  }
  return MakeState(result);
}

void MultiRobot::EigenToState(const StateXd& v, ompl::base::State* state) const {
  int current_dimension = 0;
  auto child_states = factor_->allocChildStates();
  for(const auto& robot : robots_) {
    int Nrobot = robot->GetSpaceInformation()->getStateDimension();
    auto eigen_vector = v.configuration.segment(current_dimension, Nrobot);
    auto state = child_states.at(robot->GetSpaceInformation()->getName());
    robot->EigenToState(MakeState(eigen_vector), state);

    current_dimension = current_dimension + Nrobot;
  }
  factor_->lift(child_states, state);
  factor_->freeChildStates(child_states);
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
