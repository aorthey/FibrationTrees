#include "robots/MultiRobot.hpp"
#include "TaskSpaceMultiRobotMotionValidator.hpp"
#include "DartHelper.hpp"

MultiRobot::MultiRobot(const std::vector<RobotPtr>& robots) 
  : robots_(robots) {
}

MultiRobot::~MultiRobot() {
  factor_->freeChildStates(child_states_);
}

dart::dynamics::SkeletonPtr MultiRobot::MakeSkeleton() {
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
Eigen::VectorXd MultiRobot::StateToEigen(const ompl::base::State* state) const {
  OMPL_ERROR("NYI");
  std::vector<SplitConfig> configs;
  factor_->project(state, child_states_);

  std::vector<Eigen::VectorXd> eigen_vectors;

  int dimension = 0;
  for(const auto& robot : robots_) {
    auto state = child_states_.at(robot->GetSpaceInformation()->getName());
    auto eigen_vector = robot->StateToEigen(state);
    eigen_vectors.push_back(eigen_vector);
    dimension += eigen_vector.rows();
  }

  Eigen::VectorXd result(dimension);

  int current_dimension = 0;
  for(const auto& eigen_vector : eigen_vectors) {
    for(size_t k = current_dimension; k < eigen_vector.rows(); k++) {
      result[k - current_dimension] = eigen_vector[k-current_dimension];
    }
    current_dimension += eigen_vector.rows();
  }
  return result;
}

std::vector<Eigen::Vector3d> MultiRobot::GetFK(const ompl::base::State* state) const {
  auto child_states = factor_->allocChildStates();
  factor_->project(state, child_states);

  std::vector<Eigen::Vector3d> tcps;
  for(const auto& robot : robots_) {
    for(const auto& child_state : child_states) {
      if(child_state.first == robot->GetSpaceInformation()->getName()) {
        auto tcp = ::GetFK(robot->GetSkeleton(), robot->StateToEigen(child_state.second));
        tcps.push_back(tcp);
      }
    }
  }
  factor_->freeChildStates(child_states);
  return tcps;
}

void MultiRobot::EigenToState(const Eigen::VectorXd& v, ompl::base::State* state) const {
  (void) v;
  (void) state;
  OMPL_ERROR("NYI");
}
