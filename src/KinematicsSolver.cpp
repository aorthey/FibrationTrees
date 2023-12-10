#include "KinematicsSolver.hpp"
#include "Common.hpp"

const float kStepSize = 0.1;

KinematicsSolver::KinematicsSolver(const dart::dynamics::SkeletonPtr& skeleton) 
  : skeleton_(skeleton) 
{
  endeffector_ = skeleton_->getBodyNode(skeleton_->getNumBodyNodes() - 1)->getName();
  if(skeleton_->getBodyNode(endeffector_)==nullptr) 
  {
    throw "EndeffectorDoesNotExist";
  }

}

std::optional<Eigen::VectorXd> KinematicsSolver::solve_ik(const Eigen::Vector3d& frame, size_t max_iterations) {
  const auto& lb = skeleton_->getPositionLowerLimits();
  const auto& ub = skeleton_->getPositionUpperLimits();

  Eigen::VectorXd zero = Eigen::VectorXd::Constant(lb.size(), -Inf);

  if(skeleton_->getBodyNode(endeffector_)==nullptr) 
  {
    return std::nullopt;
  }

  size_t counter = 0;

  while(counter++ < max_iterations) {
    auto config = dart::math::Random::uniform(lb, ub);
    skeleton_->setConfiguration(config);

    auto ik = skeleton_->getBodyNode(endeffector_)->getOrCreateIK();
    ik->getTarget()->setTranslation(frame);

    if(ik->solveAndApply(config, true))
    {
      return ik->getPositions();
    }
  }
  return std::nullopt;
}

const float kLimitPadding = 1e-2;
std::optional<Eigen::Vector3d> KinematicsSolver::solve_fk(const Eigen::VectorXd& config) {
  auto lb = skeleton_->getPositionLowerLimits();
  auto ub = skeleton_->getPositionUpperLimits();
  for(size_t k = 0; k < config.size(); k++) {
    if(config[k] < (lb[k] - kLimitPadding) || config[k] > (ub[k] + kLimitPadding) || config[k] != config[k]) {
      // std::cout << "Out of limits of " << k << "-th dof : " << lb[k] << "<" << config[k] << "<" << ub[k] << std::endl;
      return std::nullopt;
    }
  }
  skeleton_->setConfiguration(config);
  return skeleton_->getBodyNode(endeffector_)->getTransform().translation();
}

//Avoid duplicates, add additional configs during joint flips
void AddConfig(const Eigen::VectorXd& config, std::vector<Eigen::VectorXd>& configs) {
  if(configs.empty()) {
    configs.push_back(config);
    return;
  }
  const auto last_config = configs.back();
  const auto dist = (last_config-config).norm();
  if(dist < Epsilon) {
    //avoid duplicates
    return;
  }

  const float kAddStepSize = 0.01;
  int num_steps = dist / kAddStepSize;
  for(size_t k =0; k<num_steps;k++) {
    configs.push_back(last_config + num_steps*kAddStepSize * (config - last_config));
  }
  for(float d = kAddStepSize; d < dist; d+=kAddStepSize) {
    configs.push_back(last_config + d * (config - last_config));
  }
  if((config - configs.back()).norm() > Epsilon) {
    configs.push_back(config);
  }
  configs.push_back(config);
}


std::vector<Eigen::VectorXd> KinematicsSolver::solve_edge_ik(const Eigen::VectorXd& start_config, const Eigen::Vector3d& tcp_target) {
  success_ = false;
  std::vector<Eigen::VectorXd> configs;

  ////////////////////////////////////////////////////////////////////////////////
  // Get Tcp at start
  ////////////////////////////////////////////////////////////////////////////////
  const auto maybe_tcp_start = solve_fk(start_config);
  if(!maybe_tcp_start.has_value()) {
    std::cout << "No FK for config " << start_config << std::endl;
    return configs;
  }

  const auto tcp_start = maybe_tcp_start.value();
  auto direction = (tcp_target - tcp_start);

  ////////////////////////////////////////////////////////////////////////////////
  //generate force into direction of target
  ////////////////////////////////////////////////////////////////////////////////

  float current_error = (tcp_target - tcp_start).norm();
  auto config_current(start_config);
  auto tcp_current(tcp_start);

  if(current_error <= Epsilon) {
    return configs;
  }
  AddConfig(start_config, configs);

  const auto body_tcp = skeleton_->getBodyNode(endeffector_);
  while(true) {
    ////////////////////////////////////////////////////////////////////////////////
    //Compute direction
    ////////////////////////////////////////////////////////////////////////////////
    auto J = body_tcp->getLinearJacobian();
    Eigen::MatrixXd pinv_J
        = J.transpose()
          * (J * J.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
                .inverse();
    auto direction = (tcp_target - tcp_current).normalized();
    auto dx = (pinv_J * direction).normalized();
    ////////////////////////////////////////////////////////////////////////////////
    //Move along cspace direction
    ////////////////////////////////////////////////////////////////////////////////
    config_current = config_current + kStepSize * dx;

    ////////////////////////////////////////////////////////////////////////////////
    // Compute current tcp position
    ////////////////////////////////////////////////////////////////////////////////
    auto maybe_fk = solve_fk(config_current);
    if(!maybe_fk.has_value()) {
      return configs;
    }
    tcp_current = maybe_fk.value();

    ////////////////////////////////////////////////////////////////////////////////
    // Terminate when error increases again
    ////////////////////////////////////////////////////////////////////////////////
    auto new_error = (tcp_target - tcp_current).norm();

    if(new_error < Epsilon) {
      AddConfig(config_current, configs);
      break;
    }
    if(new_error > current_error) {
      break;
    }
    current_error = new_error;
    AddConfig(config_current, configs);

  }
  // std::cout << "Goal frame " << tcp_current << " Desired: "<< tcp_target << std::endl;
  success_ = true;
  return configs;
}
bool KinematicsSolver::lastSolveWasSuccessful() {
  return success_;
}
