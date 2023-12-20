#include "KinematicsSolver.hpp"
#include "Common.hpp"

const float kStepSize = 0.5;
// const bool kDebugInfo = false;
const bool kDebugInfo = false;

struct State {
  Eigen::VectorXd config;
  Eigen::Vector3d tcp;
  float error{0.0f};
};

float ComputeError(const State& start, const State& goal) {
  return (goal.tcp - start.tcp).norm() + (goal.config - start.config).norm();
}

Eigen::MatrixXd ComputePseudoInverse(const Eigen::MatrixXd& J) {
  return J.completeOrthogonalDecomposition().pseudoInverse();
}

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
      if(kDebugInfo) std::cout << "Out of limits of " << k << "-th dof : " << lb[k] << "<" << config[k] << "<" << ub[k] << std::endl;
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


std::vector<Eigen::VectorXd> KinematicsSolver::solve_edge_ik(const Eigen::VectorXd& start_config, const Eigen::Vector3d& goal_tcp) {
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

  const auto start_tcp = maybe_tcp_start.value();
  auto direction = (goal_tcp - start_tcp);

  ////////////////////////////////////////////////////////////////////////////////
  //generate force into direction of target
  ////////////////////////////////////////////////////////////////////////////////

  float current_error = (goal_tcp - start_tcp).norm();
  auto current_config(start_config);
  auto current_tcp(start_tcp);

  if(current_error <= Epsilon) {
    return configs;
  }
  AddConfig(start_config, configs);

  const auto body_tcp = skeleton_->getBodyNode(endeffector_);
  skeleton_->setConfiguration(current_config);

  const size_t kMaxNonIncreasingIterations = 100;
  size_t non_increasing_iteration = 0;
  auto step_size = kStepSize;
  while(true) {
    ////////////////////////////////////////////////////////////////////////////////
    //Compute direction
    ////////////////////////////////////////////////////////////////////////////////
    const auto J = body_tcp->getLinearJacobian();
    const auto J_T = J.transpose();
    const auto J_PI = ComputePseudoInverse(J);
    const auto direction_task_space = (goal_tcp - current_tcp);

    auto dx = J_PI * direction_task_space;
    ////////////////////////////////////////////////////////////////////////////////
    //Move along cspace direction
    ////////////////////////////////////////////////////////////////////////////////
    current_config = current_config + step_size * dx;

    ////////////////////////////////////////////////////////////////////////////////
    // Compute current tcp position
    ////////////////////////////////////////////////////////////////////////////////
    auto maybe_fk = solve_fk(current_config);
    if(!maybe_fk.has_value()) {
      return configs;
    }
    current_tcp = maybe_fk.value();

    ////////////////////////////////////////////////////////////////////////////////
    // Terminate when error increases again
    ////////////////////////////////////////////////////////////////////////////////
    auto new_error = (goal_tcp - current_tcp).norm();

    if(new_error < Epsilon) {
      AddConfig(current_config, configs);
      break;
    }
    if(new_error >= current_error || std::abs(new_error - current_error) < Epsilon) {
      step_size *= 0.5;
      if(non_increasing_iteration++ >= kMaxNonIncreasingIterations) {
        break;
      }
    }
    current_error = new_error;
    AddConfig(current_config, configs);

  }
  success_ = true;
  return configs;
}

Eigen::VectorXd KinematicsSolver::ForwardStep(const Eigen::VectorXd& config, const Eigen::VectorXd& dx) const {
  Eigen::VectorXd next = config + kStepSize * dx;
  const auto& lb = skeleton_->getPositionLowerLimits();
  const auto& ub = skeleton_->getPositionUpperLimits();
  for(size_t k = 0; k < next.size(); k++) {
    if(next[k] < lb[k])
    {
      next(k) = lb(k);
    }
    if(next[k] > ub[k])
    {
      next(k) = ub[k];
    }
  }
  return next;
}

std::vector<Eigen::VectorXd> KinematicsSolver::solve_edge_ik_with_config(const Eigen::VectorXd& start_config, const Eigen::VectorXd& goal_config) {
  success_ = false;
  std::vector<Eigen::VectorXd> configs;

  ////////////////////////////////////////////////////////////////////////////////
  // Get Tcp at start and goal
  ////////////////////////////////////////////////////////////////////////////////
  const auto maybe_tcp_start = solve_fk(start_config);
  if(!maybe_tcp_start.has_value()) {
    if(kDebugInfo) std::cout << "No FK for config " << start_config << std::endl;
    return configs;
  }

  const auto maybe_tcp_goal = solve_fk(goal_config);
  if(!maybe_tcp_goal.has_value()) {
    if(kDebugInfo) std::cout << "No FK for config " << goal_config << std::endl;
    return configs;
  }

  const State start{start_config, maybe_tcp_start.value()};
  const State goal{goal_config, maybe_tcp_goal.value()};

  if(kDebugInfo) {
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Solving Edge IK" << std::endl;
    std::cout << std::string(80, '-') << std::endl;
    std::cout << "Start config: " << start.config.format(CommaFmt) << std::endl;
    std::cout << "Goal  config: " << goal.config.format(CommaFmt) << std::endl;
    std::cout << "Start tcp   : " << start.tcp.format(CommaFmt) << std::endl;
    std::cout << "Goal  tcp   : " << goal.tcp.format(CommaFmt) << std::endl;
  }
  ////////////////////////////////////////////////////////////////////////////////
  //generate force into direction of target
  ////////////////////////////////////////////////////////////////////////////////

  State current{start};

  current.error = ComputeError(start, goal);
  if(current.error <= Epsilon) {
    return configs;
  }

  AddConfig(current.config, configs);

  const auto body_tcp = skeleton_->getBodyNode(endeffector_);
  skeleton_->setConfiguration(current.config);

  const size_t kMaxNonIncreasingIterations = 100;
  size_t non_increasing_iteration = 0;
  while(true) {
    ////////////////////////////////////////////////////////////////////////////////
    //Compute direction
    ////////////////////////////////////////////////////////////////////////////////

    const auto J = body_tcp->getLinearJacobian();
    const auto J_T = J.transpose();
    const auto J_PI = ComputePseudoInverse(J);
    const auto direction_task_space = (goal.tcp - current.tcp);

    // auto dx = J_PI * direction_task_space;

    //Add joint config force in nullspace
    const auto direction_joint_space = (goal.config - current.config);
    const auto I = Eigen::MatrixXd::Identity(J.cols(), J.cols());
    auto dx = (J_PI * direction_task_space + (I - J_T * J_PI.transpose()) * direction_joint_space);

    ////////////////////////////////////////////////////////////////////////////////
    //Move along cspace direction and create a new state
    ////////////////////////////////////////////////////////////////////////////////
    State next;
    next.config = ForwardStep(current.config, dx);

    auto maybe_fk = solve_fk(next.config);
    if(!maybe_fk.has_value()) {
      if(kDebugInfo) std::cout << "Could not solve FK for " << current.config.format(CommaFmt) << std::endl;
      return configs;
    }
    next.tcp = maybe_fk.value();
    next.error = ComputeError(next, goal);

    ////////////////////////////////////////////////////////////////////////////////
    // Terminate when error increases again
    ////////////////////////////////////////////////////////////////////////////////

    if(next.error < Epsilon) {
      if(kDebugInfo) std::cout << "Error below threshold " << next.error << ". Terminating." << std::endl;
      AddConfig(next.config, configs);
      break;
    }
    if(next.error >= current.error || std::abs(next.error - current.error) < Epsilon) {
      if(non_increasing_iteration++ >= kMaxNonIncreasingIterations) {
        if(kDebugInfo) {
          std::cout << "Error not decreasing: " << next.error << ">=" << current.error << " for " 
          << non_increasing_iteration << " iterations. Terminating." << std::endl;
          std::cout << " Start was         : " << start_config.format(CommaFmt) << std::endl;
          std::cout << " Goal to reach was : " << goal_config.format(CommaFmt) << std::endl;
          std::cout << " Last dx was       : " << dx.format(CommaFmt) << std::endl;
        }
        break;
      }
    }

    if(kDebugInfo) {
      std::cout << "Current error: " << next.error << " at Tcp " << next.tcp.format(CommaFmt) << " and config " << next.config.format(CommaFmt) << std::endl;
    }

    current = next;
    AddConfig(next.config, configs);
  }

  success_ = true;
  return configs;
}

bool KinematicsSolver::lastSolveWasSuccessful() {
  return success_;
}
