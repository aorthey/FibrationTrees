#include "KinematicsSolver.hpp"
#include "Common.hpp"

const float kStepSize = 0.001;

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

// std::optional<Eigen::VectorXd> KinematicsSolver::solve_seed_ik(const Eigen::Vector3d& frame, const Eigen::VectorXd& seed) {
//   skeleton_->setConfiguration(config);
//   auto ik = skeleton_->getBodyNode(endeffector_)->getOrCreateIK();
//   ik->getTarget()->setTranslation(frame);

//   if(ik->solveAndApply(seed, true))
//   {
//     return ik->getPositions();
//   }
//   return std::nullopt;
// }

std::optional<Eigen::Vector3d> KinematicsSolver::solve_fk(const Eigen::VectorXd& config) {
  auto lb = skeleton_->getPositionLowerLimits();
  auto ub = skeleton_->getPositionUpperLimits();
  for(size_t k = 0; k < config.size(); k++) {
    if(config[k] < lb[k] || config[k] > ub[k] || config[k] != config[k]) {
      // std::cout << "Out of limits: " << lb[k] << "<" << config[k] << "<" << ub[k] << std::endl;
      return std::nullopt;
    }
  }
  skeleton_->setConfiguration(config);
  return skeleton_->getBodyNode(endeffector_)->getTransform().translation();
}

//Avoid duplicates, add additional configs during joint flips
void AddConfig(const Eigen::VectorXd& config, std::vector<Eigen::VectorXd>& configs) {
  // configs.push_back(config);
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

  const float kAddStepSize = 0.1;
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


//Move along the straight line segment between c1 and c2
const float kAccuracy = 1e-2;
std::vector<Eigen::VectorXd> KinematicsSolver::solve_edge_ik(const Eigen::VectorXd& start_config, const Eigen::Vector3d& tcp_target) {

  success_ = false;
  std::vector<Eigen::VectorXd> configs;

  ////////////////////////////////////////////////////////////////////////////////
  // Get Tcp at start
  ////////////////////////////////////////////////////////////////////////////////
  const auto maybe_tcp_start = solve_fk(start_config);
  if(!maybe_tcp_start.has_value()) {
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

  // std::cout << "SOLVE EDGE IK with initial error " << current_error << std::endl;
  if(current_error <= Epsilon) {
    return configs;
  }
  AddConfig(start_config, configs);

  // std::cout << "From config " << start_config << " (frame " << tcp_start << " to frame " << tcp_target << ")" << std::endl;
  Eigen::MatrixXd M = skeleton_->getMassMatrix();
  M.setIdentity();
  for(size_t k = 0; k<6; k++) {
    M(k,k) = 0.0f;
    M(k,k) = 0.0f;
  }

  const auto body_tcp = skeleton_->getBodyNode(endeffector_);
  while(true) {
    ////////////////////////////////////////////////////////////////////////////////
    //Compute direction
    ////////////////////////////////////////////////////////////////////////////////
    // auto direction = (tcp_target - tcp_current).normalized();
    auto J = body_tcp->getLinearJacobian();
    Eigen::MatrixXd pinv_J
        = J.transpose()
          * (J * J.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
                .inverse();
    auto direction = (tcp_target - tcp_current).normalized();
    auto dx = (M * pinv_J * direction).normalized();
    ////////////////////////////////////////////////////////////////////////////////
    //Move along cspace direction
    ////////////////////////////////////////////////////////////////////////////////
    config_current = config_current + kStepSize * dx;
    for(size_t k = 0; k<6; k++) {
      config_current[k] = 0.0f;
    }
    ////////////////////////////////////////////////////////////////////////////////
    // Compute current tcp position
    ////////////////////////////////////////////////////////////////////////////////
    auto maybe_fk = solve_fk(config_current);
    if(!maybe_fk.has_value()) {
      std::cout << "Could not find FK for " << config_current << std::endl;
      return configs;
    }
    tcp_current = maybe_fk.value();

    ////////////////////////////////////////////////////////////////////////////////
    // Terminate when error increases again
    ////////////////////////////////////////////////////////////////////////////////
    auto new_error = (tcp_target - tcp_current).norm();

    if(new_error < Epsilon) {
      // configs.push_back(config_current);
      AddConfig(config_current, configs);
      // std::cout << "Final error is " << new_error << std::endl;
      break;
    }
    if(new_error > current_error) {
      // std::cout << "Final error is " << current_error << std::endl;
      break;
    }
    current_error = new_error;
    // configs.push_back(config_current);
    AddConfig(config_current, configs);

  }
  // std::cout << "Goal frame " << tcp_current << " Desired: "<< tcp_target << std::endl;
  success_ = true;
  return configs;
}
bool KinematicsSolver::lastSolveWasSuccessful() {
  return success_;
}
//Move along the straight line segment between c1 and c2
//const float kAccuracy = 1e-2;
//std::vector<Eigen::VectorXd> KinematicsSolver::solve_edge_ik(const Eigen::VectorXd& start_config, const Eigen::Vector3d& tcp_target) {

//  success_ = false;
//  std::vector<Eigen::VectorXd> configs;

//  ////////////////////////////////////////////////////////////////////////////////
//  // Get Tcp at start
//  ////////////////////////////////////////////////////////////////////////////////
//  const auto maybe_tcp_start = solve_fk(start_config);
//  if(!maybe_tcp_start.has_value()) {
//    return configs;
//  }
//  configs.push_back(start_config);

//  const auto tcp_start = maybe_tcp_start.value();

//  const auto direction = (tcp_target - tcp_start);

//  const float kStepSize = 0.1;
//  float distance = 0.0f;

//  auto tcp_current(tcp_start);
//  auto config_current(start_config);

//  float error = (tcp_current - tcp_target).norm();

//  while(error > kAccuracy) {
//    Eigen::Vector3d direction = (tcp_target - tcp_current);

//    //generate force into direction
//    dart::math::LinearJacobian J = skeleton_->getBodyNode(endeffector_)->getLinearJacobian();
//    Eigen::MatrixXd pinv_J
//        = J.transpose()
//          * (J * J.transpose() + 0.0025 * Eigen::Matrix3d::Identity())
//                .inverse();
//    auto dx = (pinv_J * direction).normalized();

//    config_current = config_current + kStepSize * dx;
//    for(size_t k = 0; k<6; k++) {
//      config_current[k] = 0.0f;
//    }

//    // Compute current tcp position
//    auto maybe_fk = solve_fk(config_current);
//    if(!maybe_fk.has_value()) {
//      // std::cout << "Could not find FK for " << config_current << std::endl;
//      return configs;
//    }
//    tcp_current = maybe_fk.value();

//    configs.push_back(config_current);

//    error = (tcp_current - tcp_target).norm();
//    // std::cout << "Current error: "<< error << "/" << Epsilon << std::endl;
//  }
//  success_ = true;
//  return configs;
//}
//bool KinematicsSolver::lastSolveWasSuccessful() {
//  return success_;
//}
