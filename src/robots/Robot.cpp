#include "robots/Robot.hpp"

#include "Common.hpp"

size_t Robot::GetDimension() const {
  return factor_->getStateDimension();
}

bool Robot::IsMultiRobot() const {
  return false;
}

const std::shared_ptr<dart::dynamics::Skeleton>& Robot::GetSkeleton() {
  return skeleton_;
}

void Robot::Hide() {
  for(const auto& body_node : skeleton_->getBodyNodes()) {
    auto shapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
    for(const auto& node : shapeNodes) {
      auto properties(node->getVisualAspect()->getProperties());
      properties.mHidden = true;
      node->getVisualAspect()->setProperties(properties);
    }
  }
}

void Robot::Show() {
  for(const auto& body_node : skeleton_->getBodyNodes()) {
    auto shapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
    for(const auto& node : shapeNodes) {
      auto properties(node->getVisualAspect()->getProperties());
      properties.mHidden = false;
      node->getVisualAspect()->setProperties(properties);
    }
  }
}

const std::shared_ptr<ompl::multilevel::FactoredSpaceInformation>& Robot::GetSpaceInformation() {
  return factor_;
}

const std::shared_ptr<CollisionChecker>& Robot::GetCollisionChecker() {
  return collision_checker_;
}

void Robot::SetSkeleton(const dart::dynamics::SkeletonPtr& skeleton) {
  skeleton_ = skeleton;
}

void Robot::SetSpaceInformation(const ompl::multilevel::FactoredSpaceInformationPtr& factor) {
  factor_ = factor;
}

void Robot::SetCollisionChecker(const CollisionCheckerPtr& collision_checker) {
  collision_checker_ = collision_checker;
}

float Robot::StateToTime(const ompl::base::State* /*state*/) const {
  return -1.0f;
}
void Robot::TimeToState(const float /*time*/, ompl::base::State* /*state*/) const {
}

std::vector<State3d> Robot::GetFK(const ompl::base::State* state) const {
  return GetFK(StateToEigen(state));
}

std::vector<State3d> Robot::GetFK(const StateXd& config) const {
  auto endeffector = skeleton_->getBodyNode(skeleton_->getNumBodyNodes() - 1)->getName();
  skeleton_->setConfiguration(config.configuration);
  return std::vector<State3d>({skeleton_->getBodyNode(endeffector)->getTransform().translation()});
}

std::string Robot::GetName() const {
  return factor_->getName();
}

void Robot::SetConfiguration(const StateXd& config) {
  skeleton_->setConfiguration(config.configuration);
}


bool Robot::HasValidJointLimits(const Eigen::VectorXd& config) const {
  auto lb = skeleton_->getPositionLowerLimits();
  auto ub = skeleton_->getPositionUpperLimits();
  for(size_t k = 0; k < (size_t) config.size(); k++) {
    const auto& value = config[k];
    if(!std::isfinite(value) || value < lb[k] || value > ub[k]) {
      //OMPL_WARN("Out of limits: %f is not in [%f, %f] (index %d).", value, lb[k], ub[k], k);
      return false;
    }
  }
  return true;
}

bool Robot::IsValid(const ompl::base::State* state) const {
  const auto eigen_state = this->StateToEigen(state);

  if(!HasValidJointLimits(eigen_state.configuration)) {
    return false;
  }

  const auto time = eigen_state.time;
  for(const auto& [obstacle, path] : dynamic_obstacles_) {
    auto config = path->GetConfigAt(time);
    obstacle->SetConfiguration(config);
  }

  skeleton_->setConfiguration(eigen_state.configuration);
  return !collision_checker_->IsInCollision();
}

CollisionCheckerPtr Robot::MakeCollisionChecker(
    const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
    const dart::simulation::WorldPtr& world,
    const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {

  std::vector<dart::dynamics::SkeletonPtr> collision_group_robot = {GetSkeleton()};
  auto collision_checker = std::make_shared<CollisionChecker>(world, collision_group_robot, obstacles);

  auto func = [&](const ompl::base::State* state) -> bool
  {
    return IsValid(state);
  };
  factor->setStateValidityChecker(func);
  factor->setStateValidityCheckingResolution(0.001);
  return collision_checker;
}

ompl::base::MotionValidatorPtr Robot::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr&) {
  return factor->getMotionValidator();
}

void Robot::AddDynamicalObstacle(const std::pair<RobotPtr, ompl::base::PathPtr>& obstacle) {
  auto path = std::make_shared<EigenPath>(obstacle.first, obstacle.second);
  dynamic_obstacles_.push_back(std::make_pair(obstacle.first, path));
}

void Robot::EnabledSmoothPath() {
  smooth_path_ = true;
}
void Robot::DisableSmoothPath() {
  smooth_path_ = false;
}
bool Robot::ShouldSmoothPath() const {
  return smooth_path_;
}
void Robot::EnabledShowPath() {
  show_path_ = true;
}
void Robot::DisableShowPath() {
  show_path_ = false;
}
bool Robot::ShouldShowPath() const {
  return show_path_;
}
