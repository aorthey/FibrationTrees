#include "robots/TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints.hpp"

bool TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints::IsValid(const ompl::base::State* state) const {
  auto config = StateToEigen(state);
  auto time = config.time;
  auto position = time / GetTMax();

  for(const auto& [obstacle, path] : obstacles_) {
    auto config = path->GetConfigAt(position);
    obstacle->SetConfiguration(config);
  }
  return Robot::IsValid(state);
}

void TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints::AddDynamicalObstacle(const std::pair<RobotPtr, ompl::base::PathPtr>& obstacle) {
  auto path = std::make_shared<EigenPath>(obstacle.first, obstacle.second);
  obstacles_.push_back(std::make_pair(obstacle.first, path));
}
