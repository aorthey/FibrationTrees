#include "CollisionCheckerHelper.hpp"

std::vector<RobotPtr> CollectAtomicRobots(const std::shared_ptr<MultiRobot>& multi_robot) {
  std::vector<RobotPtr> atomic_robots;

  const auto& robots = multi_robot->GetSubRobots();
  for(const auto& robot : robots) {
    if(robot->IsMultiRobot()) {
      //If robot is multi-robot, add all sub-robots
      auto mrobot = std::static_pointer_cast<MultiRobot>(robot);
      auto new_robots = CollectAtomicRobots(mrobot);
      atomic_robots.insert(atomic_robots.end(), new_robots.begin(), new_robots.end());
    } else {
      atomic_robots.push_back(robot);
    }
  }
  return atomic_robots;
}

