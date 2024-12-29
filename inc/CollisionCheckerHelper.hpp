#pragma once

#include "robots/MultiRobot.hpp"
#include "robots/Robot.hpp"

std::vector<RobotPtr> CollectAtomicRobots(const std::shared_ptr<MultiRobot>& multi_robot);
