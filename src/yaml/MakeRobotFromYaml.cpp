#include "yaml/MakeRobotFromYaml.hpp"

#include <ompl/util/Console.h>

#include "yaml/MakeFromYamlHelpers.hpp"

#include "DartHelper.hpp"
#include "ToString.hpp"

#include "robots/SphereRobot.hpp"
#include "robots/MobileCar.hpp"
#include "robots/MobileCarDisk.hpp"
#include "robots/DiskRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/ZeppelinRobot.hpp"
#include "robots/ZeppelinInnerSphereRobot.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"

RobotPtr MakeAnyRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles,
    std::unordered_map<std::string, RobotPtr> child_robots) {
  if(node["name"].as<std::string>() == "MultiRobot") {
    return MakeMultiRobotFromNode(node, world, obstacles, child_robots);
  } else {
    return MakeAtomicRobotFromNode(node, world, obstacles);
  }
}

RobotPtr MakeMultiRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles,
    std::unordered_map<std::string, RobotPtr> child_robots) {

  if(!node["name"]) {
    throw std::domain_error("No name for multi robot.");
  }
  if(node["name"].as<std::string>() != "MultiRobot") {
    throw std::domain_error("Not a MultiRobot");
  }

  auto robot_names = node["robots"].as<std::vector<std::string>>();
  std::vector<RobotPtr> robots_for_multirobot;
  for(const auto& robot_name : robot_names) {
    auto findit = child_robots.find(robot_name);
    if (findit == child_robots.end()) {
      OMPL_WARN("Robot name not existing: %s", robot_name.c_str());
      throw std::out_of_range("Not a valid robot.");
    }else {
      robots_for_multirobot.push_back(findit->second);
    }
  }
  OMPL_DEBUG(">> Create multi robot from robots: %s", ToString(robot_names).c_str());

  return MultiRobot::MakeMultiRobot(robots_for_multirobot);
}

RobotPtr MakeAtomicRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  std::string name = node["name"].as<std::string>();
  RobotPtr robot;

  if(name == "KukaRobotTaskSpace") {
    robot = MakeRobot<KukaRobotTaskSpace>(world, obstacles);

  } else if(name == "MobileKukaRobot") {
    robot = MakeRobot<MobileKukaRobot>(world, obstacles);
    // auto lower_limit = node["lower_limits"].as<std::vector<double>>();
    // auto upper_limit = node["upper_limits"].as<std::vector<double>>();
    // robot->GetSkeleton()->setPositionLowerLimits(MakeState(lower_limit).configuration);
    // robot->GetSkeleton()->setPositionUpperLimits(MakeState(upper_limit).configuration);

  } else if(name == "MobileKukaRobotTaskSpace") {
    robot = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles);

  } else if(name == "TimeBasedMobileKukaRobotTaskSpace") {
    robot = MakeRobot<TimeBasedMobileKukaRobotTaskSpace>(world, obstacles);
    if(node["max_velocity"]) {
      std::static_pointer_cast<TimeBasedMobileKukaRobotTaskSpace>(robot)->SetVMax(node["max_velocity"].as<double>());
    }
    if(node["max_time"]) {
      std::static_pointer_cast<TimeBasedMobileKukaRobotTaskSpace>(robot)->SetTMax(node["max_time"].as<double>());
    }
  } else if(name == "TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints") {
    robot = MakeRobot<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(world, obstacles);
    if(node["max_velocity"]) {
      std::static_pointer_cast<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(robot)->SetVMax(node["max_velocity"].as<double>());
    }
    if(node["max_time"]) {
      std::static_pointer_cast<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(robot)->SetTMax(node["max_time"].as<double>());
    }

  } else if(name == "MobileCar") {
    robot = MakeRobot<MobileCar>(world, obstacles);

  } else if(name == "MobileCarDisk") {
    robot = MakeRobot<MobileCar>(world, obstacles);

  } else if(name == "ZeppelinRobot") {
    robot = MakeRobot<ZeppelinRobot>(world, obstacles);

  } else if(name == "ZeppelinInnerSphereRobot") {
    robot = MakeRobot<ZeppelinInnerSphereRobot>(world, obstacles);

  } else if(name == "DiskRobot") {
    robot = MakeRobot<DiskRobot>(world, obstacles);
    auto lower_limit = node["lower_limits"].as<std::vector<double>>();
    auto upper_limit = node["upper_limits"].as<std::vector<double>>();
    const auto task_space_limits = std::make_pair(MakeState2d(lower_limit), MakeState2d(upper_limit));
    std::static_pointer_cast<DiskRobot>(robot)->SetLimits(task_space_limits);

  } else if(name == "SphereRobot") {
    robot = MakeRobot<SphereRobot>(world, obstacles);
    auto lower_limit = node["lower_limits"].as<std::vector<double>>();
    auto upper_limit = node["upper_limits"].as<std::vector<double>>();
    const auto task_space_limits = std::make_pair(MakeState3d(lower_limit), MakeState3d(upper_limit));
    std::static_pointer_cast<SphereRobot>(robot)->SetLimits(task_space_limits);
  } else {
    OMPL_ERROR("No robot with name %s available.", name.c_str());
    throw std::out_of_range("No robot with name");
  }

  if(node["translation"]) {
    auto translation = node["translation"].as<std::vector<double>>();
    Eigen::Isometry3d transform(Eigen::Isometry3d::Identity());
    transform.translation() = MakeState3d(translation);
    robot->GetSkeleton()->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(transform);
  }

  MaybeChangeColor(node, robot->GetSkeleton());
  if(node["smooth_path"]) {
    auto smooth_path = node["smooth_path"].as<bool>();
    if(smooth_path) {
      robot->EnabledSmoothPath();
    } else {
      robot->DisableSmoothPath();
    }
  }
  if(node["show_path"]) {
    auto show_path = node["show_path"].as<bool>();
    if(show_path) {
      robot->EnabledShowPath();
    } else {
      robot->DisableShowPath();
    }
  }
  return robot;
}

std::unordered_map<std::string, RobotPtr> MakeChildRobotsFromYamlFilename(const std::string& filename, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {

  std::unordered_map<std::string, RobotPtr> robots;
  YAML::Node config = YAML::LoadFile(filename);
  const auto yaml_robots = config["robots"];

  //Make all atomic robots
  for(const auto& yaml_robot : yaml_robots) {
    const auto node = yaml_robot.second;

    if(node["root"]) {
      if(node["root"].as<bool>()) {
        continue;
      }
    }

    RobotPtr robot;

    if(node["name"]) {
      if(node["name"].as<std::string>() == "MultiRobot") {
        continue;
      }
    }

    OMPL_DEBUG(">> Create robot %s", node["name"].as<std::string>().c_str());
    robot = MakeAtomicRobotFromNode(node, world, obstacles);

    if(node["hide"]) {
      if(node["hide"].as<bool>()) {
        hide(robot->GetSkeleton());
      }
    }

    const auto key = yaml_robot.first.as<std::string>();
    robots[key] = robot;
  }

  //Make all multi robots
  for(const auto& yaml_robot : yaml_robots) {
    const auto node = yaml_robot.second;

    if(node["root"]) {
      if(node["root"].as<bool>()) {
        continue;
      }
    }

    if(node["name"]) {
      if(node["name"].as<std::string>() != "MultiRobot") {
        continue;
      }
    }

    RobotPtr robot = MakeMultiRobotFromNode(node, world, obstacles, robots);

    const auto key = yaml_robot.first.as<std::string>();
    robots[key] = robot;
  }

  return robots;
}

RobotPtr MakeRootRobotFromYamlFilename(const std::string& filename,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles, 
    std::unordered_map<std::string, RobotPtr> child_robots) {

  YAML::Node config = YAML::LoadFile(filename);
  const auto yaml_robots = config["robots"];

  RobotPtr root_robot;
  bool found_root_robot = false;
  for(const auto& yaml_robot : yaml_robots) {
    auto node = yaml_robot.second;
    if(node["root"]) {
      if(node["root"].as<bool>()) {
        if(found_root_robot) {
          OMPL_ERROR("Multiple robots declared as root in yaml file.");
          throw std::out_of_range("Multiple robots declared as root in yaml file.");
        }
        if(!node["name"]) {
          OMPL_ERROR("Root robot has no name.");
          throw std::domain_error("No valid name.");
        }
        root_robot = MakeAnyRobotFromNode(node, world, obstacles, child_robots);
        found_root_robot = true;
      }
    }
  }
  if(!found_root_robot) {
    OMPL_ERROR("Requires at least one root robot in yaml file.");
    throw std::out_of_range("Requires at least one root robot in yaml file.");
  }
  return root_robot;
}

std::string GetRootRobotNameFromYamlFilename(const std::string& filename) {

  YAML::Node config = YAML::LoadFile(filename);
  const auto yaml_robots = config["robots"];

  for(const auto& yaml_robot : yaml_robots) {
    auto node = yaml_robot.second;
    if(node["root"]) {
      if(node["root"].as<bool>()) {
        return yaml_robot.first.as<std::string>();
      }
    }
  }
  OMPL_ERROR("Requires at least one root robot in yaml file.");
  throw std::out_of_range("Requires at least one root robot in yaml file.");
}


