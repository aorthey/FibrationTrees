#include "yaml/MakeRobotFromYaml.hpp"

#include <ompl/util/Console.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>

#include "yaml/MakeFromYamlHelpers.hpp"

#include "DartHelper.hpp"
#include "ToString.hpp"

#include "robots/Drone.hpp"
#include "robots/DiskRobot.hpp"
#include "robots/CubeRobot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/MobileCar.hpp"
#include "robots/MobileCarDisk.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/ZeppelinRobot.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"

RobotPtr MakeAnyRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles,
    std::unordered_map<std::string, RobotPtr> child_robots) {
  if(node["name"].as<std::string>() == "MultiRobot") {
    return MakeMultiRobotFromNode(node, world, obstacles, child_robots);
  } else {
    return MakeAtomicRobotFromNode(node, world, obstacles);
  }
}

bool CanMakeMultiRobotFromNode(const YAML::Node& node,
    std::unordered_map<std::string, RobotPtr> child_robots) {

  if(!node["robots"]) {
    throw std::domain_error("Requires yaml field called robots.");
  }
  auto robot_names = node["robots"].as<std::vector<std::string>>();

  return std::all_of(robot_names.begin(), robot_names.end(), [&](const std::string& robot_name) {
    auto findit = child_robots.find(robot_name);
    return (findit != child_robots.end());
  });
}

RobotPtr MakeMultiRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>&,
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

  auto multi_robot = MultiRobot::MakeMultiRobot(robots_for_multirobot);
  OMPL_DEBUG(">> Create multi robot %s from robots: %s", multi_robot->GetName().c_str(), ToString(robot_names).c_str());

  auto si = multi_robot->GetSpaceInformation();

  auto pairwise_collision_checker = std::make_shared<MultiRobotCollisionChecker>(world, multi_robot);
  si->setStateValidityChecker(pairwise_collision_checker);
  si->setStateValidityCheckingResolution(0.001);

  bool task_space_robot = false;
  if(node["task_space"]) {
    if(node["task_space"].as<bool>()) {
      auto motion_validator = std::make_shared<MotionValidatorTaskSpaceMultiRobot>(si, multi_robot);
      si->setMotionValidator(motion_validator);
      task_space_robot = true;
    }
  }

  if(!task_space_robot) {
    //If the multi robot does not interpolate in task space, then all subrobots
    //should also not interpolate in task space
    for(const auto& robot : robots_for_multirobot) {
      auto motion_validator = robot->GetSpaceInformation()->getMotionValidator();
      if(std::dynamic_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(motion_validator) != nullptr) {
        throw std::runtime_error("Multi robot has no task space interpolation, but subrobot " + robot->GetName() + " has.");
      }
    }
  }


  return multi_robot;
}

RobotPtr MakeAtomicRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  std::string name = node["name"].as<std::string>();
  RobotPtr robot;

  if(name == "KukaRobotTaskSpace") {
    robot = MakeRobot<KukaRobotTaskSpace>(world, obstacles, node);
  } else if(name == "MobileKukaRobotTaskSpace") {
    robot = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles, node);
  } else if(name == "MobileKukaRobot") {
    robot = MakeRobot<MobileKukaRobot>(world, obstacles, node);
  } else if(name == "MobileKukaBase") {
    robot = MakeRobot<MobileKukaBase>(world, obstacles, node);

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
    robot = MakeRobot<MobileCar>(world, obstacles, node);

  } else if(name == "MobileCarDisk") {
    robot = MakeRobot<MobileCar>(world, obstacles, node);

  } else if(name == "ZeppelinRobot") {
    robot = MakeRobot<ZeppelinRobot>(world, obstacles, node);

  } else if(name == "CubeRobot") {
    robot = MakeRobot<CubeRobot>(world, obstacles, node);

  } else if(name == "DiskRobot") {
    robot = MakeRobot<DiskRobot>(world, obstacles, node);

  } else if(name == "SphereRobot") {
    robot = MakeRobot<SphereRobot>(world, obstacles, node);

  } else if(name == "Drone") {
    robot = MakeRobot<Drone>(world, obstacles, node);
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

void CheckSingletonRootRobot(const YAML::Node& yaml_robots) {
  size_t number_of_root_robots = 0;
  for(const auto& yaml_robot : yaml_robots) {
    const auto node = yaml_robot.second;

    if(node["root"]) {
      if(node["root"].as<bool>()) {
        number_of_root_robots++;
      }
    }
  }
  if(number_of_root_robots != 1) {
    throw std::out_of_range("Requires exactly one root robot, but yaml has " + std::to_string(number_of_root_robots));
  }
}

std::unordered_map<std::string, RobotPtr> MakeChildRobotsFromYamlFilename(const std::string& filename, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {

  std::unordered_map<std::string, RobotPtr> robots;
  YAML::Node config = YAML::LoadFile(filename);
  const auto yaml_robots = config["robots"];

  CheckSingletonRootRobot(yaml_robots);

  //Make all atomic robots
  for(const auto& yaml_robot : yaml_robots) {
    const auto node = yaml_robot.second;

    if(node["root"]) {
      if(node["root"].as<bool>()) {
        continue;
      }
    }

    if(node["name"]) {
      if(node["name"].as<std::string>() == "MultiRobot") {
        continue;
      }
    }

    auto robot = MakeAtomicRobotFromNode(node, world, obstacles);

    if(node["hide"]) {
      if(node["hide"].as<bool>()) {
        hide(robot->GetSkeleton());
      }
    }

    const auto key = yaml_robot.first.as<std::string>();
    robots[key] = robot;
    OMPL_DEBUG(">> Create robot %s: %s", key.c_str(), node["name"].as<std::string>().c_str());
  }

  //Make all multi robots
  while(true) {
    bool added_new_robot = false;
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
      const auto key = yaml_robot.first.as<std::string>();
      if(robots.contains(key)) {
        continue;
      }

      if(!CanMakeMultiRobotFromNode(node, robots)) {
        continue;
      }

      RobotPtr robot = MakeMultiRobotFromNode(node, world, obstacles, robots);
      robots[key] = robot;
      added_new_robot = true;
    }
    if(!added_new_robot) {
      break;
    }
  }

  //Verify that all robots have been created.
  std::vector<std::string> uncreated_robots;
  for(const auto& yaml_robot : yaml_robots) {
    const auto node = yaml_robot.second;
    if(node["root"]) {
      if(node["root"].as<bool>()) {
        continue;
      }
    }
    const auto key = yaml_robot.first.as<std::string>();
    if(!robots.contains(key)) {
      uncreated_robots.push_back(key);
    }
  }
  if(!uncreated_robots.empty()) {
    throw std::runtime_error("Could not load robots " + ToString(uncreated_robots));
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

