#include <ompl/base/State.h>
#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/goals/FactoredGoal.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/datastructures/projections/RNSO2_RN.h>
#include <ompl/multilevel/datastructures/Projection.h>
#include <ompl/multilevel/datastructures/projections/TimeBasedProjection.h>

#include <dart/dart.hpp>
#include "yaml-cpp/yaml.h"

#include "FilePath.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "TaskSpaceGoal.hpp"
#include "TimeGoal.hpp"
#include "robots/Robot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/DiskRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/ZeppelinRobot.hpp"
#include "robots/ZeppelinInnerSphereRobot.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints.hpp"
#include "projections/ProjectionTaskSpace.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"

////////////////////////////////////////////////////////////////////////////////
// Helper functions
////////////////////////////////////////////////////////////////////////////////
void MaybeChangeColor(const YAML::Node& node, const dart::dynamics::SkeletonPtr& skeleton) {
  if(!node["color"]) {
    return;
  }
  auto color_vector = node["color"].as<std::vector<double>>();

  if(color_vector.size() != 4) {
    throw std::out_of_range("Color needs to have four values, but has " + std::to_string(color_vector.size()));
  }
  const Eigen::Vector4d color(color_vector.data());
  changeBodyColor(skeleton, color);
}
////////////////////////////////////////////////////////////////////////////////
// Robots
////////////////////////////////////////////////////////////////////////////////
RobotPtr MakeRobotFromNode(const YAML::Node& node,
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  std::string name = node["name"].as<std::string>();
  RobotPtr robot;

  if(name == "KukaRobotTaskSpace") {
    robot = MakeRobot<KukaRobotTaskSpace>(world, obstacles);

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
  return robot;
}

std::unordered_map<std::string, RobotPtr> MakeChildRobotsFromYamlFilename(const std::string& filename, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {

  std::unordered_map<std::string, RobotPtr> robots;
  YAML::Node config = YAML::LoadFile(filename);
  const auto yaml_robots = config["robots"];
  for(const auto& yaml_robot : yaml_robots) {
    const auto node = yaml_robot.second;

    if(node["root"]) {
      if(node["root"].as<bool>()) {
        continue;
      }
    }

    RobotPtr robot = MakeRobotFromNode(node, world, obstacles);

    if(node["hide"]) {
      if(node["hide"].as<bool>()) {
        hide(robot->GetSkeleton());
      }
    }

    const auto key = yaml_robot.first.as<std::string>();
    OMPL_DEBUG("Create robot %s", key.c_str());
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
          throw std::out_of_range("No valid name.");
        }
        if(node["name"].as<std::string>() == "MultiRobot") {
          auto robot_names = node["robots"].as<std::vector<std::string>>();

          std::vector<RobotPtr> robots_for_multirobot;
          for(const auto& robot_name : robot_names) {
            auto findit = child_robots.find(robot_name);
            if (findit == child_robots.end()) {
              OMPL_WARN("Robot name not existing: %s", robot_name.c_str());
              throw std::out_of_range("Not a valid robot.");
            }else {
              OMPL_DEBUG("Found robot %s for multi robot.", robot_name.c_str());
              robots_for_multirobot.push_back(findit->second);
            }
          }

          root_robot = MultiRobot::MakeMultiRobot(robots_for_multirobot);
        } else {
          root_robot = MakeRobotFromNode(node, world, obstacles);
        }
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


////////////////////////////////////////////////////////////////////////////////
// Obstacles
////////////////////////////////////////////////////////////////////////////////

std::vector<dart::dynamics::SkeletonPtr> MakeObstaclesFromYamlFilename(const std::string& filename) {
  YAML::Node config = YAML::LoadFile(filename);

  std::vector<dart::dynamics::SkeletonPtr> obstacles;

  const auto yaml_obstacles = config["obstacles"];
  for(const auto& obstacle : yaml_obstacles) {
    auto node = obstacle.second;
    std::string name = node["name"].as<std::string>();
    std::string type = node["type"].as<std::string>();
    OMPL_INFORM("Creating obstacle %s from type %s.", name.c_str(), type.c_str());

    if(type == "robot") {
      //dynamic obstacle
      continue;
    }

    if(type == "floor") {
      if(node["height"] && node["width"]) {
        double height = node["height"].as<double>();
        double width = node["width"].as<double>();
        OMPL_INFORM("Create floor with height %f and width %f.", height, width);
        obstacles.push_back(createFloor(height, width));
      } else if(node["height"]) {
        double height = node["height"].as<double>();
        obstacles.push_back(createFloor(height));
      } else {
        obstacles.push_back(createFloor());
      }
      MaybeChangeColor(node, obstacles.back());
      continue;
    } else if(type == "urdf") {
      std::vector<double> position = node["position"].as<std::vector<double>>();
      std::string urdf_filename = node["filename"].as<std::string>();
      auto state = State3d(position.at(0), position.at(1), position.at(2));
      auto urdf = createFromURDF(GetDataFolder() + "objects/maze.urdf", state);
      MaybeChangeColor(node, urdf);
      obstacles.push_back(urdf);
      continue;
    } else if(type == "box") {
      std::vector<double> position = node["position"].as<std::vector<double>>();
      std::vector<double> size = node["size"].as<std::vector<double>>();
      auto state = State3d(position.at(0), position.at(1), position.at(2));
      auto box = createBox(state, size.at(0), size.at(1), size.at(2));
      MaybeChangeColor(node, box);
      obstacles.push_back(box);
      continue;
    } else if(type == "cylinder") {
      std::vector<double> position = node["position"].as<std::vector<double>>();
      std::vector<double> size = node["size"].as<std::vector<double>>();
      auto state = State3d(position.at(0), position.at(1), position.at(2));
      auto cylinder = createCylinder(state, size.at(0), size.at(1));
      MaybeChangeColor(node, cylinder);
      obstacles.push_back(cylinder);
      continue;
    } else {
      OMPL_ERROR("Could not create obstacles from type %s", type.c_str());
      throw std::invalid_argument("Unknown obstacle type:" + type);
    }
  }
  return obstacles;
}

std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> 
MakeDynamicObstaclesFromYamlFilename(const std::string& filename, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& static_obstacles) {

  YAML::Node config = YAML::LoadFile(filename);

  std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> dynamic_obstacles;
  const auto yaml_obstacles = config["obstacles"];
  for(const auto& obstacle : yaml_obstacles) {
    std::string name = obstacle.second["name"].as<std::string>();
    std::string type = obstacle.second["type"].as<std::string>();
    if(type != "robot") {
      continue;
    }

    if(!obstacle.second["start"]) {
      throw std::domain_error("Requires start values");
    }
    if(!obstacle.second["goal"]) {
      throw std::domain_error("Requires goal values");
    }
    if(!obstacle.second["time_start"]) {
      throw std::domain_error("Requires start time");
    }
    if(!obstacle.second["time_goal"]) {
      throw std::domain_error("Requires goal time");
    }

    auto robot = MakeRobotFromNode(obstacle.second, world, static_obstacles);

    auto config1 = obstacle.second["start"].as<std::vector<double>>();
    auto config2 = obstacle.second["goal"].as<std::vector<double>>();

    auto state1 = MakeState(config1);
    state1.time = obstacle.second["time_start"].as<double>();
    auto state2 = MakeState(config2);
    state2.time = obstacle.second["time_goal"].as<double>();

    auto si = robot->GetSpaceInformation();
    auto start = si->allocState();
    auto goal = si->allocState();

    robot->EigenToState(state1, start);
    robot->EigenToState(state2, goal);

    auto path = std::make_shared<ompl::geometric::PathGeometric>(si, start, goal);
    dynamic_obstacles.push_back(std::make_pair(robot, path));
  }

  return dynamic_obstacles;
}

////////////////////////////////////////////////////////////////////////////////
// ProblemDefinition
////////////////////////////////////////////////////////////////////////////////

ompl::base::GoalPtr MakeGoalRegionFromGoalNode(const YAML::Node& node, const ompl::base::SpaceInformationPtr& factor, 
    const RobotPtr& robot, const ompl::base::State* start, ompl::base::State* goal) {

  const auto goal_type = node["type"].as<std::string>();
  if(goal_type == "task") {
    auto goal_region = std::make_shared<ompl::base::GoalState>(factor);
    goal_region->setState(goal);
    const auto goal_threshold = node["threshold"].as<double>();
    goal_region->setThreshold(goal_threshold);
    return goal_region;
  } else if(goal_type == "time") {
    auto robot_in_time = std::static_pointer_cast<TimeBasedMobileKukaRobotTaskSpace>(robot);
    robot_in_time->TimeToState(robot_in_time->GetTMax(), goal);
    auto time_goal = std::make_shared<TimeGoal>(robot_in_time, robot_in_time->GetVMax(), start, goal);
    const auto goal_threshold = node["threshold"].as<double>();
    time_goal->setThreshold(0.5);
    return time_goal;
  } else {
    OMPL_ERROR("Goal type %s not found.", goal_type.c_str());
    throw std::invalid_argument("Unknown goal type");
  }
}

ompl::base::ProblemDefinitionPtr MakeProblemDefinitionFromYamlFilename(
    const std::string& filename, const dart::simulation::WorldPtr& world, 
    const ompl::multilevel::FactoredSpaceInformationPtr& root_factor,
    const RobotPtr& root_robot, const std::unordered_map<std::string, RobotPtr>& child_robots) {

  YAML::Node node = YAML::LoadFile(filename);
  OMPL_INFORM("Loading File %s", node["name"].as<std::string>().c_str());

  if(!node["problem"]) {
    OMPL_WARN("Requires a problem tag in %s file.", filename.c_str());
    return nullptr;
    //throw std::invalid_argument("No problem tag");
  }

  auto start_node = node["problem"]["start"];
  if(!start_node) {
    OMPL_ERROR("Requires a start tag in %s file.", filename.c_str());
    throw std::invalid_argument("No start tag");
  }
  auto goal_node = node["problem"]["goal"];
  if(!goal_node) {
    OMPL_ERROR("Requires a goal tag in %s file.", filename.c_str());
    throw std::invalid_argument("No goal tag");
  }

  const auto start_type = start_node["type"].as<std::string>();
  if(start_type == "task") {
    ////////////////////////////////////////////////////////////////////////////////
    // Compute start configuration by projecting states upwards
    ////////////////////////////////////////////////////////////////////////////////
    std::unordered_map<std::string, ompl::base::State*> start_child_states;
    for(const auto& robot_node : start_node) {
      auto name = robot_node.first.as<std::string>();
      if(name == "type" || name == "time") {
        continue;
      }
      auto config = robot_node.second["config"].as<std::vector<double>>();
      auto start_eigen = MakeState(config);

      //Get SpaceInformation for robot 
      if(root_robot->GetName() == name) {
        OMPL_WARN("Cannot specify a task start on the root space.");
        throw std::runtime_error("No root space allowed for task starts.");
      } else {
        auto robot = child_robots.at(name);
        auto si = robot->GetSpaceInformation();
        ompl::base::State *start_ompl_state = si->allocState();
        robot->EigenToState(start_eigen, start_ompl_state);
        start_child_states[si->getName()] = start_ompl_state;
        OMPL_INFORM("Start state:");
        si->printState(start_ompl_state);
      }

      //Maybe visualize
      if(robot_node.second["visualize"]) {
        auto visualize = robot_node.second["visualize"].as<bool>();
        if(visualize) {
          world->addSimpleFrame(createCylinderFrame(start_eigen.configuration, State3d(0.0, M_PI*0.5, 0.0), 0.01, 0.001, State4d(0.1, 0.5, 0.1, 0.5)));
        }
      }
    }

    auto maybe_start = ComputeValidTotalState(root_factor, start_child_states);
    OMPL_INFORM("Done");
    if(!maybe_start.has_value()){
      OMPL_ERROR("Could not compute valid start.");
      throw std::runtime_error("Could not compute valid start");
    }
    auto start = maybe_start.value();
    
    if(start_node["time"]) {
      root_robot->TimeToState(start_node["time"].as<double>(), start);
    }
    OMPL_INFORM("Found start state:");
    root_factor->printState(start);

    ////////////////////////////////////////////////////////////////////////////////
    // Compute goal configuration to ensure a goal exists
    ////////////////////////////////////////////////////////////////////////////////
    const auto goal_type = goal_node["type"].as<std::string>();

    if(!(goal_type == "task" || goal_type == "time")) {
      OMPL_ERROR("Goal type needs to be task for a task start type.");
      throw std::runtime_error("Goal type needs to be task for a task start type.");
    }

    std::unordered_map<std::string, ompl::base::State*> goal_child_states;
    std::unordered_map<std::string, std::string> space_name_to_robot_name;
    for(const auto& robot_node : goal_node) {
      auto name = robot_node.first.as<std::string>();
      if(name == "type" || name=="threshold") {
        continue;
      }
      auto config = robot_node.second["config"].as<std::vector<double>>();
      auto goal_eigen = MakeState(config);

      //Get SpaceInformation for robot 
      if(root_robot->GetName() == name) {
        OMPL_WARN("Cannot specify a task goal on the root space.");
        throw std::runtime_error("No root space allowed for task goal.");
      } else {
        if(child_robots.find(name) == child_robots.end()) {
          OMPL_ERROR("Could not find robot %s.", name.c_str());
          throw std::domain_error("Child robot " + name + " does not exist.");
        }
        auto robot = child_robots.at(name);
        auto si = robot->GetSpaceInformation();
        ompl::base::State *goal_ompl_state = si->allocState();
        robot->EigenToState(goal_eigen, goal_ompl_state);
        goal_child_states[si->getName()] = goal_ompl_state;
        OMPL_INFORM("Goal state:");
        si->printState(goal_ompl_state);
        space_name_to_robot_name[si->getName()] = name;
      }

      //Maybe visualize
      if(robot_node.second["visualize"]) {
        auto visualize = robot_node.second["visualize"].as<bool>();
        if(visualize) {
          world->addSimpleFrame(createCylinderFrame(goal_eigen.configuration, State3d(0.0, M_PI*0.5, 0.0), 0.01, 0.001, State4d(0.1, 0.5, 0.1, 0.5)));
        }
      }
    }

    auto maybe_goal = ComputeValidTotalState(root_factor, goal_child_states);
    if(!maybe_goal.has_value()){
      OMPL_ERROR("Could not compute valid goal.");
      throw std::runtime_error("Could not compute valid goal");
    }
    auto goal = maybe_goal.value();

    OMPL_INFORM("Found goal state:");
    root_factor->printState(goal);

    //////////////////////////////////////////////////////////////////////////////// 
    // Create child task space goals
    //////////////////////////////////////////////////////////////////////////////// 
    ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(root_factor);
    pdef->addStartState(start);

    // const auto use_goal_region = false;
    // if(use_goal_region) {
    //   OMPL_INFORM("Create goal region");
    //   std::unordered_map<std::string, ompl::base::GoalSampleableRegionPtr> goal_regions;

    //   for(const auto& goal_child_state : goal_child_states) {
    //     auto space_name = goal_child_state.first;
    //     auto robot_name = space_name_to_robot_name[space_name];
    //     auto state = goal_child_state.second;
    //     auto robot = child_robots.at(robot_name);
    //     auto factor = robot->GetSpaceInformation();
    //     auto projection = factor->getProjection();

    //     OMPL_INFORM("Task space goal for %s with space %s", robot_name.c_str(), factor->getName().c_str());
    //     auto goal_region = std::make_shared<TaskSpaceGoalBaseState>(factor, state, projection);
    //     OMPL_WARN("Make threshold from yaml");
    //     goal_region->setThreshold(0.1);
    //     goal_regions[factor->getName()] = goal_region;
    //   }
    //   OMPL_INFORM("Done");

    //   auto goal_region = std::make_shared<ompl::base::FactoredGoal>(root_factor, goal_regions);
    //   const auto goal_threshold = goal_node["threshold"].as<double>();
    //   goal_region->setThreshold(goal_threshold);
    //   pdef->setGoal(goal_region);
    // } else {
    auto goal_region = MakeGoalRegionFromGoalNode(goal_node, root_factor, root_robot, start, goal);
    pdef->setGoal(goal_region);

    //////////////////////////////////////////////////////////////////////////////// 
    // Assemble start and goal into problem definition
    //////////////////////////////////////////////////////////////////////////////// 
    return pdef;

  } else if (start_type == "joint") {
    OMPL_ERROR("JOINT. NYI");
    throw std::invalid_argument("NYI");
  } else {
    throw std::out_of_range("Unknown start type. Can only be task or joint.");
  }
  return nullptr;

}

ompl::multilevel::ProjectionPtr MakeProjectionFromName(const std::string& name, const ompl::base::SpaceInformationPtr& parent, const ompl::base::SpaceInformationPtr& child, const RobotPtr& parent_robot) {
  if(name == "ProjectionTaskSpace") {
    return std::make_shared<ProjectionTaskSpace>(parent, child, parent_robot);
  } else if(name == "Projection_RNSO2_RN") {
    return std::make_shared<ompl::multilevel::Projection_RNSO2_RN>(parent->getStateSpace(), child->getStateSpace());
  } else if(name == "Projection_TimeBased") {
    return std::make_shared<ompl::multilevel::Projection_TimeBased>(parent->getStateSpace(), child->getStateSpace());
  } else {
    OMPL_ERROR("Could not find a projection with name %s", name.c_str());
    throw std::domain_error("No projection with this name.");
  }
}

std::tuple<ompl::multilevel::FactoredSpaceInformationPtr, ompl::base::ProblemDefinitionPtr, RobotPtr, std::unordered_map<std::string, RobotPtr>, std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> >
MakeFactoredSpaceInformationFromYamlFilename(const std::string& filename, const dart::simulation::WorldPtr& world) {

  YAML::Node config = YAML::LoadFile(filename);
  OMPL_INFORM("Loading File %s", config["name"].as<std::string>().c_str());

  //////////////////////////////////////////////////////////////////////////////// 
  // Load obstacles
  //////////////////////////////////////////////////////////////////////////////// 
  OMPL_INFORM("-- Loading objects");

  auto static_obstacles = MakeObstaclesFromYamlFilename(filename);
  auto dynamic_obstacles = MakeDynamicObstaclesFromYamlFilename(filename, world, static_obstacles);

  OMPL_INFORM("Found %d static and %d dynamic obstacles.", static_obstacles.size(), dynamic_obstacles.size());

  std::vector<dart::dynamics::SkeletonPtr> obstacles;

  for(const auto& obstacle : static_obstacles) {
    obstacles.push_back(obstacle);
    world->addSkeleton(obstacle);
  }

  for(const auto& obstacle : dynamic_obstacles) {
    obstacles.push_back(obstacle.first->GetSkeleton());
  }

  //////////////////////////////////////////////////////////////////////////////// 
  // Load world parameters
  //////////////////////////////////////////////////////////////////////////////// 
  std::vector<double> gravity = config["world"]["gravity"].as<std::vector<double>>();
  world->setGravity(State3d(gravity.at(0), gravity.at(1), gravity.at(2)));

  bool coordinate_frame = config["world"]["coordinate_frame"].as<bool>();
  if(coordinate_frame) {
    addCoordinateFrameToWorld(world);
  }

  //////////////////////////////////////////////////////////////////////////////// 
  // Load robots
  //////////////////////////////////////////////////////////////////////////////// 
  OMPL_INFORM("-- Loading robots");
  
  std::unordered_map<std::string, RobotPtr> child_robots = MakeChildRobotsFromYamlFilename(filename, world, obstacles);

  OMPL_INFORM("-- Loading root robot");

  auto root_robot = MakeRootRobotFromYamlFilename(filename, world, obstacles, child_robots);

  for(const auto& dynamic_obstacle : dynamic_obstacles) {
    OMPL_INFORM("Add dynamical obstacle: %s", dynamic_obstacle.first->GetName());
    root_robot->AddDynamicalObstacle(dynamic_obstacle);
  }

  auto root = root_robot->GetSpaceInformation();
  auto root_name = GetRootRobotNameFromYamlFilename(filename);

  OMPL_INFORM("-- Loading projections");
  const auto yaml_projections = config["projections"];
  for(const auto& yaml_projection : yaml_projections) {

    auto node = yaml_projection.second;
    std::string projection_name = node["name"].as<std::string>();
    OMPL_INFORM("Loading projection %s", projection_name.c_str());

    if(node["connection"]) {
      //Connection projection
      std::pair<std::string, std::string> connection = node["connection"].as<std::pair<std::string, std::string>>();
      OMPL_INFORM("Loading projection %s -> %s", connection.first.c_str(), connection.second.c_str());
      auto parent_robot = (connection.first == root_name ? root_robot : child_robots.at(connection.first));
      auto parent = (connection.first == root_name ? root : parent_robot->GetSpaceInformation());
      auto child_robot = child_robots.at(connection.second);
      auto child = child_robot->GetSpaceInformation();
      auto projection = MakeProjectionFromName(projection_name, parent, child, parent_robot);

      if(!parent->addChild(child, projection)) {
        OMPL_ERROR("Could not add projection for child %s", child->getName().c_str());
        throw std::out_of_range("Unknown projection type.");
      }

    } else if(projection_name == "ProjectionMultiRobot") {
      auto parent = node["parent"].as<std::string>();

      if(parent != root_name) {
        OMPL_ERROR("Multi robot projections are only possible on the root node for now.");
        throw std::out_of_range("Non-root MultiRobot projection");
      }
      if(!node["children"]) {
        OMPL_ERROR("Multi robot projections requires children.");
        throw std::domain_error("Requires children");
      }

      auto children_name = node["children"].as<std::vector<std::string>>();

      size_t subspace_index = 0;
      bool computer_fiber_space = false;

      OMPL_INFORM("Loading multi robot projection %s -> ", parent.c_str());
      std::vector<RobotPtr> child_robots_of_root;

      for(const auto& child_name : children_name) {
        auto child_robot = child_robots.at(child_name);
        auto child = child_robot->GetSpaceInformation();
        OMPL_INFORM(" -> %s", child_name.c_str());
        child_robots_of_root.push_back(child_robot);

        auto projection = std::make_shared<ompl::multilevel::Projection_Subspace>(root->getStateSpace(), child->getStateSpace(), subspace_index);
        subspace_index++;

        if(!root->addChild(child, projection, computer_fiber_space)) {
          OMPL_ERROR("Could not add child");
          throw std::out_of_range("Could not add child");
        }

      }
      auto pairwise_collision_checker = std::make_shared<DartMultiRobotCollisionChecker>(root, world, child_robots_of_root);
      root->setStateValidityChecker(pairwise_collision_checker);
      root->setStateValidityCheckingResolution(0.001);

      if(node["task_space"]) {
        if(node["task_space"].as<bool>()) {
          auto motion_validator = std::make_shared<MotionValidatorTaskSpaceMultiRobot>(root);
          root->setMotionValidator(motion_validator);
        }
      }

    } else {
      OMPL_ERROR("Unknown projection type %s", projection_name.c_str());
      throw std::out_of_range("Unknown projection type.");
    }
  }

  root->printFactorization(std::cout);

  auto pdef = MakeProblemDefinitionFromYamlFilename(filename, world, root, root_robot, child_robots);

  return std::make_tuple(root, pdef, root_robot, child_robots, dynamic_obstacles);
}
