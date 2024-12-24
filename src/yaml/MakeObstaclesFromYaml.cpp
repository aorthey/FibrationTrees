#include "yaml/MakeObstaclesFromYaml.hpp"

#include <ompl/util/Console.h>

#include "yaml/MakeFromYamlHelpers.hpp"
#include "yaml/MakeRobotFromYaml.hpp"

#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "FilePath.hpp"

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
      std::string urdf_filename = node["filename"].as<std::string>();

      auto state = State3d(0, 0, 0);
      if(node["position"]) {
        std::vector<double> position = node["position"].as<std::vector<double>>();
        state = State3d(position.at(0), position.at(1), position.at(2));
      }
      auto urdf = createFromURDF(GetDataFolder() + urdf_filename, state);
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

    auto robot = MakeAtomicRobotFromNode(obstacle.second, world, static_obstacles);

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


