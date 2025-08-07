#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include <dart/dart.hpp>
#include "yaml-cpp/yaml.h"

#include "yaml/MakeObstaclesFromYaml.hpp"
#include "yaml/MakeRobotFromYaml.hpp"
#include "yaml/MakeFromYamlHelpers.hpp"
#include "yaml/MakeProjectionsFromYaml.hpp"
#include "yaml/MakeProblemDefinitionFromYaml.hpp"

#include "FilePath.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "robots/Robot.hpp"

////////////////////////////////////////////////////////////////////////////////
// Make factored space information
////////////////////////////////////////////////////////////////////////////////
std::tuple<ompl::multilevel::FactoredSpaceInformationPtr, ompl::base::ProblemDefinitionPtr, RobotPtr, std::unordered_map<std::string, RobotPtr>, std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> >
MakeFactoredSpaceInformationFromYamlFilename(const std::string& filename, const dart::simulation::WorldPtr& world) {

  YAML::Node config = YAML::LoadFile(filename);
  OMPL_INFORM("Loading File %s", config["name"].as<std::string>().c_str());

  //////////////////////////////////////////////////////////////////////////////// 
  // Load obstacles
  //////////////////////////////////////////////////////////////////////////////// 
  OMPL_INFORM("-- Loading objects --");

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
  if(config["world"]) {
    if(config["world"]["gravity"]) {
      std::vector<double> gravity = config["world"]["gravity"].as<std::vector<double>>();
      world->setGravity(State3d(gravity.at(0), gravity.at(1), gravity.at(2)));
    }

    if(config["world"]["coordinate_frame"]) {
      bool coordinate_frame = config["world"]["coordinate_frame"].as<bool>();
      if(coordinate_frame) {
        addCoordinateFrameToWorld(world);
      }
    }
  }

  //////////////////////////////////////////////////////////////////////////////// 
  // Load robots
  //////////////////////////////////////////////////////////////////////////////// 
  OMPL_INFORM("-- Loading robots --");
  
  std::unordered_map<std::string, RobotPtr> child_robots = MakeChildRobotsFromYamlFilename(filename, world, obstacles);

  OMPL_INFORM("-- Loading root robot --");

  auto root_robot = MakeRootRobotFromYamlFilename(filename, world, obstacles, child_robots);

  for(const auto& dynamic_obstacle : dynamic_obstacles) {
    OMPL_INFORM("Add dynamical obstacle: %s", dynamic_obstacle.first->GetName().c_str());
    root_robot->AddDynamicalObstacle(dynamic_obstacle);
  }

  auto root = root_robot->GetSpaceInformation();

  //////////////////////////////////////////////////////////////////////////////// 
  // Load projections
  //////////////////////////////////////////////////////////////////////////////// 
  OMPL_INFORM("-- Loading projections --");

  MakeProjectionsFromYamlFilename(filename, world, root, root_robot, child_robots);
  root->printFactorization(std::cout);

  //////////////////////////////////////////////////////////////////////////////// 
  // Make problemdefinition
  //////////////////////////////////////////////////////////////////////////////// 

  OMPL_INFORM("-- Loading problem definition --");
  auto pdef = MakeProblemDefinitionFromYamlFilename(filename, world, root, root_robot, child_robots);

  auto start = pdef->getStartState(0);
  root_robot->SetConfiguration(root_robot->StateToEigen(start));

  return std::make_tuple(root, pdef, root_robot, child_robots, dynamic_obstacles);
}
