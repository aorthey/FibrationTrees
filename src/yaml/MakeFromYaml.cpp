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

#include "yaml/MakeObstaclesFromYaml.hpp"
#include "yaml/MakeRobotFromYaml.hpp"
#include "yaml/MakeFromYamlHelpers.hpp"

#include "FilePath.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "TaskSpaceGoal.hpp"
#include "TimeGoal.hpp"
#include "robots/Robot.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "projections/ProjectionTaskSpace.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"

////////////////////////////////////////////////////////////////////////////////
// ProblemDefinition
////////////////////////////////////////////////////////////////////////////////

ompl::base::GoalPtr MakeGoalRegionFromGoalNode(const YAML::Node& node, const ompl::base::SpaceInformationPtr& factor, 
    const RobotPtr& robot, const ompl::base::State* start, ompl::base::State* goal) {

  const auto goal_type = node["type"].as<std::string>();
  if(goal_type == "task" || goal_type == "joint") {
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
    throw std::invalid_argument("No problem tag");
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
      VerifyRobotNameExistsAndIsNotRoot(name, root_robot, child_robots);

      auto robot = child_robots.at(name);
      auto si = robot->GetSpaceInformation();
      ompl::base::State *start_ompl_state = si->allocState();
      robot->EigenToState(start_eigen, start_ompl_state);
      start_child_states[si->getName()] = start_ompl_state;

      //Maybe visualize
      if(robot_node.second["visualize"]) {
        auto visualize = robot_node.second["visualize"].as<bool>();
        if(visualize) {
          world->addSimpleFrame(createCylinderFrame(start_eigen.configuration, State3d(0.0, M_PI*0.5, 0.0), 0.01, 0.001, State4d(0.1, 0.5, 0.1, 0.5)));
        }
      }
    }

    auto maybe_start = ComputeValidTotalState(root_factor, start_child_states);

    if(!maybe_start.has_value()){
      OMPL_ERROR("Could not compute valid start.");
      throw std::runtime_error("Could not compute valid start");
    }
    auto start = maybe_start.value();
    
    if(start_node["time"]) {
      root_robot->TimeToState(start_node["time"].as<double>(), start);
    }

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
      VerifyRobotNameExistsAndIsNotRoot(name, root_robot, child_robots);
      auto robot = child_robots.at(name);
      auto si = robot->GetSpaceInformation();
      ompl::base::State *goal_ompl_state = si->allocState();
      robot->EigenToState(goal_eigen, goal_ompl_state);
      goal_child_states[si->getName()] = goal_ompl_state;
      space_name_to_robot_name[si->getName()] = name;

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
    if(!start_node["config"]) {
      throw std::domain_error("Requires config for root level robot " + root_robot->GetName());
    }
    auto start_vector = start_node["config"].as<std::vector<double>>();
    auto start = root_factor->allocState();
    root_robot->EigenToState(MakeState(start_vector), start);

    if(start_node["time"]) {
      root_robot->TimeToState(start_node["time"].as<double>(), start);
    }
    root_factor->printState(start);


    const auto goal_type = goal_node["type"].as<std::string>();

    if(!(goal_type == "joint")) {
      throw std::domain_error("Goal type needs to be joint for a joint start type.");
    }
    if(!goal_node["config"]) {
      throw std::domain_error("Requires config for root level robot " + root_robot->GetName());
    }
    auto goal_vector = goal_node["config"].as<std::vector<double>>();
    auto goal = root_factor->allocState();
    root_robot->EigenToState(MakeState(goal_vector), goal);

    if(goal_node["time"]) {
      root_robot->TimeToState(goal_node["time"].as<double>(), goal);
    }
    root_factor->printState(goal);

    ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(root_factor);
    pdef->addStartState(start);
    auto goal_region = MakeGoalRegionFromGoalNode(goal_node, root_factor, root_robot, start, goal);
    pdef->setGoal(goal_region);
    return pdef;

  } else {
    throw std::out_of_range("Unknown start type. Can only be task or joint.");
  }

  throw std::runtime_error("Could not return a valid ProblemDefinition.");
  return nullptr;
}

////////////////////////////////////////////////////////////////////////////////
// Make projections
////////////////////////////////////////////////////////////////////////////////

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
