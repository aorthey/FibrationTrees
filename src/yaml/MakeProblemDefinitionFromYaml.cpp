#include "yaml/MakeProblemDefinitionFromYaml.hpp"

#include <ompl/base/Goal.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/goals/FactoredGoal.h>

#include "TaskSpaceGoal.hpp"
#include "TimeGoal.hpp"
#include "OmplHelper.hpp"
#include "DartHelper.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "yaml/MakeFromYamlHelpers.hpp"

ompl::base::GoalPtr MakeGoalRegionFromGoalNode(const YAML::Node& node, const ompl::base::SpaceInformationPtr& factor, 
    const RobotPtr& robot, const ompl::base::State* start, ompl::base::State* goal) {

  const auto goal_type = node["type"].as<std::string>();
  if(goal_type == "task" || goal_type == "joint") {
    auto goal_region = std::make_shared<ompl::base::GoalState>(factor);
    goal_region->setState(goal);
    if(node["threshold"]) {
      const auto goal_threshold = node["threshold"].as<double>();
      goal_region->setThreshold(goal_threshold);
    }
    return goal_region;
  } else if(goal_type == "time") {
    auto robot_in_time = std::static_pointer_cast<TimeBasedMobileKukaRobotTaskSpace>(robot);
    robot_in_time->TimeToState(robot_in_time->GetTMax(), goal);
    auto time_goal = std::make_shared<TimeGoal>(robot_in_time, robot_in_time->GetVMax(), start, goal);
    if(node["threshold"]) {
      const auto goal_threshold = node["threshold"].as<double>();
      time_goal->setThreshold(goal_threshold);
    }
    return time_goal;
  } else {
    OMPL_ERROR("Goal type %s not found.", goal_type.c_str());
    throw std::invalid_argument("Unknown goal type");
  }
}
int GetMaxIterations(const YAML::Node& node) {
  if(!node["max_iterations"]) {
    return kMaxResampleIteration;
  }
  return node["max_iterations"].as<int>();
}

ompl::base::ProblemDefinitionPtr MakeProblemDefinitionFromYamlFilename(
    const std::string& filename, const dart::simulation::WorldPtr& world, 
    const ompl::multilevel::FactoredSpaceInformationPtr& root_factor,
    const RobotPtr& root_robot, const std::unordered_map<std::string, RobotPtr>& child_robots) {

  YAML::Node node = YAML::LoadFile(filename);

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
      if(name == "type" || name == "time" || name == "max_iterations") {
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
        if(visualize && start_eigen.configuration.size() > 2) {
          world->addSimpleFrame(createCylinderFrame(start_eigen.configuration, State3d(0.0, M_PI*0.5, 0.0), 0.01, 0.001, State4d(0.1, 0.5, 0.1, 0.5)));
        }
      }
    }

    auto maybe_start = ComputeValidTotalState(root_factor, start_child_states, GetMaxIterations(start_node));

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
      if(name == "type" || name=="threshold" || name=="max_iterations") {
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

    auto maybe_goal = ComputeValidTotalState(root_factor, goal_child_states, GetMaxIterations(goal_node));

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

    if(start_vector.size() != root_robot->GetDimension()) {
      throw std::runtime_error("Start vector has wrong dimension " + std::to_string(start_vector.size()) 
          + ", expected " + std::to_string(root_robot->GetDimension()));
    }

    auto start = root_factor->allocState();

    root_robot->EigenToState(MakeState(start_vector), start);

    if(start_node["time"]) {
      root_robot->TimeToState(start_node["time"].as<double>(), start);
    }

    const auto goal_type = goal_node["type"].as<std::string>();

    if(!(goal_type == "joint")) {
      throw std::domain_error("Goal type needs to be joint for a joint start type.");
    }
    if(!goal_node["config"]) {
      throw std::domain_error("Requires config for root level robot " + root_robot->GetName());
    }
    auto goal_vector = goal_node["config"].as<std::vector<double>>();
    if(goal_vector.size() != root_robot->GetDimension()) {
      throw std::runtime_error("Goal vector has wrong dimension " + std::to_string(goal_vector.size()) 
          + ", expected " + std::to_string(root_robot->GetDimension()));
    }

    auto goal = root_factor->allocState();
    auto eigen_goal = MakeState(goal_vector);
    root_robot->EigenToState(eigen_goal, goal);
    if(goal_node["visualize"]) {
      auto visualize = goal_node["visualize"].as<bool>();
      if(visualize) {
        auto frames = root_robot->GetFK(eigen_goal);
        for(const auto& frame : frames) {
          world->addSimpleFrame(createCylinderFrame(frame, State3d(0.0, 0.0, 0.0), 0.25, 0.125, State4d(0.5, 0.5, 0.1, 0.5)));
        }
      }
    }

    if(goal_node["time"]) {
      root_robot->TimeToState(goal_node["time"].as<double>(), goal);
    }

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


