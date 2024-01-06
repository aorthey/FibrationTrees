#include "gui/Visualizer.hpp"

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "DartHelper.hpp"
#include "OmplHelper.hpp"

Visualizer::Visualizer(const dart::simulation::WorldPtr& world) {

  viewer = new dart::gui::osg::ImGuiViewer();

  world_node = new PathReplayWorldNode(world);

  ////////////////////////////////////////////////////////////////////////////////
  // Handle key press events
  ////////////////////////////////////////////////////////////////////////////////
  viewer->addEventHandler(new PathReplayEventHandler(world_node.get()));
  viewer->addWorldNode(world_node);
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<TextWidget>(viewer, world_node.get()));

  viewer->setUpViewInWindow(0, 0, 640, 480);

  const auto& eye = ::osg::Vec3(-3, 0, 2);
  const auto& center = ::osg::Vec3(0, 0, 0.5);
  const auto& up = ::osg::Vec3(0, 0, 1);

  viewer->getCameraManipulator()->setHomePosition(eye, center, up);
  viewer->setCameraManipulator(viewer->getCameraManipulator()); //update 

  viewer->simulate(true);
}

void Visualizer::AddPlanner(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PlannerPtr& planner) {
  ////////////////////////////////////////////////////////////////////////////////
  // Visualize Planner data
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::PlannerData planner_data(planner->getSpaceInformation());
  planner->getPlannerData(planner_data);
  OMPL_INFORM("Found %d vertices.", planner_data.numVertices());
  // OMPL_INFORM("Add planner data to visualizer...");
  // world_node->AddPlannerData(skeleton, planner_data);

  ////////////////////////////////////////////////////////////////////////////////
  // Maybe add solution path
  ////////////////////////////////////////////////////////////////////////////////
  auto pdef = planner->getProblemDefinition();
  if(pdef->hasApproximateSolution() ||
     pdef->hasExactSolution())
  {
    OMPL_INFORM("Print solution path...");
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    OMPL_INFORM("Found path with %d states.", pgeo.getStateCount());
    for(const auto& state : pgeo.getStates()) {
      path->getSpaceInformation()->printState(state);
      const auto config = StateToEigenVectorXd(path->getSpaceInformation(), state);
      const auto v = GetFK(skeleton, config);
      std::cout << "EndEffector position is " << v[0] << ", " << v[1] << ", " << v[2] << std::endl;
    }
    OMPL_INFORM("Interpolate path...");
    // pgeo.interpolate();
    OMPL_INFORM("Add path to visualizer...");
    AddPath(skeleton, path);
  }
}

void Visualizer::AddPlanner(const RobotPtr& robot, const ompl::base::PlannerPtr& planner) {
  ////////////////////////////////////////////////////////////////////////////////
  // Visualize Planner data
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::PlannerData planner_data(planner->getSpaceInformation());
  planner->getPlannerData(planner_data);
  OMPL_INFORM("Found %d vertices.", planner_data.numVertices());
  OMPL_INFORM("Add planner data to visualizer...");
  world_node->AddPlannerData(robot, planner_data);

  ////////////////////////////////////////////////////////////////////////////////
  // Maybe add solution path
  ////////////////////////////////////////////////////////////////////////////////
  auto pdef = planner->getProblemDefinition();
  if(pdef->hasApproximateSolution() ||
     pdef->hasExactSolution())
  {
    OMPL_INFORM("Print solution path...");
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    OMPL_INFORM("Found path with %d states.", pgeo.getStateCount());
    for(const auto& state : pgeo.getStates()) {
      path->getSpaceInformation()->printState(state);
      const auto config = robot->StateToEigen(state);
      const auto v = GetFK(robot->GetSkeleton(), config);
      std::cout << "EndEffector position is " << v[0] << ", " << v[1] << ", " << v[2] << std::endl;
    }
    OMPL_INFORM("Interpolate path...");
    pgeo.interpolate();
    OMPL_INFORM("Add path to visualizer...");
    world_node->AddPath(robot, path, kDefaultPathColor);
  }
}

void Visualizer::AddPath(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PathPtr& path, const Eigen::Vector3d& color) {
  world_node->AddPath(skeleton, path, color);
}

void Visualizer::SetCollisionChecker(const CollisionCheckerPtr& collision_checker) {
  world_node->SetCollisionChecker(collision_checker);
}

void Visualizer::AddMultiRobotPath(const std::unordered_map<std::string, dart::dynamics::SkeletonPtr>& skeletons, const ompl::base::PathPtr& path) {
  typedef std::unordered_map<std::string, ompl::base::State*> SplitConfig;

  auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(path->getSpaceInformation());
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());

  std::vector<SplitConfig> configs;
  for(const auto& state : pgeo.getStates()) {
    factor->printState(state);
    auto childStates = factor->allocChildStates();
    factor->project(state, childStates);
    configs.push_back(childStates);
  }

  for(const auto& skeleton : skeletons) {
    const auto& name = skeleton.first;
    std::vector<const ompl::base::State*> states;
    for(const auto& config : configs) {
      auto it = config.find(name);
      if(it == config.end()){
        OMPL_ERROR("Could not find %s in states.", name.c_str());
        throw "NameDoesNotExist";
      }
      states.push_back(it->second); 
    }

    auto child = factor->getChild(name);
    auto path = std::make_shared<ompl::geometric::PathGeometric>(child, states);
    AddPath(skeleton.second, path);
  }
}

void Visualizer::AddMultiRobotPlanner(const std::unordered_map<std::string, dart::dynamics::SkeletonPtr>& skeletons, 
    const ompl::base::PlannerPtr& planner) {
  ////////////////////////////////////////////////////////////////////////////////
  // Visualize Planner data
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::PlannerData planner_data(planner->getSpaceInformation());
  planner->getPlannerData(planner_data);
  OMPL_INFORM("Found %d vertices.", planner_data.numVertices());

  world_node->AddMultiRobotPlannerData(skeletons, planner_data);

  ////////////////////////////////////////////////////////////////////////////////
  // Maybe add solution path
  ////////////////////////////////////////////////////////////////////////////////
  auto pdef = planner->getProblemDefinition();
  if(pdef->hasApproximateSolution() ||
     pdef->hasExactSolution())
  {
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    for(const auto& state : pgeo.getStates()) {
      path->getSpaceInformation()->printState(state);
    }
    // pgeo.interpolate();
    AddMultiRobotPath(skeletons, path);
  }
}

void Visualizer::AddMultiRobotPath(const std::vector<RobotPtr>& robots, const ompl::base::PathPtr& path) {
  typedef std::unordered_map<std::string, ompl::base::State*> SplitConfig;

  auto factor = std::static_pointer_cast<ompl::multilevel::FactoredSpaceInformation>(path->getSpaceInformation());
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());

  std::vector<SplitConfig> configs;
  for(const auto& state : pgeo.getStates()) {
    factor->printState(state);
    auto childStates = factor->allocChildStates();
    factor->project(state, childStates);
    configs.push_back(childStates);
  }

  for(const auto& robot : robots) {
    const auto& name = robot->GetSpaceInformation()->getName();
    std::vector<const ompl::base::State*> states;
    for(const auto& config : configs) {
      auto it = config.find(name);
      if(it == config.end()){
        OMPL_ERROR("Could not find %s in states.", name.c_str());
        throw "NameDoesNotExist";
      }
      states.push_back(it->second); 
    }

    auto child = factor->getChild(name);
    auto child_path = std::make_shared<ompl::geometric::PathGeometric>(child, states);
    world_node->AddPath(robot, child_path, kDefaultPathColor);
  }
}

void Visualizer::AddMultiRobotPlanner(const std::vector<RobotPtr>& robots, 
    const ompl::base::PlannerPtr& planner) {
  ////////////////////////////////////////////////////////////////////////////////
  // Visualize Planner data
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::PlannerData planner_data(planner->getSpaceInformation());
  planner->getPlannerData(planner_data);
  OMPL_INFORM("Found %d vertices.", planner_data.numVertices());

  world_node->AddMultiRobotPlannerData(robots, planner_data);

  ////////////////////////////////////////////////////////////////////////////////
  // Maybe add solution path
  ////////////////////////////////////////////////////////////////////////////////
  auto pdef = planner->getProblemDefinition();
  if(pdef->hasApproximateSolution() ||
     pdef->hasExactSolution())
  {
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    for(const auto& state : pgeo.getStates()) {
      path->getSpaceInformation()->printState(state);
    }
    // pgeo.interpolate();
    AddMultiRobotPath(robots, path);
  }
}

void Visualizer::Run() {
  viewer->run();
}
