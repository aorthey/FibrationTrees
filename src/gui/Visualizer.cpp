#include "gui/Visualizer.hpp"

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <osgGA/FirstPersonManipulator>

#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"
#include "gui/PathReplayEventHandler.hpp"

CameraData MakeDefaultCameraData() {
  CameraData data;
  data.eye = ::osg::Vec3(-10, -10, 5);
  data.center = ::osg::Vec3(0, 0, 0);
  data.up = ::osg::Vec3(0, 0, 1);
  return data;
}

Visualizer::Visualizer(const dart::simulation::WorldPtr& world) 
  : Visualizer(world, MakeDefaultCameraData()) {
}

Visualizer::Visualizer(const dart::simulation::WorldPtr& world, const CameraData& camera_data) {
  viewer = new dart::gui::osg::ImGuiViewer();

  world_node = new PathReplayWorldNode(world);

  ////////////////////////////////////////////////////////////////////////////////
  // Handle key press events
  ////////////////////////////////////////////////////////////////////////////////
  viewer->addEventHandler(new PathReplayEventHandler(world_node.get()));
  viewer->addWorldNode(world_node);

  // viewer->getImGuiHandler()->addWidget(
  //     std::make_shared<TextWidget>(viewer, world_node.get()));

  //viewer->setUpViewInWindow(0, 0, 1200, 800);

  const auto& eye = camera_data.eye;
  const auto& center = camera_data.center;
  const auto& up = camera_data.up;

  //auto manip = new osgGA::FirstPersonManipulator();
  //viewer->setCameraManipulator(manip);
  viewer->getCameraManipulator()->setHomePosition(eye, center, up);
  viewer->setCameraManipulator(viewer->getCameraManipulator()); //update 

  viewer->simulate(true);


}

void Visualizer::SetEndTime(float end_time) {
  world_node->setEndTime(end_time);
}

void Visualizer::AddPlanner(const RobotPtr& robot, const ompl::base::PlannerPtr& planner, bool displayPlannerData) {
  ////////////////////////////////////////////////////////////////////////////////
  // Visualize Planner data
  ////////////////////////////////////////////////////////////////////////////////
  if(planner->getSpaceInformation() != robot->GetSpaceInformation()) {
    OMPL_ERROR("StateSpace of robot differs from planner.");
    OMPL_ERROR("-----------------------------------------------");
    OMPL_ERROR("Planner SpaceInformation");
    planner->getSpaceInformation()->printSettings(std::cout);
    OMPL_ERROR("-----------------------------------------------");
    OMPL_ERROR("Robot SpaceInformation");
    robot->GetSpaceInformation()->printSettings(std::cout);
    OMPL_ERROR("-----------------------------------------------");
    throw std::runtime_error("Invalid state space");
  }

  if(displayPlannerData) {
    ompl::base::PlannerData planner_data(planner->getSpaceInformation());
    planner->getPlannerData(planner_data);
    OMPL_INFORM("Found %d vertices.", planner_data.numVertices());
    OMPL_INFORM("Add planner data to visualizer...");
    world_node->AddPlannerData(robot, planner_data);
  }

  ////////////////////////////////////////////////////////////////////////////////
  // Maybe add solution path
  ////////////////////////////////////////////////////////////////////////////////
  auto pdef = planner->getProblemDefinition();
  if(pdef->hasApproximateSolution() ||
     pdef->hasExactSolution())
  {
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    //OMPL_INFORM("Interpolate path...");
    //pgeo.interpolate();
    OMPL_INFORM("Add path with %d states to visualizer.", pgeo.getStateCount());
    world_node->AddPath(robot, path, kDefaultPathColor);
  }
}

void Visualizer::AddPath(const RobotPtr& robot, const ompl::base::PathPtr& path, const State3d& color) {
  world_node->AddPath(robot, path, color);
}

void Visualizer::SetCollisionChecker(const CollisionCheckerPtr& collision_checker) {
  world_node->SetCollisionChecker(collision_checker);
}

void Visualizer::Run() {
  viewer->realize();
  viewer->run();
}
