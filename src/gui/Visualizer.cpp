#include "gui/Visualizer.hpp"

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
  world_node->AddPlannerData(skeleton, planner_data);

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
    pgeo.interpolate(100);
    world_node->AddSolutionPath(skeleton, path);
  }
}

void Visualizer::AddPath(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PathPtr& path) {
  world_node->AddSolutionPath(skeleton, path);
}

void Visualizer::Run() {
  viewer->run();
}
