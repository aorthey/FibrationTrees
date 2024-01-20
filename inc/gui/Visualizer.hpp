#pragma once

#include <dart/gui/osg/osg.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/Path.h>

#include "gui/PathReplayWorldNode.hpp"

const Eigen::Vector3d kDefaultPathColor = Eigen::Vector3d(0.8, 0.2, 0.8);

class Visualizer {
  public:
    Visualizer() = delete;

    explicit Visualizer(const dart::simulation::WorldPtr& world);
    void AddPath(const RobotPtr& robot, const ompl::base::PathPtr& path, const Eigen::Vector3d& color = kDefaultPathColor);
    void AddPlanner(const RobotPtr& robot, const ompl::base::PlannerPtr& planner, bool displayPlannerData = false);
    void AddMultiRobotPath(const std::vector<RobotPtr>& robots, const ompl::base::PathPtr& path);
    void AddMultiRobotPlanner(const std::vector<RobotPtr>& robots, const ompl::base::PlannerPtr& planner);

    void SetCollisionChecker(const CollisionCheckerPtr& collision_checker);

    void Run();

  private:
    osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer;
    osg::ref_ptr<PathReplayWorldNode> world_node;
};
