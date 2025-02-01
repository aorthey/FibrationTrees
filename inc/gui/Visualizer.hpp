#pragma once

#include <dart/gui/osg/osg.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/Path.h>

#include "gui/PathReplayWorldNode.hpp"

const State3d kDefaultPathColor = State3d(0.8, 0.2, 0.8);

struct CameraData {
  ::osg::Vec3 eye;
  ::osg::Vec3 center;
  ::osg::Vec3 up;
};

class Visualizer {
  public:
    Visualizer() = delete;

    explicit Visualizer(const dart::simulation::WorldPtr& world);
    explicit Visualizer(const dart::simulation::WorldPtr& world, const CameraData& camera_data);
    void AddPath(const RobotPtr& robot, const ompl::base::PathPtr& path, const State3d& color = kDefaultPathColor);
    void AddPlanner(const RobotPtr& robot, const ompl::base::PlannerPtr& planner, bool displayPlannerData = false);

    void SetCollisionChecker(const CollisionCheckerPtr& collision_checker);

    void Run();

    void SetEndTime(float end_time);

  private:
    osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer;
    osg::ref_ptr<PathReplayWorldNode> world_node;
};
