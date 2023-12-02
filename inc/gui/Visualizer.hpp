#pragma once

#include <dart/gui/osg/osg.hpp>

#include <ompl/base/Planner.h>
#include <ompl/base/PlannerData.h>
#include <ompl/base/Path.h>

#include "gui/DartEventHandler.hpp"

class Visualizer {
  public:
    Visualizer() = delete;

    explicit Visualizer(const dart::simulation::WorldPtr& world);

    void AddPlanner(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PlannerPtr& planner);
    void AddPath(const dart::dynamics::SkeletonPtr& skeleton, const ompl::base::PathPtr& path);

    void Run();

  private:
    osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer;
    osg::ref_ptr<PathReplayWorldNode> world_node;
};
