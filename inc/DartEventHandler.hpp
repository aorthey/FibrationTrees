#pragma once

#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/external/imgui/imgui.h>
#include <ompl/base/Path.h>

#include "CollisionChecker.hpp"

const float kStepSize = 0.01;
class PathReplayWorldNode : public dart::gui::osg::RealTimeWorldNode
{
public:
  PathReplayWorldNode(
      dart::simulation::WorldPtr world,
      dart::dynamics::SkeletonPtr manipulator,
      const ompl::base::PathPtr& path,
      const CollisionCheckerPtr& collision_checker);

  ~PathReplayWorldNode();

  void customPreRefresh();
  void customPostRefresh();
  void customPreStep();
  void customPostStep();
  void toggleStartStop();
  void toggleReverse();

  float getCurrentPosition() const;

protected:
  dart::dynamics::SkeletonPtr manipulator_;
  ompl::base::PathPtr path_;
  CollisionCheckerPtr collision_checker_;

  std::vector<float> lengths_;
  int current_index_;
  float current_position_;
  float step_size_;
  ompl::base::State* tmpState_;

  bool pause_;
  bool reverse_;
};

class PathReplayEventHandler : public osgGA::GUIEventHandler
{
public:
  PathReplayEventHandler(PathReplayWorldNode* worldNode);

  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

private:
  PathReplayWorldNode* mWorldNode;
};


class TextWidget : public dart::gui::osg::ImGuiWidget
{
public:
  TextWidget(dart::gui::osg::ImGuiViewer* viewer, PathReplayWorldNode* worldNode);

  void render() override;

protected:
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer_;
  PathReplayWorldNode* mWorldNode;
};
