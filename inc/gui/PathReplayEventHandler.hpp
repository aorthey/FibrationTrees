#pragma once
#include <dart/gui/osg/osg.hpp>
#include <dart/external/imgui/imgui.h>

#include "gui/PathReplayWorldNode.hpp"

class PathReplayEventHandler : public osgGA::GUIEventHandler
{
public:
  PathReplayEventHandler(PathReplayWorldNode* world_node);

  bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) override;

private:
  PathReplayWorldNode* world_node_;
};

