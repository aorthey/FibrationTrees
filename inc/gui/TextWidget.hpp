#pragma once
#include <dart/external/imgui/imgui.h>
#include "gui/PathReplayWorldNode.hpp"

class TextWidget : public dart::gui::osg::ImGuiWidget
{
public:
  TextWidget(dart::gui::osg::ImGuiViewer* viewer, PathReplayWorldNode* world_node);

  void render() override;

protected:
  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer_;
  PathReplayWorldNode* world_node_;
};

