#include "gui/TextWidget.hpp"

TextWidget::TextWidget(
      dart::gui::osg::ImGuiViewer* viewer, PathReplayWorldNode* world_node)
    : viewer_(viewer),
      world_node_(world_node)
{
}

void TextWidget::render()
{
  ImGui::SetNextWindowPos(ImVec2(10, 20));
  ImGui::SetNextWindowSize(ImVec2(240, 320));
  ImGui::SetNextWindowBgAlpha(0.5f);
  if (!ImGui::Begin(
          "Path Control",
          nullptr,
          ImGuiWindowFlags_NoResize | ImGuiWindowFlags_MenuBar
              | ImGuiWindowFlags_HorizontalScrollbar))
  {
    ImGui::End();
    return;
  }

  //ImGui::Text("Path position: %.2f (%s)\nPath speed: %f)", 
      //world_node_->getCurrentPosition(), 
  //ImGui::Text("%s", viewer_->getInstructions().c_str());
  ImGui::Text("Path position: %.2f (%s)\nPath speed: %f)", 
      world_node_->getCurrentPosition(), 
      (world_node_->isRunning() ? "running" : "pause"),
      world_node_->getStepSize());
  //ImGui::Text("Config : %s", world_node_->getCurrentJointConfiguration().c_str());
  ImGui::End();
}

