#include "gui/PathReplayEventHandler.hpp"

PathReplayEventHandler::PathReplayEventHandler(PathReplayWorldNode* world_node)
{
  world_node_ = world_node;
}

bool PathReplayEventHandler::handle(
    const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) 
{
  if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN)
  {
    const auto events = world_node_->GetKeyPressEvents();
    for(const auto& event : events) {
      if(ea.getKey() != event.key) {
        continue;
      }
      event.function();
      return true;
    }
  }
  return false;
}

