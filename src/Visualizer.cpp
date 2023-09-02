#include "Visualizer.hpp"

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/LineWidth>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osgDB/ReadFile>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgGA/GUIEventHandler>

class MyKeyboardEventHandler : public osgGA::GUIEventHandler {
public:
  virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'a':
          std::cout << "A key pressed" << std::endl;
          break;
        case 'b':
          break;
        default:
          break;
      }
    }
    return false;
  }
};

Visualizer::Visualizer(const PinocchioInterface& pinocchio_interface) :
  pinocchio_interface_(pinocchio_interface) 
{
}

int Visualizer::Run() {
  osgViewer::Viewer viewer;
  //viewer.setSceneData(root);

  MyKeyboardEventHandler* handler = new MyKeyboardEventHandler();
  viewer.addEventHandler(handler);

  return viewer.run();
}
