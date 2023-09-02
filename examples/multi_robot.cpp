#include "PinocchioInterface.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/SimpleSetup.h>

#include <ompl/config.h>

#include <urdf_parser/urdf_parser.h>

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>

#include <iostream>

using namespace ompl;

int main(int argc, char ** argv)
{
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
  auto space(std::make_shared<base::SE3StateSpace>());
  base::RealVectorBounds bounds(3);
  bounds.setLow(-1);
  bounds.setHigh(1);
  space->setBounds(bounds);
  auto si(std::make_shared<base::SpaceInformation>(space));

  PinocchioInterface pi;
  // pi.LoadRobot("ur_description/urdf/ur5_robot.urdf");
  pi.LoadRobot("/home/aorthey/git/FibrationTrees/data/nasa_valkyrie_model/valkyrie.urdf");

  // for(size_t k = 0; k < 100; k++) {
  //   auto q = pi.GetRandomConfiguration();
  //   if(!pi.IsInCollision(q)) {
  //     std::cout << "Collision Free: q: " << q.transpose() << std::endl;
  //     return pi.Visualize(q);
  //   }
  // }
  auto config = pi.GetRandomConfiguration();
  std::cout << "Visualizing config " << config << std::endl;
  return pi.Visualize(config);
}
