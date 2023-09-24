#include "PinocchioInterface.hpp"
#include "Visualizer.hpp"

#include <iostream>

int main(int argc, char ** argv)
{
  PinocchioInterface pi;
  pi.LoadRobot("/home/aorthey/git/FibrationTrees/data/nasa_valkyrie_model/valkyrie.urdf");
  //pi.LoadRobot("/home/aorthey/git/FibrationTrees/data/nasa_valkyrie_model/valkyrie_sim.urdf");

  Visualizer visualizer(pi);
  return visualizer.Run();
}
