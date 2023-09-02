#include "PinocchioInterface.hpp"
#include "Visualizer.hpp"

#include <iostream>

int main(int argc, char ** argv)
{
  PinocchioInterface pi;
  pi.LoadRobot("/home/aorthey/git/FibrationTrees/data/nasa_valkyrie_model/valkyrie.urdf");

  Visualizer visualizer(pi);
  return visualizer.Run();
  // for(size_t k = 0; k < 100; k++) {
  //   auto q = pi.GetRandomConfiguration();
  //   if(!pi.IsInCollision(q)) {
  //     std::cout << "Collision Free: q: " << q.transpose() << std::endl;
  //     return pi.Visualize(q);
  //   }
  // }
  //auto config = pi.GetRandomConfiguration();
  //std::cout << "Visualizing config " << config << std::endl;
  // return pi.Visualize(config);
}
