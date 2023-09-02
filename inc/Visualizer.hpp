#pragma once
#include "PinocchioInterface.hpp"

class Visualizer {
  public:
    Visualizer(const PinocchioInterface& pinocchio_interface);

    int Run();
  private:
    PinocchioInterface pinocchio_interface_;
};

