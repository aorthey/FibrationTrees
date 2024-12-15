#include <gtest/gtest.h>

#include "FibrationTreesSolverExecuter.hpp"

TEST(FibrationTreesSolverTest, LoadingScenariosTest) {
  char* cmd[] = { "FibrationTreesSolverTest", "../data/scenarios/00_test/01_ThreeDisks.yaml", "--dry" };
  int argc = sizeof(cmd) / sizeof(cmd[0]);
  FibrationTreesSolverExecuter(argc, cmd);
}
