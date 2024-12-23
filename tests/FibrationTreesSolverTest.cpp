#include <gtest/gtest.h>

#include "input/FibrationTreesSolverExecuter.hpp"

TEST(FibrationTreesSolverTest, LoadingScenariosTest) {
  const char* cmd[]{"FibrationTreesSolverTest", "../data/scenarios/00_test/01_ThreeDisks.yaml", "--dry" };
  int argc = sizeof(cmd) / sizeof(cmd[0]);
  FibrationTreesSolverExecuter(argc, cmd);
}
