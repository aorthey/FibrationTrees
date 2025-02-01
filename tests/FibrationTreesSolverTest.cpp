#include <gtest/gtest.h>

#include "input/FibrationTreesSolverExecuter.hpp"

TEST(FibrationTreesSolverTest, LoadingScenariosTest) {
  const char* cmd[]{"FibrationTreesSolverTest", "../tests/data/three_disks.yaml", "--dry" };
  int argc = sizeof(cmd) / sizeof(cmd[0]);
  EXPECT_NO_THROW(FibrationTreesSolverExecuter(argc, cmd));
}
