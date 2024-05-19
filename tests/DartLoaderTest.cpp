#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "FilePath.hpp"

TEST(DartLoaderTest, LoadUrdfTest) {
  dart::utils::DartLoader loader;
  dart::dynamics::SkeletonPtr manipulator
    = loader.parseSkeleton(GetDataFolder() + "robots/kuka_lwr/kuka.urdf");
  auto numDofs = manipulator->getNumDofs();
  EXPECT_EQ(numDofs, 13u);
  const auto lb = manipulator->getPositionLowerLimits();
  const auto ub = manipulator->getPositionUpperLimits();
  const auto config = manipulator->getConfiguration().mPositions;
  for(size_t k = 0; k < config.size(); k++) {
    EXPECT_LT(config[k], ub[k]);
    EXPECT_GT(config[k], lb[k]);
  }
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

