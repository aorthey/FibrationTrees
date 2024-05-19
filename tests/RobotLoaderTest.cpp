#include <gtest/gtest.h>
#include "robots/KukaRobot.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MobileKukaBase.hpp"
#include "robots/MobileKukaRobot.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/MobileCar.hpp"
#include "robots/MobileCarDisk.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "robots/TimeBasedSphereRobot.hpp"
#include "robots/ZeppelinInnerSphereRobot.hpp"
#include "robots/ZeppelinRobot.hpp"
#include "robots/RobotFactory.hpp"

#include "OmplHelper.hpp"
#include "DartHelper.hpp"

template <class T>
class RobotLoaderTest : public testing::Test {};

using RobotTypes = ::testing::Types<
    KukaRobot, 
    KukaRobotTaskSpace, 
    MobileCar, 
    MobileCarDisk, 
    MobileKukaBase,
    MobileKukaRobot, 
    MobileKukaRobotTaskSpace, 
    SphereRobot, 
    //TimeBasedMobileKukaRobotTaskSpace, 
    //TimeBasedSphereRobot, 
    ZeppelinInnerSphereRobot,
    ZeppelinRobot
>;

TYPED_TEST_SUITE(RobotLoaderTest, RobotTypes);

TYPED_TEST(RobotLoaderTest, DefaultLoaderTest) {
  auto robot = MakeRobot<TypeParam>();
  std::cout << "Done" << std::endl;

  auto si = robot->GetSpaceInformation();

  const auto Ndim = si->getStateDimension();
  //const auto Ndim = robot->GetSkeleton()->getNumDofs();

  std::cout << "Created robot " << robot->GetSpaceInformation()->getName() << " with " << Ndim << " dimensions." << std::endl;
  StateXd v = MakeConstantState(Ndim, 0.0f);
  for(size_t k = 0; k < Ndim; k++) {
    v[k] = k+1;
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::cout << "Test EigenToState" << std::endl;
  ////////////////////////////////////////////////////////////////////////////////
  auto q = si->allocState();
  robot->EigenToState(v, q);

  ////////////////////////////////////////////////////////////////////////////////
  std::cout << "Test StateToEigen" << std::endl;
  ////////////////////////////////////////////////////////////////////////////////
  auto w = robot->StateToEigen(q);

  for(size_t k = 0; k < Ndim; k++) {
    EXPECT_EQ(v[k], w[k]);
  }

  ////////////////////////////////////////////////////////////////////////////////
  std::cout << "Test IsValid" << std::endl;
  ////////////////////////////////////////////////////////////////////////////////
  for(size_t k = 0; k < Ndim; k++) {
    v[k] = std::numeric_limits<double>::quiet_NaN();
  }
  robot->EigenToState(v, q);
  EXPECT_FALSE(robot->IsValid(q));

  for(size_t k = 0; k < Ndim; k++) {
    v[k] = 0.0f;
  }
  robot->EigenToState(v, q);
  EXPECT_NO_THROW(robot->IsValid(q));
  EXPECT_NO_THROW(robot->GetSpaceInformation()->isValid(q));

  si->freeState(q);
}

TYPED_TEST(RobotLoaderTest, ObstacleLoaderTest) {

  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  dart::dynamics::SkeletonPtr floor = createFloor();
  obstacles.push_back(floor);

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(floor);

  ////////////////////////////////////////////////////////////////////////////////
  std::cout << "Create Robot" << std::endl;
  ////////////////////////////////////////////////////////////////////////////////
  auto robot = MakeRobot<TypeParam>(world, obstacles);

  auto factor = robot->GetSpaceInformation();
  const auto Ndim = factor->getStateDimension();

  auto q = factor->allocState();
  StateXd v = MakeConstantState(Ndim, std::numeric_limits<double>::quiet_NaN());

  ////////////////////////////////////////////////////////////////////////////////
  std::cout << "Test IsValid" << std::endl;
  ////////////////////////////////////////////////////////////////////////////////

  robot->EigenToState(v, q);
  EXPECT_FALSE(factor->isValid(q));

  //Inf is invalid
  for(size_t k = 0; k < Ndim; k++) {
    v[k] = std::numeric_limits<double>::infinity();
  }
  robot->EigenToState(v, q);
  EXPECT_FALSE(factor->isValid(q));

  //Zero does not throw
  for(size_t k = 0; k < Ndim; k++) {
    v[k] = 0.0f;
  }
  robot->EigenToState(v, q);
  EXPECT_NO_THROW(factor->isValid(q));

  ////////////////////////////////////////////////////////////////////////////////
  factor->freeState(q);
}

TYPED_TEST(RobotLoaderTest, JointLimitTest) {
  // RobotFactory<TypeParam> robot_factory;
  // auto robot = robot_factory.Create();
  auto robot = MakeRobot<TypeParam>();//robot_factory.Create();

  auto skeleton = robot->GetSkeleton();
  auto lb = skeleton->getPositionLowerLimits();
  auto ub = skeleton->getPositionUpperLimits();

  for(size_t k = 0; k < lb.size(); k++) {
    if(!skeleton->getDof(k)->hasPositionLimit()) {
      continue;
    }
    EXPECT_LT(lb[k], std::numeric_limits<double>::infinity());
    EXPECT_LT(ub[k], std::numeric_limits<double>::infinity());
    EXPECT_GT(lb[k], -std::numeric_limits<double>::infinity());
    EXPECT_GT(ub[k], -std::numeric_limits<double>::infinity());
  }
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
