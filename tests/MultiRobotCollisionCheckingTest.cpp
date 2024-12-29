#include <gtest/gtest.h>
#include <dart/dart.hpp>

#include <ompl/base/State.h>
#include <ompl/base/ScopedState.h>

#include "CollisionChecker.hpp"
#include "Common.hpp"
#include "CollisionCheckerHelper.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"

TEST(MultiRobotCollisionCheckingTest, MultiRobotCollisionCheckerNoThrowTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto robotA = MakeRobot<SphereRobot>(world);
  auto robotB = MakeRobot<SphereRobot>(world);
  auto robotC = MakeRobot<SphereRobot>(world);
  auto robotD = MakeRobot<SphereRobot>(world);

  auto robotX = MultiRobot::MakeMultiRobot({robotA, robotB});
  auto robotY = MultiRobot::MakeMultiRobot({robotX, robotC});
  auto robotZ = MultiRobot::MakeMultiRobot({robotY, robotD});

  EXPECT_FALSE(robotA->IsMultiRobot());
  EXPECT_FALSE(robotB->IsMultiRobot());
  EXPECT_FALSE(robotC->IsMultiRobot());
  EXPECT_FALSE(robotD->IsMultiRobot());

  EXPECT_TRUE(robotX->IsMultiRobot());
  EXPECT_TRUE(robotY->IsMultiRobot());
  EXPECT_TRUE(robotZ->IsMultiRobot());

  EXPECT_EQ(robotX->GetSubRobots().size(), 2u);
  EXPECT_EQ(robotY->GetSubRobots().size(), 2u);
  EXPECT_EQ(robotZ->GetSubRobots().size(), 2u);

  auto spaceX = robotX->GetSpaceInformation();
  auto spaceY = robotY->GetSpaceInformation();
  auto spaceZ = robotZ->GetSpaceInformation();

  EXPECT_TRUE(spaceX->getStateSpace()->isCompound());
  EXPECT_TRUE(spaceY->getStateSpace()->isCompound());
  EXPECT_TRUE(spaceZ->getStateSpace()->isCompound());

  auto collision_checker_robotX = std::make_shared<MultiRobotCollisionChecker>(world, robotX);
  auto collision_checker_robotY = std::make_shared<MultiRobotCollisionChecker>(world, robotY);
  auto collision_checker_robotZ = std::make_shared<MultiRobotCollisionChecker>(world, robotZ);

  ompl::base::ScopedState<> stateX(spaceX);
  EXPECT_NO_THROW(collision_checker_robotX->isValid(stateX.get()));

  ompl::base::ScopedState<> stateY(spaceY);
  EXPECT_NO_THROW(collision_checker_robotY->isValid(stateY.get()));

  ompl::base::ScopedState<> stateZ(spaceZ);
  EXPECT_NO_THROW(collision_checker_robotZ->isValid(stateZ.get()));
}

TEST(MultiRobotCollisionCheckingTest, CollectAtomicRobotsTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto robotA = MakeRobot<SphereRobot>(world);
  auto robotB = MakeRobot<SphereRobot>(world);
  auto robotC = MakeRobot<SphereRobot>(world);
  auto robotD = MakeRobot<SphereRobot>(world);

  auto robotX = MultiRobot::MakeMultiRobot({robotA, robotB});
  auto robotY = MultiRobot::MakeMultiRobot({robotX, robotC});
  auto robotZ = MultiRobot::MakeMultiRobot({robotY, robotD});

  EXPECT_EQ(CollectAtomicRobots(robotX).size(), 2u);
  EXPECT_EQ(CollectAtomicRobots(robotY).size(), 3u);
  EXPECT_EQ(CollectAtomicRobots(robotZ).size(), 4u);
}

TEST(MultiRobotCollisionCheckingTest, GetFKTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto robotA = MakeRobot<SphereRobot>(world);
  auto robotB = MakeRobot<SphereRobot>(world);
  auto robotC = MakeRobot<SphereRobot>(world);
  auto robotD = MakeRobot<SphereRobot>(world);

  auto robotX = MultiRobot::MakeMultiRobot({robotA, robotB});
  { 
    auto tcpX = robotX->GetFK(MakeState({1.0, 1.0, 1.0, 2.0, 2.0, 2.0}));
    EXPECT_EQ(tcpX.size(), 2u);

    EXPECT_NEAR(tcpX.at(0)[0], 1.0, Epsilon);
    EXPECT_NEAR(tcpX.at(0)[1], 1.0, Epsilon);
    EXPECT_NEAR(tcpX.at(0)[2], 1.0, Epsilon);
    EXPECT_NEAR(tcpX.at(1)[0], 2.0, Epsilon);
    EXPECT_NEAR(tcpX.at(1)[1], 2.0, Epsilon);
    EXPECT_NEAR(tcpX.at(1)[2], 2.0, Epsilon);
  }

  auto robotY = MultiRobot::MakeMultiRobot({robotX, robotC});
  {
    auto tcpY = robotY->GetFK(MakeState({1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0}));
    EXPECT_EQ(tcpY.size(), 3u);

    EXPECT_NEAR(tcpY.at(0)[0], 1.0, Epsilon);
    EXPECT_NEAR(tcpY.at(0)[1], 1.0, Epsilon);
    EXPECT_NEAR(tcpY.at(0)[2], 1.0, Epsilon);
    EXPECT_NEAR(tcpY.at(1)[0], 2.0, Epsilon);
    EXPECT_NEAR(tcpY.at(1)[1], 2.0, Epsilon);
    EXPECT_NEAR(tcpY.at(1)[2], 2.0, Epsilon);
    EXPECT_NEAR(tcpY.at(2)[0], 3.0, Epsilon);
    EXPECT_NEAR(tcpY.at(2)[1], 3.0, Epsilon);
    EXPECT_NEAR(tcpY.at(2)[2], 3.0, Epsilon);
  }

  auto robotZ = MultiRobot::MakeMultiRobot({robotY, robotD});
  {
    auto tcpZ = robotZ->GetFK(MakeState({1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0}));
    EXPECT_EQ(tcpZ.size(), 4u);

    EXPECT_NEAR(tcpZ.at(0)[0], 1.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(0)[1], 1.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(0)[2], 1.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(1)[0], 2.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(1)[1], 2.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(1)[2], 2.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(2)[0], 3.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(2)[1], 3.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(2)[2], 3.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(3)[0], 4.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(3)[1], 4.0, Epsilon);
    EXPECT_NEAR(tcpZ.at(3)[2], 4.0, Epsilon);
  }
}

TEST(MultiRobotCollisionCheckingTest, JointLimitTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto robotA = MakeRobot<SphereRobot>(world);
  auto robotB = MakeRobot<SphereRobot>(world);
  auto robotC = MakeRobot<SphereRobot>(world);
  auto robotD = MakeRobot<SphereRobot>(world);

  robotA->SetLimits(std::make_pair(State3d(0.0, 0.0, 0.0), State3d(0.0, 0.0, 0.0)));
  robotB->SetLimits(std::make_pair(State3d(0.0, 0.0, 0.0), State3d(0.0, 0.0, 0.0)));
  robotC->SetLimits(std::make_pair(State3d(0.0, 0.0, 0.0), State3d(0.0, 0.0, 0.0)));
  robotD->SetLimits(std::make_pair(State3d(0.0, 0.0, 0.0), State3d(0.0, 0.0, 0.0)));

  auto robotX = MultiRobot::MakeMultiRobot({robotA, robotB});
  auto robotY = MultiRobot::MakeMultiRobot({robotX, robotC});
  auto robotZ = MultiRobot::MakeMultiRobot({robotY, robotD});

  auto collision_checker_robotX = std::make_shared<MultiRobotCollisionChecker>(world, robotX);
  auto collision_checker_robotY = std::make_shared<MultiRobotCollisionChecker>(world, robotY);
  auto collision_checker_robotZ = std::make_shared<MultiRobotCollisionChecker>(world, robotZ);

  {
    auto eigen_state = MakeState({1.0, 1.0, 1.0, 2.0, 2.0, 2.0});
    auto state = robotX->GetSpaceInformation()->allocState();
    robotX->EigenToState(eigen_state, state);
    EXPECT_FALSE(collision_checker_robotX->isValid(state));
    robotX->GetSpaceInformation()->freeState(state);
  }

  {
    auto eigen_state = MakeState({1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0});
    auto state = robotY->GetSpaceInformation()->allocState();
    robotY->EigenToState(eigen_state, state);
    EXPECT_FALSE(collision_checker_robotY->isValid(state));
    robotY->GetSpaceInformation()->freeState(state);
  }

  {
    auto eigen_state = MakeState({1.0, 1.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0, 3.0, 4.0, 4.0, 4.0});
    auto state = robotZ->GetSpaceInformation()->allocState();
    robotZ->EigenToState(eigen_state, state);
    EXPECT_FALSE(collision_checker_robotZ->isValid(state));
    robotZ->GetSpaceInformation()->freeState(state);
  }
}
