#include <gtest/gtest.h>

#include "EigenPath.hpp"
#include "OmplHelper.hpp"
#include "Common.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "robots/RobotFactory.hpp"

const float kPathAccuracy = 1e-6;

TEST(EigenPathTest, EmptyTest) {
  std::vector<StateXd> configs;
  EXPECT_THROW(auto path = EigenPath(configs), std::length_error);
}

TEST(EigenPathTest, StraightLineTest) {
  auto v1 = MakeState({0.0, 0.0});
  auto v2 = MakeState({0.0, 1.0});
  auto v3 = MakeState({0.0, 2.0});
  auto v4 = MakeState({0.0, 3.0});
  std::vector<StateXd> configs = {v1, v2, v3, v4};

  EigenPath path(configs);

  auto x = path.GetConfigAt(0.0);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.0, kPathAccuracy);

  x = path.GetConfigAt(0.25);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.75, kPathAccuracy);

  x = path.GetConfigAt(0.5);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 1.5, kPathAccuracy);

  x = path.GetConfigAt(0.75);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 2.25, kPathAccuracy);

  x = path.GetConfigAt(1.0);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 3.0, kPathAccuracy);

  EXPECT_EQ(path.GetLength(), 3.0);
}

TEST(EigenPathTest, BendLineTest) {
  auto v1 = MakeState({0.0, 0.0});
  auto v2 = MakeState({1.0, 0.0});
  auto v3 = MakeState({1.0, 1.0});
  auto v4 = MakeState({2.0, 1.0});
  std::vector<StateXd> configs = {v1, v2, v3, v4};

  EigenPath path(configs);

  auto x = path.GetConfigAt(0.0);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.0, kPathAccuracy);

  x = path.GetConfigAt(0.25);
  EXPECT_NEAR(x[0], 0.75, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.0, kPathAccuracy);

  x = path.GetConfigAt(0.5);
  EXPECT_NEAR(x[0], 1.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.5, kPathAccuracy);

  x = path.GetConfigAt(0.75);
  EXPECT_NEAR(x[0], 1.25, kPathAccuracy);
  EXPECT_NEAR(x[1], 1.0, kPathAccuracy);

  x = path.GetConfigAt(1.0);
  EXPECT_NEAR(x[0], 2.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 1.0, kPathAccuracy);

  x = path.GetConfigAt(1.5);
  EXPECT_NEAR(x[0], 2.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 1.0, kPathAccuracy);
  EXPECT_EQ(path.GetLength(), 3.0);
}

TEST(EigenPathTest, SquareTest) {
  auto v1 = MakeState({0.0, 0.0});
  auto v2 = MakeState({1.0, 0.0});
  auto v3 = MakeState({1.0, 1.0});
  auto v4 = MakeState({0.0, 1.0});
  auto v5 = MakeState({0.0, 0.0});
  std::vector<StateXd> configs = {v1, v2, v3, v4, v5};

  EigenPath path(configs);

  auto x = path.GetConfigAt(0.0);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.0, kPathAccuracy);

  x = path.GetConfigAt(0.25);
  EXPECT_NEAR(x[0], 1.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.0, kPathAccuracy);

  x = path.GetConfigAt(0.5);
  EXPECT_NEAR(x[0], 1.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 1.0, kPathAccuracy);

  x = path.GetConfigAt(0.75);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 1.0, kPathAccuracy);

  x = path.GetConfigAt(1.0);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.0, kPathAccuracy);

  x = path.GetConfigAt(1.5);
  EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
  EXPECT_NEAR(x[1], 0.0, kPathAccuracy);
  EXPECT_EQ(path.GetLength(), 4.0);
}

TEST(EigenPathTest, ManipulatorTest) {
  auto v1 = MakeState({0, 0, 0, 0, 0, 0, 0.0613498, -1.03295, 0.0240684, 1.41603, -0.886764, 2.00979, -2.65123});
  auto v2 = MakeState({0, 0, 0, 0, 0, 0, -0.0361243, -1.04131, 0.174474, 1.41603, -1.00731, 1.93957, -2.70616});
  auto v3 = MakeState({0, 0, 0, 0, 0, 0, -1.0888, 1.5708, 1.72408, 2.08122, 0.869073, 1.57153, -2.01826});
  auto v4 = MakeState({0, 0, 0, 0, 0, 0, -0.964628, 1.5106, -1.41911, -1.58668, -2.21218, 1.18343, -2.52305});
  std::vector<StateXd> configs = {v1, v2, v3, v4};

  auto vmax1 = CwiseMax(v1, v2);
  auto vmax2 = CwiseMax(vmax1, v3);
  auto vmax = CwiseMax(vmax2, v4);

  auto vmin1 = CwiseMin(v1, v2);
  auto vmin2 = CwiseMin(vmin1, v3);
  auto vmin = CwiseMin(vmin2, v4);

  EigenPath path(configs);

  for(const auto& d : {0.0, 0.1, 0.3, 0.35, 0.5, 0.8, 1.1}) {
    auto x = path.GetConfigAt(d);
    for(size_t k = 0; k < x.size(); k++) {
      EXPECT_LE(x[k], vmax[k]);
      EXPECT_GE(x[k], vmin[k]);
    }
  }
}

TEST(EigenPathTest, DISABLED_SaveLoadTest) {
  auto v1 = MakeState({0, 0, 0, 0, 0, 0, 0.0613498, -1.03295, 0.0240684, 1.41603, -0.886764, 2.00979, -2.65123});
  auto v2 = MakeState({0, 0, 0, 0, 0, 0, -0.0361243, -1.04131, 0.174474, 1.41603, -1.00731, 1.93957, -2.70616});
  auto v3 = MakeState({0, 0, 0, 0, 0, 0, -1.0888, 1.5708, 1.72408, 2.08122, 0.869073, 1.57153, -2.01826});
  auto v4 = MakeState({0, 0, 0, 0, 0, 0, -0.964628, 1.5106, -1.41911, -1.58668, -2.21218, 1.18343, -2.52305});
  std::vector<StateXd> configs = {v1, v2, v3, v4};

  auto length = Distance(v1, v2) + Distance(v2, v3) + Distance(v3, v4);

  EigenPath path_saved(configs);
  path_saved.Save();
  EXPECT_NEAR(path_saved.GetLength(), length, 1e-5);

  EigenPath path_loaded = EigenPath::FromFile();
  EXPECT_NEAR(path_loaded.GetLength(), length, 1e-5);

  for(double d=0.0; d<1.0; d+=0.05) {
    auto q = path_saved.GetConfigAt(d);
    auto p = path_loaded.GetConfigAt(d);
    EXPECT_NEAR(Distance(q, p), 0.0, 1e-6);
  }
}

TEST(EigenPathTest, EigenSplitTest) {
  auto v = MakeState({0, 1, 2, 3, 4, 5});
  EXPECT_EQ(v.size(), 6);
  auto v1 = v.configuration.segment(0, 3);
  EXPECT_EQ(v1.size(), 3);
  auto v2 = v.configuration.segment(3, 3);
  EXPECT_EQ(v2.size(), 3);
}

TEST(EigenPathTest, EmptyOmplPath) {
  auto robot = MakeRobot<SphereRobot>();
  auto si = robot->GetSpaceInformation();
  auto ompl_path = std::make_shared<ompl::geometric::PathGeometric>(si);
  EXPECT_THROW(auto path = EigenPath(robot, ompl_path), std::length_error);
}

TEST(EigenPathTest, OmplPathWithoutTiming) {
  auto robot = MakeRobot<SphereRobot>();
  auto si = robot->GetSpaceInformation();

  auto space = si->getStateSpace();
  ompl::base::ScopedState<> t1(space);
  t1[0] = 0;
  t1[1] = 0;
  t1[2] = 0;

  ompl::base::ScopedState<> t2(space);
  t2[0] = 0;
  t2[1] = 0.5;
  t2[2] = 0.0;

  ompl::base::ScopedState<> t3(space);
  t3[0] = 0;
  t3[1] = 1.0;
  t3[2] = 0.0;

  auto ompl_path = std::make_shared<ompl::geometric::PathGeometric>(si);
  ompl_path->append(t1.get());
  ompl_path->append(t2.get());
  ompl_path->append(t3.get());

  auto path = EigenPath(robot, ompl_path);

  {
    auto x = path.GetConfigAt(0.0);
    EXPECT_EQ(x.size(), 3u);
    EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
    EXPECT_NEAR(x[1], 0.0, kPathAccuracy);
    EXPECT_NEAR(x[2], 0.0, kPathAccuracy);
  }

  {
    auto x = path.GetConfigAt(0.5);
    EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
    EXPECT_NEAR(x[1], 0.5, kPathAccuracy);
    EXPECT_NEAR(x[2], 0.0, kPathAccuracy);
  }

  {
    auto x = path.GetConfigAt(1.0);
    EXPECT_NEAR(x[0], 0.0, kPathAccuracy);
    EXPECT_NEAR(x[1], 1.0, kPathAccuracy);
    EXPECT_NEAR(x[2], 0.0, kPathAccuracy);
  }
}


TEST(EigenPathTest, OmplPathWithTiming) {
  auto robot = MakeRobot<TimeBasedMobileKukaRobotTaskSpace>();
  auto si = robot->GetSpaceInformation();
  auto space = si->getStateSpace();

  auto ompl_path = std::make_shared<ompl::geometric::PathGeometric>(si);

  ompl::base::ScopedState<> t1(space);
  auto v1 = MakeState({0, 0, 0, 0});
  v1.time = 0.0;
  robot->EigenToState(v1, t1.get());
  ompl_path->append(t1.get());

  ompl::base::ScopedState<> t2(space);
  auto v2 = MakeState({0, 1, 0, 0});
  v2.time = 2.0;
  robot->EigenToState(v2, t2.get());
  ompl_path->append(t2.get());

  ompl::base::ScopedState<> t3(space);
  auto v3 = MakeState({0, 2, 0, 0});
  v3.time = 10.0;
  robot->EigenToState(v3, t3.get());
  ompl_path->append(t3.get());

  auto path = EigenPath(robot, ompl_path);
  {
    auto x = path.GetConfigAt(0.0);
    EXPECT_NEAR(x.time, 0.0, Epsilon);
    EXPECT_NEAR(x[0], 0.0, Epsilon);
    EXPECT_NEAR(x[1], 0.0, Epsilon);
  }
  {
    auto x = path.GetConfigAt(0.1);
    EXPECT_NEAR(x.time, 1.0, Epsilon);
    EXPECT_NEAR(x[0], 0.0, Epsilon);
    EXPECT_NEAR(x[1], 0.5, Epsilon);
  }
  {
    auto x = path.GetConfigAt(0.2);
    EXPECT_NEAR(x.time, 2.0, Epsilon);
    EXPECT_NEAR(x[0], 0.0, Epsilon);
    EXPECT_NEAR(x[1], 1.0, Epsilon);
  }
  {
    auto x = path.GetConfigAt(0.6);
    EXPECT_NEAR(x.time, 6.0, Epsilon);
    EXPECT_NEAR(x[0], 0.0, Epsilon);
    EXPECT_NEAR(x[1], 1.5, Epsilon);
  }
  {
    auto x = path.GetConfigAt(1.1);
    EXPECT_NEAR(x.time, 10.0, Epsilon);
    EXPECT_NEAR(x[0], 0.0, Epsilon);
    EXPECT_NEAR(x[1], 2.0, Epsilon);
  }
}
