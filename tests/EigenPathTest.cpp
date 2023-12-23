#include <gtest/gtest.h>

#include "EigenPath.hpp"
#include "Common.hpp"

TEST(EigenPathTest, EmptyTest) {
  std::vector<Eigen::VectorXd> configs;
  EigenPath path(configs);
  path.GetConfigAt(0.0f);
}

TEST(EigenPathTest, StraightLineTest) {
  Eigen::VectorXd v1(2); v1[0]=0.0; v1[1]=0.0;
  Eigen::VectorXd v2(2); v2[0]=0.0; v2[1]=1.0;
  Eigen::VectorXd v3(2); v3[0]=0.0; v3[1]=2.0;
  Eigen::VectorXd v4(2); v4[0]=0.0; v4[1]=3.0;
  std::vector<Eigen::VectorXd> configs = {v1, v2, v3, v4};

  EigenPath path(configs);

  auto x = path.GetConfigAt(0.0);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 0.0, Epsilon);

  x = path.GetConfigAt(0.25);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 0.75, Epsilon);

  x = path.GetConfigAt(0.5);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 1.5, Epsilon);

  x = path.GetConfigAt(0.75);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 2.25, Epsilon);

  x = path.GetConfigAt(1.0);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 3.0, Epsilon);

  EXPECT_EQ(path.GetLength(), 3.0);
}

TEST(EigenPathTest, BendLineTest) {
  Eigen::VectorXd v1(2); v1[0]=0.0; v1[1]=0.0;
  Eigen::VectorXd v2(2); v2[0]=1.0; v2[1]=0.0;
  Eigen::VectorXd v3(2); v3[0]=1.0; v3[1]=1.0;
  Eigen::VectorXd v4(2); v4[0]=2.0; v4[1]=1.0;
  std::vector<Eigen::VectorXd> configs = {v1, v2, v3, v4};

  EigenPath path(configs);

  auto x = path.GetConfigAt(0.0);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 0.0, Epsilon);

  x = path.GetConfigAt(0.25);
  EXPECT_NEAR(x[0], 0.75, Epsilon);
  EXPECT_NEAR(x[1], 0.0, Epsilon);

  x = path.GetConfigAt(0.5);
  EXPECT_NEAR(x[0], 1.0, Epsilon);
  EXPECT_NEAR(x[1], 0.5, Epsilon);

  x = path.GetConfigAt(0.75);
  EXPECT_NEAR(x[0], 1.25, Epsilon);
  EXPECT_NEAR(x[1], 1.0, Epsilon);

  x = path.GetConfigAt(1.0);
  EXPECT_NEAR(x[0], 2.0, Epsilon);
  EXPECT_NEAR(x[1], 1.0, Epsilon);

  x = path.GetConfigAt(1.5);
  EXPECT_NEAR(x[0], 2.0, Epsilon);
  EXPECT_NEAR(x[1], 1.0, Epsilon);
  EXPECT_EQ(path.GetLength(), 3.0);
}

TEST(EigenPathTest, SquareTest) {
  Eigen::VectorXd v1(2); v1[0]=0.0; v1[1]=0.0;
  Eigen::VectorXd v2(2); v2[0]=1.0; v2[1]=0.0;
  Eigen::VectorXd v3(2); v3[0]=1.0; v3[1]=1.0;
  Eigen::VectorXd v4(2); v4[0]=0.0; v4[1]=1.0;
  Eigen::VectorXd v5(2); v5[0]=0.0; v5[1]=0.0;
  std::vector<Eigen::VectorXd> configs = {v1, v2, v3, v4, v5};

  EigenPath path(configs);

  auto x = path.GetConfigAt(0.0);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 0.0, Epsilon);

  x = path.GetConfigAt(0.25);
  EXPECT_NEAR(x[0], 1.0, Epsilon);
  EXPECT_NEAR(x[1], 0.0, Epsilon);

  x = path.GetConfigAt(0.5);
  EXPECT_NEAR(x[0], 1.0, Epsilon);
  EXPECT_NEAR(x[1], 1.0, Epsilon);

  x = path.GetConfigAt(0.75);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 1.0, Epsilon);

  x = path.GetConfigAt(1.0);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 0.0, Epsilon);

  x = path.GetConfigAt(1.5);
  EXPECT_NEAR(x[0], 0.0, Epsilon);
  EXPECT_NEAR(x[1], 0.0, Epsilon);
  EXPECT_EQ(path.GetLength(), 4.0);
}

TEST(EigenPathTest, ManipulatorTest) {
  Eigen::VectorXd v1(13), v2(13), v3(13), v4(13);
  v1 << 0, 0, 0, 0, 0, 0, 0.0613498, -1.03295, 0.0240684, 1.41603, -0.886764, 2.00979, -2.65123;
  v2 << 0, 0, 0, 0, 0, 0, -0.0361243, -1.04131, 0.174474, 1.41603, -1.00731, 1.93957, -2.70616;
  v3 << 0, 0, 0, 0, 0, 0, -1.0888, 1.5708, 1.72408, 2.08122, 0.869073, 1.57153, -2.01826;
  v4 << 0, 0, 0, 0, 0, 0, -0.964628, 1.5106, -1.41911, -1.58668, -2.21218, 1.18343, -2.52305;
  std::vector<Eigen::VectorXd> configs = {v1, v2, v3, v4};

  auto vmax1 = v1.cwiseMax(v2);
  auto vmax2 = vmax1.cwiseMax(v3);
  auto vmax = vmax2.cwiseMax(v4);

  auto vmin1 = v1.cwiseMin(v2);
  auto vmin2 = vmin1.cwiseMin(v3);
  auto vmin = vmin2.cwiseMin(v4);

  EigenPath path(configs);

  for(const auto& d : {0.0, 0.1, 0.3, 0.35, 0.5, 0.8, 1.1}) {
    auto x = path.GetConfigAt(d);
    for(size_t k = 0; k < x.size(); k++) {
      EXPECT_LE(x[k], vmax[k]);
      EXPECT_GE(x[k], vmin[k]);
    }
  }
}

TEST(EigenPathTest, SaveLoadTest) {
  Eigen::VectorXd v1(13), v2(13), v3(13), v4(13);
  v1 << 0, 0, 0, 0, 0, 0, 0.0613498, -1.03295, 0.0240684, 1.41603, -0.886764, 2.00979, -2.65123;
  v2 << 0, 0, 0, 0, 0, 0, -0.0361243, -1.04131, 0.174474, 1.41603, -1.00731, 1.93957, -2.70616;
  v3 << 0, 0, 0, 0, 0, 0, -1.0888, 1.5708, 1.72408, 2.08122, 0.869073, 1.57153, -2.01826;
  v4 << 0, 0, 0, 0, 0, 0, -0.964628, 1.5106, -1.41911, -1.58668, -2.21218, 1.18343, -2.52305;
  std::vector<Eigen::VectorXd> configs = {v1, v2, v3, v4};

  auto length = (v1 - v2).norm() + (v2 - v3).norm() + (v3 - v4).norm();

  EigenPath path_saved(configs);
  path_saved.Save();
  EXPECT_NEAR(path_saved.GetLength(), length, 1e-5);

  EigenPath path_loaded = EigenPath::FromFile();
  EXPECT_NEAR(path_loaded.GetLength(), length, 1e-5);

  for(double d=0.0; d<1.0; d+=0.05) {
    auto q = path_saved.GetConfigAt(d);
    auto p = path_loaded.GetConfigAt(d);
    EXPECT_NEAR((q-p).norm(), 0.0, 1e-6);
  }
}

