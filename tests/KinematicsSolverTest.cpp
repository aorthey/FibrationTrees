#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "KinematicsSolver.hpp"
#include "Common.hpp"
#include "robots/KukaSkeleton.hpp"

const float kAccuracy = 1e-1; //Staying along the straight line path
const float kAccuracyGoal = 1e-1; //Reaching goal frame

void CheckFrontAndBackConfigs(const KinematicsSolverPtr& kinematics_solver, const std::vector<Eigen::VectorXd>& configs, const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame) {
  auto maybe_config1_frame = kinematics_solver->solve_fk(configs.front());
  EXPECT_TRUE(maybe_config1_frame.has_value());
  auto maybe_config2_frame = kinematics_solver->solve_fk(configs.back());
  EXPECT_TRUE(maybe_config2_frame.has_value());

  auto config1_frame = maybe_config1_frame.value();
  auto config2_frame = maybe_config2_frame.value();

  // std::cout << "Start Configuration " << configs.front().format(CommaFmt) << " -> Tcp " << config1_frame.format(CommaFmt) << std::endl;
  // std::cout << "Goal Configuration " << configs.back().format(CommaFmt) << " -> Tcp " << config2_frame.format(CommaFmt) << std::endl;
  std::cout << "Start Tcp (Desired): " << start_frame.format(CommaFmt) << std::endl;
  std::cout << "Start Tcp (Actual) : " << config1_frame.format(CommaFmt) << std::endl;
  std::cout << "Goal Tcp (Desired): " << goal_frame.format(CommaFmt) << std::endl;
  std::cout << "Goal Tcp (Actual) : " << config2_frame.format(CommaFmt) << std::endl;

  EXPECT_NEAR((config1_frame - start_frame).norm(), 0.0f, kAccuracyGoal);
  EXPECT_NEAR((config2_frame - goal_frame).norm(), 0.0f, kAccuracyGoal);
}

std::vector<Eigen::VectorXd> GetEdgeIKConfigs(const KinematicsSolverPtr& kinematics_solver, 
    const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame) {
  dart::math::Random::setSeed(0);

  const size_t kMaxResampleIterations = 100;
  auto maybe_start_ik = kinematics_solver->solve_ik(start_frame, kMaxResampleIterations);
  EXPECT_TRUE(maybe_start_ik.has_value());

  auto start = maybe_start_ik.value();

  auto configs = kinematics_solver->solve_edge_ik(start, goal_frame);
  EXPECT_GT(configs.size(), 0u);
  std::cout << "Found edge IK with " << configs.size() << " configs." << std::endl;
  return configs;
}

void CheckAlignedAxisEdgeIK(const KinematicsSolverPtr& kinematics_solver, 
    const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame, const size_t axis_index) {

  auto configs = GetEdgeIKConfigs(kinematics_solver, start_frame, goal_frame);
  CheckFrontAndBackConfigs(kinematics_solver, configs, start_frame, goal_frame);

  for(const auto& config : configs) {
    auto maybe_config_frame = kinematics_solver->solve_fk(config);
    EXPECT_TRUE(maybe_config_frame.has_value());
    auto config_frame = maybe_config_frame.value();
    for(size_t k = 0; k < 3; k++) {
      if(k==axis_index) {
        continue;
      }
      EXPECT_NEAR(config_frame[k], goal_frame[k], kAccuracy);
      EXPECT_NEAR(config_frame[k], start_frame[k], kAccuracy);
    }
  }
}

void CheckLineSegmentEdgeIK(const KinematicsSolverPtr& kinematics_solver, const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame) {
  auto configs = GetEdgeIKConfigs(kinematics_solver, start_frame, goal_frame);

  CheckFrontAndBackConfigs(kinematics_solver, configs, start_frame, goal_frame);

  const auto& a = start_frame;
  const auto& b = goal_frame;
  const auto& n = (b-a).normalized();

  std::cout << "Checking " << configs.size() << " configs." << std::endl;
  for(const auto& config : configs) {
    auto maybe_config_frame = kinematics_solver->solve_fk(config);
    EXPECT_TRUE(maybe_config_frame.has_value());
    //Check that points are near line segment between start and goal
    auto p = maybe_config_frame.value();
    auto s = (p-a).dot(n);
    auto d = ((p-a)-s*n).norm();
    EXPECT_NEAR(d, 0.0f, kAccuracy);
  }
}

TEST(KinematicsSolverTest, EdgeIKTest) {
  dart::dynamics::SkeletonPtr manipulator = 
    createKukaSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  CheckAlignedAxisEdgeIK(kinematics_solver, Eigen::Vector3d(0.4, 0.4, 0.3), Eigen::Vector3d(0.4, 0.4, 0.6), 2);
  CheckAlignedAxisEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.3), Eigen::Vector3d(0.6, 0.3, 0.3), 1);
  CheckAlignedAxisEdgeIK(kinematics_solver, Eigen::Vector3d(0.7, 0.2, 0.3), Eigen::Vector3d(0.5, 0.2, 0.3), 0);

  CheckAlignedAxisEdgeIK(kinematics_solver, Eigen::Vector3d(0.7, 0.2, 0.3), Eigen::Vector3d(0.5, 0.2, 0.3), 0);

  CheckLineSegmentEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.3), Eigen::Vector3d(0.4, 0.3, 0.3));
  CheckLineSegmentEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.2), Eigen::Vector3d(0.4, 0.4, 0.5));
}

TEST(KinematicsSolverTest, SingularityEdgeIKTest) {
  dart::dynamics::SkeletonPtr manipulator = 
    createKukaSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  const auto kStartFrame = Eigen::Vector3d(0.6, 0.2, 0.3);
  const auto kGoalFrame =  Eigen::Vector3d(0.3, 0.2, 0.3);

  auto configs = GetEdgeIKConfigs(kinematics_solver, kStartFrame, kGoalFrame);
  EXPECT_GT(configs.size(), 1u);
  EXPECT_FALSE(kinematics_solver->lastSolveWasSuccessful());

  auto maybe_config1_frame = kinematics_solver->solve_fk(configs.front());
  EXPECT_TRUE(maybe_config1_frame.has_value());
  auto config1_frame = maybe_config1_frame.value();
  EXPECT_NEAR((config1_frame - kStartFrame).norm(), 0.0f, kAccuracyGoal);
}

TEST(KinematicsSolverTest, ConfigEdgeIKTest) {
  dart::dynamics::SkeletonPtr manipulator = 
    createKukaSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  dart::math::Random::setSeed(0);

  const auto start_frame = Eigen::Vector3d(0.6, 0.1, 0.2);
  const auto goal_frame = Eigen::Vector3d(0.4, 0.4, 0.5);
  const size_t kMaxResampleIterations = 100;

  auto maybe_start_ik = kinematics_solver->solve_ik(start_frame, kMaxResampleIterations);
  EXPECT_TRUE(maybe_start_ik.has_value());
  auto start = maybe_start_ik.value();

  auto maybe_goal_ik = kinematics_solver->solve_ik(goal_frame, kMaxResampleIterations);
  EXPECT_TRUE(maybe_goal_ik.has_value());
  auto goal = maybe_goal_ik.value();

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
  EXPECT_GE(configs.size(), 2u);

  std::cout << "Start (Desired): " << start.format(CommaFmt) << ", Goal (Desired): " << goal.format(CommaFmt) << std::endl;
  std::cout << "Start (Actual) : " << configs.front().format(CommaFmt) << ", Goal (Actual) : " << configs.back().format(CommaFmt) << std::endl;
  EXPECT_NEAR((start - configs.front()).norm(), 0.0f, Epsilon);
  // EXPECT_NEAR((goal - configs.back()).norm(), 0.0f, Epsilon);
  CheckFrontAndBackConfigs(kinematics_solver, configs, start_frame, goal_frame);
}

TEST(KinematicsSolverTest, ConfigEdgeIKSameConfigTest) {
  dart::dynamics::SkeletonPtr manipulator = 
    createKukaSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);
  dart::math::Random::setSeed(0);

  const auto N = manipulator->getNumDofs();
  Eigen::VectorXd start(N);
  start << 1.61707, -0.713562, 1.95916, -1.70716, 0.124328, -0.775085, 2.52864;

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, start);
  EXPECT_EQ(configs.size(), 0u);
}

TEST(KinematicsSolverTest, ConfigEdgeIKSameFrameTest) {
  dart::dynamics::SkeletonPtr manipulator = 
    createKukaSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  dart::math::Random::setSeed(0);

  const auto start_frame = Eigen::Vector3d(0.4, 0.4, 0.5);

  const size_t kMaxResampleIterations = 100;
  auto maybe_start_ik = kinematics_solver->solve_ik(start_frame, kMaxResampleIterations);
  EXPECT_TRUE(maybe_start_ik.has_value());
  auto start = maybe_start_ik.value();

  auto maybe_goal_ik = kinematics_solver->solve_ik(start_frame, kMaxResampleIterations);
  EXPECT_TRUE(maybe_goal_ik.has_value());
  auto goal = maybe_goal_ik.value();

  EXPECT_GT((start-goal).norm(), Epsilon);

  const auto maybe_tcp_start = kinematics_solver->solve_fk(start);
  EXPECT_TRUE(maybe_tcp_start.has_value());
  auto start_tcp = maybe_tcp_start.value();

  const auto maybe_tcp_goal = kinematics_solver->solve_fk(goal);
  EXPECT_TRUE(maybe_tcp_goal.has_value());
  auto goal_tcp = maybe_tcp_goal.value();

  EXPECT_NEAR((start_tcp - start_frame).norm(), 0.0f, kAccuracyGoal);
  EXPECT_NEAR((goal_tcp - start_frame).norm(), 0.0f, kAccuracyGoal);

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
  // EXPECT_FALSE(kinematics_solver->lastSolveWasSuccessful());
  EXPECT_GT(configs.size(), 2u);
}
