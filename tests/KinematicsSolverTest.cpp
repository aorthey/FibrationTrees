#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "KinematicsSolver.hpp"
#include "DartHelper.hpp"
#include "Common.hpp"
#include "robots/KukaRobot.hpp"
#include "robots/RobotFactory.hpp"

const float kAccuracyStraightLine = 1e-1; //Staying along the straight line path
const float kAccuracyGoal = 1e-2; //Reaching goal frame
const size_t kNumberRandomConfigs = 10;

bool CheckStraightLineAccuracy(const KinematicsSolverPtr& kinematics_solver, 
    const std::vector<Eigen::VectorXd>& configs) {

  if(configs.size() < 2) {
    return true;
  }

  const auto maybe_config1_frame = kinematics_solver->solve_fk(configs.front());
  EXPECT_TRUE(maybe_config1_frame.has_value());
  const auto maybe_config2_frame = kinematics_solver->solve_fk(configs.back());
  EXPECT_TRUE(maybe_config2_frame.has_value());

  const auto start_frame = maybe_config1_frame.value();
  const auto goal_frame = maybe_config2_frame.value();

  double max_error = 0.0;
  for(const auto& config : configs) {
    auto maybe_config_frame = kinematics_solver->solve_fk(config);
    if(!maybe_config_frame.has_value()) {
      std::cout << "Could not solve FK for " << config.format(CommaFmt) << std::endl;
    }
    EXPECT_TRUE(maybe_config_frame.has_value());
    auto config_frame = maybe_config_frame.value();
    auto d = LineDistance(start_frame, goal_frame, config_frame);
    max_error = std::max(max_error, d);
  }

  EXPECT_NEAR(max_error, 0.0f, kAccuracyStraightLine);
  if(max_error > kAccuracyStraightLine) {
    std::cout << "Maximal error deviation from line : " << max_error << std::endl;
    std::cout << "start << " << configs.front().format(CommaFmt) << std::endl;
    std::cout << "goal  << " << configs.back().format(CommaFmt) << std::endl;
    exit(0);
  }
  return true;
}


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

void EdgeIKTestFromFrames(const KinematicsSolverPtr& kinematics_solver, 
    const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame) {
  dart::math::Random::setSeed(0);
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Start Tcp (Desired): " << start_frame.format(CommaFmt) << std::endl;
  std::cout << "Goal Tcp (Desired): " << goal_frame.format(CommaFmt) << std::endl;

  const size_t kMaxResampleIterations = 100;

  auto maybe_start_ik = kinematics_solver->solve_ik(start_frame, kMaxResampleIterations);
  EXPECT_TRUE(maybe_start_ik.has_value());
  auto start = maybe_start_ik.value();

  auto maybe_goal_ik = kinematics_solver->solve_ik(goal_frame, kMaxResampleIterations);
  EXPECT_TRUE(maybe_goal_ik.has_value());
  auto goal = maybe_goal_ik.value();

  const auto maybe_config1_frame = kinematics_solver->solve_fk(start);
  EXPECT_TRUE(maybe_config1_frame.has_value());
  const auto maybe_config2_frame = kinematics_solver->solve_fk(goal);
  EXPECT_TRUE(maybe_config2_frame.has_value());

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
  EXPECT_GE(configs.size(), 1u);

  std::cout << "Start (Desired): " << start.format(CommaFmt) << ", Goal (Desired): " << goal.format(CommaFmt) << std::endl;
  std::cout << "Start (Actual) : " << configs.front().format(CommaFmt) << ", Goal (Actual) : " << configs.back().format(CommaFmt) << std::endl;
  EXPECT_NEAR((start - configs.front()).norm(), 0.0f, Epsilon);
  EXPECT_TRUE(CheckStraightLineAccuracy(kinematics_solver, configs));
}

TEST(KinematicsSolverTest, ConfigEdgeIKSameConfigTest) {
  auto robot = MakeRobot<KukaRobot>();
  auto manipulator = robot->GetSkeleton();
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);
  dart::math::Random::setSeed(0);

  const auto N = manipulator->getNumDofs();
  Eigen::VectorXd start(N);
  start << 1.61707, -0.713562, 1.95916, -1.70716, 0.124328, -0.775085, 2.52864;

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, start);
  EXPECT_EQ(configs.size(), 0u);
}

TEST(KinematicsSolverTest, ConfigEdgeIKSameFrameTest) {
  auto robot = MakeRobot<KukaRobot>();
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());

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
  EXPECT_GT(configs.size(), 2u);
  std::cout << "Start (Desired) : " << start.format(CommaFmt) << std::endl;
  std::cout << "Goal  (Desired) : " << goal.format(CommaFmt) << std::endl;
  CheckFrontAndBackConfigs(kinematics_solver, configs, start_frame, start_frame);
  std::cout << "Found " << configs.size() << " configs for edge IK." << std::endl;
}

//Goal has no FK solution -> Cannot make progress
TEST(KinematicsSolverTest, NonFKConfigTest) {
  auto robot = MakeRobot<KukaRobot>();
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  Eigen::VectorXd start(7);
  Eigen::VectorXd goal(7);
  start << -0.989524, -0.200034, -1.2738, -1.28626, 1.61383, -0.17553, -0.8159;
  goal << 2.78314, 1.41906, -2.96706, -1.4611, -2.07017, 1.40266, 1.89722; //Does not have FK solution
  auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
  EXPECT_EQ(configs.size(), 0u);
  EXPECT_FALSE(kinematics_solver->lastSolveWasSuccessful());
}

// Straight line moves outside joint limits
TEST(KinematicsSolverTest, CannotReachGoalConfigTest) {
  auto robot = MakeRobot<KukaRobot>();
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  Eigen::VectorXd start(7);
  Eigen::VectorXd goal(7);
  start << -0.503705, -1.08742, -1.14191, -1.98434, 0.363036, -1.08686, -0.0262207;
  goal << -1.39498, -1.55469, 2.28343, 1.95646, -1.09705, -0.305218, -1.31804;

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
  EXPECT_GE(configs.size(), 2u);

  EXPECT_TRUE(CheckStraightLineAccuracy(kinematics_solver, configs));
}

TEST(KinematicsSolverTest, JointFlipTest) {
  auto robot = MakeRobot<KukaRobot>();
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
  Eigen::VectorXd start(7);
  Eigen::VectorXd goal(7);
  start << -1.06341,0.0364156,-1.5652,-1.36632,2.24109,0.174882,-0.205112;
  goal  << 1.28814,-0.0755119,0.874968,-1.04689,1.22978,-0.116029,0.427566;

  auto maybe_start_frame = kinematics_solver->solve_fk(start);
  EXPECT_TRUE(maybe_start_frame.has_value());
  auto maybe_goal_frame = kinematics_solver->solve_fk(goal);
  EXPECT_TRUE(maybe_goal_frame.has_value());
  auto start_frame = maybe_start_frame.value();
  auto goal_frame = maybe_goal_frame.value();

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
  EXPECT_GT(configs.size(), 2u);
  //CheckFrontAndBackConfigs(kinematics_solver, configs, start_frame, goal_frame);
  EXPECT_TRUE(CheckStraightLineAccuracy(kinematics_solver, configs));
}

TEST(KinematicsSolverTest, RandomizedConfigsTest) {
  auto robot = MakeRobot<KukaRobot>();
  auto manipulator = robot->GetSkeleton();
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  for(size_t k = 0; k < kNumberRandomConfigs; k++) {
    auto start = GetRandomPosition(manipulator);
    auto goal = GetRandomPosition(manipulator);
    auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
    EXPECT_GE(configs.size(), 1u);
    EXPECT_TRUE(CheckStraightLineAccuracy(kinematics_solver, configs));
  }
}

TEST(KinematicsSolverTest, StraightLineTaskSpaceTest) {
  auto robot = MakeRobot<KukaRobot>();
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());

  EdgeIKTestFromFrames(kinematics_solver, Eigen::Vector3d(0.7, 0.2, 0.3), Eigen::Vector3d(0.5, 0.2, 0.3));
  EdgeIKTestFromFrames(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.3), Eigen::Vector3d(0.6, 0.3, 0.3));
  EdgeIKTestFromFrames(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.3), Eigen::Vector3d(0.4, 0.5, 0.3));
  EdgeIKTestFromFrames(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.2), Eigen::Vector3d(0.4, 0.4, 0.5));
  EdgeIKTestFromFrames(kinematics_solver, Eigen::Vector3d(0.4, 0.4, 0.3), Eigen::Vector3d(0.4, 0.4, 0.6));
  EdgeIKTestFromFrames(kinematics_solver, Eigen::Vector3d(0.3, 0.4, 0.2), Eigen::Vector3d(0.6, 0.4, 0.7));
}

