#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "KinematicsSolver.hpp"
#include "robots/KukaSkeleton.hpp"

void CheckFrontAndBackConfigs(const KinematicsSolverPtr& kinematics_solver, const std::vector<Eigen::VectorXd>& configs, const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame) {
  auto maybe_config1_frame = kinematics_solver->solve_fk(configs.front());
  EXPECT_TRUE(maybe_config1_frame.has_value());
  auto maybe_config2_frame = kinematics_solver->solve_fk(configs.back());
  EXPECT_TRUE(maybe_config2_frame.has_value());

  auto config1_frame = maybe_config1_frame.value();
  auto config2_frame = maybe_config2_frame.value();
  std::cout << "Start Configuration " << configs.front() << "-> " << config1_frame << std::endl;
  std::cout << "Goal Configuration " << configs.back() << "-> " << config2_frame << std::endl;

  EXPECT_NEAR((config1_frame - start_frame).norm(), 0.0f, 1e-2);
  EXPECT_NEAR((config2_frame - goal_frame).norm(), 0.0f, 1e-2);
}

std::vector<Eigen::VectorXd> GetEdgeIKConfigs(const KinematicsSolverPtr& kinematics_solver, const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame) {
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

void CheckAlignedAxisEdgeIK(const KinematicsSolverPtr& kinematics_solver, const Eigen::Vector3d& start_frame, const Eigen::Vector3d& goal_frame, size_t axis_index) {

  auto configs = GetEdgeIKConfigs(kinematics_solver, start_frame, goal_frame);
  CheckFrontAndBackConfigs(kinematics_solver, configs, start_frame, goal_frame);

  for(const auto& config : configs) {
    auto maybe_config_frame = kinematics_solver->solve_fk(config);
    EXPECT_TRUE(maybe_config_frame.has_value());
    auto config_frame = maybe_config_frame.value();
    for(size_t k = 0; k < 3; k++) {
      if(k==axis_index) continue;
      EXPECT_NEAR(config_frame[k], goal_frame[k], 1e-1);
      EXPECT_NEAR(config_frame[k], start_frame[k], 1e-1);
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
    EXPECT_NEAR(d, 0.0f, 1e-1);
  }
}

TEST(KinematicsSolverTest, EdgeIKTest) {
  dart::dynamics::SkeletonPtr manipulator = createKukaSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  CheckAlignedAxisEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.3), Eigen::Vector3d(0.6, 0.1, 0.6), 2);
  CheckAlignedAxisEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.3), Eigen::Vector3d(0.6, 0.3, 0.3), 1);
  CheckAlignedAxisEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.2, 0.3), Eigen::Vector3d(0.4, 0.2, 0.3), 0);
  CheckLineSegmentEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.3), Eigen::Vector3d(0.4, 0.3, 0.3));
  CheckLineSegmentEdgeIK(kinematics_solver, Eigen::Vector3d(0.6, 0.1, 0.2), Eigen::Vector3d(0.4, 0.4, 0.5));
}
