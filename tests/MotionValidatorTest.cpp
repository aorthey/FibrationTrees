#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ompl/base/State.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>

#include "CollisionChecker.hpp"
#include "Common.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "TranslationTaskSpaceMotionValidator.hpp"
#include "TaskSpaceMultiRobotMotionValidator.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"

const size_t kMaxConnections = 20;
const float kLineAccuracy = 0.1;

class TestSphereRobot : public SphereRobot {
  public:
    TestSphereRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override {
      return createSphere(0.05);
    }
};

//Evaluate line segments where there is a collision mid-way. Check that the
//validator stops AND that all configurations are on a straight line in task
//space
TEST(MotionValidatorTest, EarlyStoppageLineDistanceTest) {
  dart::math::Random::setSeed(0);
  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto obstacle = MakeRobot<TestSphereRobot>(world);

  auto robot = MakeRobot<KukaRobotTaskSpace>(world, obstacle->GetSkeleton());

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor = robot->GetSpaceInformation();
  auto motion_validator = static_pointer_cast<ompl::multilevel::TaskSpaceMotionValidator>(factor->getMotionValidator());
  
  auto s1 = factor->allocState();
  auto s2 = factor->allocState();
  
  PrintSkeletonInfo(robot->GetSkeleton());
  robot->GetSpaceInformation()->printSettings(std::cout);
  for(size_t k = 0; k < kMaxConnections; k++) {
    std::cout << std::string(80,'-') << std::endl;
    auto random_s1 = GetRandomPosition(robot->GetSkeleton());
    auto random_s2 = GetRandomPosition(robot->GetSkeleton());
    robot->EigenToState(random_s1, s1);
    robot->EigenToState(random_s2, s2);
    factor->printState(s1);
    factor->printState(s2);
    const auto s1_tcp = GetFK(robot->GetSkeleton(), random_s1);
    const auto s2_tcp = GetFK(robot->GetSkeleton(), random_s2);

    ////////////////////////////////////////////////////////////////////////////////
    //Set obstacle to mid point along task space straight line
    ////////////////////////////////////////////////////////////////////////////////
    const auto mid = s1_tcp + 0.5 * (s2_tcp - s1_tcp);
    Eigen::VectorXd mid_config(3);
    mid_config << mid[0], mid[1], mid[2];
    obstacle->GetSkeleton()->setConfiguration(mid_config);

    auto configs = motion_validator->propagateMotion(s1, s2);
    if(configs.empty()) {
      std::cout << "Could not make progress on " << random_s1.format(CommaFmt) << std::endl;
      continue;
    }
    EXPECT_GT(configs.size(), 0);
    auto d_total = factor->distance(configs.front(), configs.back());
    std::cout << "Propagation Progress is " << d_total << "/" << factor->distance(s1, s2) << std::endl;
    EXPECT_LT(d_total, 0.5 * factor->distance(s1, s2));
    EXPECT_GE(d_total, 0.0);

    auto config = robot->StateToEigen(configs.back());
    std::cout << "Reached config " << config.format(CommaFmt) << std::endl;
    const auto s3_tcp = GetFK(robot->GetSkeleton(), config);

    std::cout << "  Tcp[start]   " << s1_tcp.format(CommaFmt) << std::endl;
    std::cout << "  Tcp[goal]    " << s2_tcp.format(CommaFmt) << std::endl;
    std::cout << "  Tcp[reached] " << s3_tcp.format(CommaFmt) << std::endl;
    std::cout << "  Tcp[mid]     " << mid.format(CommaFmt) << std::endl;
    std::cout << std::string(80,'-') << std::endl;
    auto d_mid = LineDistance(s1_tcp, s2_tcp, mid_config);
    auto d = LineDistance(s1_tcp, s2_tcp, s3_tcp);
    EXPECT_LT(d_mid, kLineAccuracy);
    EXPECT_LT(d, kLineAccuracy);
  }

  factor->freeState(s2);
  factor->freeState(s1);
}

TEST(MotionValidatorTest, MultiRobotCollisionTest) {
  dart::math::Random::setSeed(0);
  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto robot1 = MakeRobot<MobileKukaRobotTaskSpace>(world);
  auto robot2 = MakeRobot<MobileKukaRobotTaskSpace>(world);

  std::vector<RobotPtr> robots = {robot1, robot2};
  auto robot = MultiRobot::MakeMultiRobot(robots);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor = robot->GetSpaceInformation();

  auto pairwise_collision_checker = std::make_shared<DartMultiRobotCollisionChecker>(factor, world, robots);
  factor->setStateValidityChecker(pairwise_collision_checker);

  auto motion_validator = std::make_shared<TaskSpaceMultiRobotMotionValidator>(factor);
  factor->setMotionValidator(motion_validator);

  auto child1 = robot1->GetSpaceInformation();
  auto child2 = robot2->GetSpaceInformation();
  
  auto task_start1 = child1->allocState();
  auto task_start2 = child2->allocState();
  auto task_goal1 = child1->allocState();
  auto task_goal2 = child2->allocState();

  auto task_start1_eigen = MakeEigen({1.0, 0.0, 0.8});
  auto task_goal1_eigen = MakeEigen({0.0, 0.0, 0.8});
  auto task_start2_eigen = MakeEigen({-1.0, 0.0, 0.8);
  auto task_goal2_eigen = MakeEigen({0.0, 0.0, 0.8);

  robot1->EigenToState(task_start1_eigen, task_start1);
  robot1->EigenToState(task_goal1_eigen, task_goal1);
  robot2->EigenToState(task_start2_eigen, task_start2);
  robot2->EigenToState(task_goal2_eigen, task_goal2);

  std::unordered_map<std::string, ompl::base::State*> task_space_start_states;
  task_space_start_states[child1->getName()] = task_start1;
  task_space_start_states[child2->getName()] = task_start2;

  auto maybe_start = ComputeValidTotalState(factor, task_space_start_states);
  EXPECT_TRUE(maybe_start.has_value());
  auto start = maybe_start.value();
  factor->printState(start);

  std::unordered_map<std::string, ompl::base::State*> task_space_goal_states;
  task_space_goal_states[child1->getName()] = task_goal1;
  task_space_goal_states[child2->getName()] = task_goal2;

  auto maybe_goal = ComputeValidTotalState(factor, task_space_goal_states);
  EXPECT_TRUE(maybe_goal.has_value());
  auto goal = maybe_goal.value();
  factor->printState(goal);

  auto configs = motion_validator->propagateMotion(start, goal);
  EXPECT_GT(configs.size(), 0);
  auto d_total = factor->distance(configs.front(), configs.back());
  std::cout << "Propagation Progress is " << d_total << "/" << factor->distance(start, goal) << std::endl;
  auto config = robot->StateToEigen(configs.back());
  std::cout << "Reached config " << config.format(CommaFmt) << std::endl;

  auto tcps = robot->GetFK(configs.back());
  std::cout << "  Tcp for " << config.format(CommaFmt) << std::endl;
  for(const auto& tcp : tcps) {
    std::cout << "    " << tcp.format(CommaFmt) << std::endl;
  }

}
