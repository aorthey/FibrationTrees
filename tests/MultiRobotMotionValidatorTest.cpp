#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>

#include "CollisionChecker.hpp"
#include "Common.hpp"
#include "EigenPath.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "validators/MotionValidatorTaskSpaceTranslation.hpp"
#include "projections/TaskSpaceProjection.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"

const float kIKSolutionAccuracy = 1e-5;

void CheckMultiRobotEdge(
  const StateXd& task_start1_eigen,
  const StateXd& task_goal1_eigen,
  const StateXd& task_start2_eigen,
  const StateXd& task_goal2_eigen) {

  dart::math::Random::setSeed(0);
  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto robot1 = MakeRobot<MobileKukaRobotTaskSpace>(world);
  auto robot2 = MakeRobot<MobileKukaRobotTaskSpace>(world);

  std::vector<RobotPtr> robots = {robot1, robot2};
  auto multi_robot = MultiRobot::MakeMultiRobot(robots);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  //
  //           factor (both robots)
  //           /                 \
  //         /                     \
  //     child1 (robot1)        child2 (robot2)
  //        |                        |
  //  grand_child1 (point1)    grand_child2 (point2)
  //
  ////////////////////////////////////////////////////////////////////////////////
  auto factor = multi_robot->GetSpaceInformation();

  auto child1 = robot1->GetSpaceInformation();
  auto child2 = robot2->GetSpaceInformation();

  ////////////////////////////////////////////////////////////////////////////////
  ////Add task space projections
  ////////////////////////////////////////////////////////////////////////////////
  auto point1 = MakeRobot<SphereRobot>(world);
  const auto limits = std::make_pair(State3d(-1.5, -0.1, 0.0), State3d(1.5, 0.1, 2.0));
  point1->SetLimits(limits);
  auto grand_child1 = point1->GetSpaceInformation();
  auto projection1 = std::make_shared<TaskSpaceProjection>(child1, grand_child1, robot1);
  child1->addChild(grand_child1, projection1);

  auto point2 = MakeRobot<SphereRobot>(world);
  point2->SetLimits(limits);
  auto grand_child2 = point2->GetSpaceInformation();
  auto projection2 = std::make_shared<TaskSpaceProjection>(child2, grand_child2, robot2);
  child2->addChild(grand_child2, projection2);
  ////////////////////////////////////////////////////////////////////////////////
  ////Add subspace projections
  ////////////////////////////////////////////////////////////////////////////////
  auto projection_to_1 = std::make_shared<ompl::multilevel::SubspaceProjection>(factor, child1, 0);
  auto projection_to_2 = std::make_shared<ompl::multilevel::SubspaceProjection>(factor, child2, 1);

  bool computer_fiber_space = false;
  EXPECT_TRUE(factor->addChild(child1, projection_to_1, computer_fiber_space));
  EXPECT_TRUE(factor->addChild(child2, projection_to_2, computer_fiber_space));

  auto pairwise_collision_checker = std::make_shared<MultiRobotCollisionChecker>(world, multi_robot);
  factor->setStateValidityChecker(pairwise_collision_checker);

  auto motion_validator = std::make_shared<MotionValidatorTaskSpaceMultiRobot>(factor, multi_robot);
  factor->setMotionValidator(motion_validator);

  auto task_start1 = grand_child1->allocState();
  auto task_start2 = grand_child2->allocState();
  auto task_goal1 = grand_child1->allocState();
  auto task_goal2 = grand_child2->allocState();

  point1->EigenToState(task_start1_eigen, task_start1);
  point1->EigenToState(task_goal1_eigen, task_goal1);
  point2->EigenToState(task_start2_eigen, task_start2);
  point2->EigenToState(task_goal2_eigen, task_goal2);

  ////////////////////////////////////////////////////////////////////////////////
  // Compute IK solution for start
  ////////////////////////////////////////////////////////////////////////////////
  std::unordered_map<std::string, ompl::base::State*> task_space_start_states;
  task_space_start_states[grand_child1->getName()] = task_start1;
  task_space_start_states[grand_child2->getName()] = task_start2;

  auto maybe_start = ComputeValidTotalState(factor, task_space_start_states);
  EXPECT_TRUE(maybe_start.has_value());
  auto start = maybe_start.value();
  std::cout << "Found start IK solution" << std::endl;
  factor->printState(start);

  auto tcp_start = multi_robot->Robot::GetFK(start);
  EXPECT_EQ(tcp_start.size(), 2u);
  std::cout << "Tcp start[1] " << tcp_start.at(0) << std::endl;
  std::cout << "Tcp start[2] " << tcp_start.at(1) << std::endl;
  EXPECT_NEAR(Distance(tcp_start.at(0), task_start1_eigen.configuration), 0.0, kIKSolutionAccuracy);
  EXPECT_NEAR(Distance(tcp_start.at(1), task_start2_eigen.configuration), 0.0, kIKSolutionAccuracy);

  ////////////////////////////////////////////////////////////////////////////////
  // Compute IK solution for goal
  ////////////////////////////////////////////////////////////////////////////////
  std::unordered_map<std::string, ompl::base::State*> task_space_goal_states;
  task_space_goal_states[grand_child1->getName()] = task_goal1;
  task_space_goal_states[grand_child2->getName()] = task_goal2;

  auto maybe_goal = ComputeValidTotalState(factor, task_space_goal_states);
  EXPECT_TRUE(maybe_goal.has_value());
  auto goal = maybe_goal.value();
  std::cout << "Found goal IK solution" << std::endl;
  factor->printState(goal);

  auto tcp_goal = multi_robot->Robot::GetFK(goal);
  EXPECT_EQ(tcp_goal.size(), 2u);
  std::cout << "Tcp goal[1] " << tcp_goal.at(0) << std::endl;
  std::cout << "Tcp goal[2] " << tcp_goal.at(1) << std::endl;
  EXPECT_NEAR(Distance(tcp_goal.at(0), task_goal1_eigen.configuration), 0.0, kIKSolutionAccuracy);
  EXPECT_NEAR(Distance(tcp_goal.at(1), task_goal2_eigen.configuration), 0.0, kIKSolutionAccuracy);

  ////////////////////////////////////////////////////////////////////////////////
  // Motion propagation
  ////////////////////////////////////////////////////////////////////////////////
  std::cout << std::string(80, '-') << std::endl;
  std::cout << "Starting to propagate motion..." << std::endl;
  auto configs = motion_validator->propagateMotion(start, goal);
  EXPECT_GT(configs.size(), 0);

  auto d_total = factor->distance(configs.front(), configs.back());

  EXPECT_GT(d_total, 0.1*factor->distance(start, goal));
  EXPECT_LT(d_total, factor->distance(start, goal));
  std::cout << "Propagation Progress is " << d_total << "/" << factor->distance(start, goal) << std::endl;

  for(const auto& config : configs) {
    EXPECT_TRUE(pairwise_collision_checker->isValid(config));
    EXPECT_TRUE(factor->isValid(config));
  }
  for(size_t k = 1; k < configs.size(); k++) {
    auto a = configs.at(k-1);
    auto b = configs.at(k);
    auto d = factor->distance(a, b);
    EXPECT_GT(d, 0.0);
    EXPECT_LT(d, 0.1);
  }
  auto config = multi_robot->StateToEigen(configs.back());
  std::cout << "Reached config " << config << std::endl;

  //Check for invalid entries in config
  for(size_t k = 0; k < config.size(); k++) {
    EXPECT_GT(std::abs(config[k]), 1e-10);
  }

  auto tcps = multi_robot->Robot::GetFK(configs.back());
  std::cout << "  Reached Tcp for " << config << std::endl;
  for(const auto& tcp : tcps) {
    std::cout << "    " << tcp << std::endl;
  }
}

TEST(MultiRobotMotionValidatorTest, CollisionTest) {
  const float kHeight = 0.5;
  {
    auto start1 = MakeState({1.0, 0.0, kHeight});
    auto goal1 = MakeState({0.1, 0.0, kHeight});
    auto start2 = MakeState({-1.0, 0.0, kHeight});
    auto goal2 = MakeState({-0.1, 0.0, kHeight});
    CheckMultiRobotEdge(start1, goal1, start2, goal2);
  }

  {
    auto start1 = MakeState({0.0, 1.0, kHeight});
    auto goal1 = MakeState({0.0, -1.0, kHeight});
    auto start2 = MakeState({-1.0, 0.0, kHeight});
    auto goal2 = MakeState({+1.0, 0.0, kHeight});
    CheckMultiRobotEdge(start1, goal1, start2, goal2);
  }

  {
    auto start1 = MakeState({-1.0, -1.0, kHeight});
    auto goal1 = MakeState({+1.0, +1.0, kHeight});
    auto start2 = MakeState({-1.0, +1.0, kHeight});
    auto goal2 = MakeState({+1.0, -1.0, kHeight});
    CheckMultiRobotEdge(start1, goal1, start2, goal2);
  }
}

class DiskRobot : public SphereRobot {
  public:
    DiskRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton(const YAML::Node& /*node*/) override {
      return createSphere(0.2);
    }
};

class DiskRobotTest : public testing::Test {
protected:
  void SetUp() override {
    dart::simulation::WorldPtr world(new dart::simulation::World);

    robot1 = MakeRobot<DiskRobot>(world);
    const auto limits1 = std::make_pair(State3d(-1.0, 0.0, 0.0), State3d(1.0, 0.0, 0.0));
    robot1->SetLimits(limits1);
    auto space1 = robot1->GetSpaceInformation();

    robot2 = MakeRobot<DiskRobot>(world);
    const auto limits2 = std::make_pair(State3d(0.0, -1.0, 0.0), State3d(0.0, 1.0, 0.0));
    robot2->SetLimits(limits2);
    auto space2 = robot2->GetSpaceInformation();

    ////////////////////////////////////////////////////////////////////////////////
    // Setup multi robot space
    ////////////////////////////////////////////////////////////////////////////////
    std::vector<RobotPtr> robots = {robot1, robot2};
    multi_robot = MultiRobot::MakeMultiRobot(robots);
    factor = multi_robot->GetSpaceInformation();
    auto projection_to_1 = std::make_shared<ompl::multilevel::SubspaceProjection>(factor, space1, 0);
    auto projection_to_2 = std::make_shared<ompl::multilevel::SubspaceProjection>(factor, space2, 1);
    bool computer_fiber_space = false;
    EXPECT_TRUE(factor->addChild(space1, projection_to_1, computer_fiber_space));
    EXPECT_TRUE(factor->addChild(space2, projection_to_2, computer_fiber_space));

    collision_checker = std::make_shared<MultiRobotCollisionChecker>(world, multi_robot);
    factor->setStateValidityChecker(collision_checker);

    name1 = robot1->GetSpaceInformation()->getName();
    name2 = robot2->GetSpaceInformation()->getName();
  }

  EigenPathPtr CreateDiskEdgePath(const std::vector<StateXd>& configs) {
    auto children_states = factor->allocChildStates();
    std::vector<const ompl::base::State*> states;
    for(const auto& config : configs) {
      auto state = factor->allocState();
      multi_robot->EigenToState(config, state);
      states.push_back(state);
    }
    auto ompl_path = std::make_shared<ompl::geometric::PathGeometric>(factor, states);
    auto path = std::make_shared<EigenPath>(multi_robot, ompl_path);
    return path;
  }

  void CheckDiskEdgePath(const EigenPathPtr& path, bool desired_result) {
    const float kStepSize = 0.01;

    auto children_states = factor->allocChildStates();
    auto state = factor->allocState();

    for(float position = 0.0; position < 1.0; position += kStepSize) {
      auto config = path->GetConfigAt(position);
      //std::cout << "[Step " << position << "] Config " << config << std::endl;
      multi_robot->EigenToState(config, state);
      factor->project(state, children_states);

      auto config1 = robot1->StateToEigen(children_states.at(name1));
      robot1->GetSkeleton()->setConfiguration(config1.configuration);
      auto config2 = robot2->StateToEigen(children_states.at(name2));
      robot2->GetSkeleton()->setConfiguration(config2.configuration);

      EXPECT_EQ(collision_checker->GetCollisionChecker()->IsInCollision(), desired_result);
    }

    factor->freeState(state);
    factor->freeChildStates(children_states);
  }

  ompl::multilevel::FactoredSpaceInformationPtr factor;
  std::shared_ptr<DiskRobot> robot1;
  std::shared_ptr<DiskRobot> robot2;
  std::string name1;
  std::string name2;
  MultiRobotPtr multi_robot;
  std::shared_ptr<MultiRobotCollisionChecker> collision_checker;
};

TEST_F(DiskRobotTest, PathSpacingTest) {
  CheckDiskEdgePath(
      CreateDiskEdgePath({
        MakeState({-1,0,0,0,-1,0}), 
        MakeState({+1,0,0,0,-1,0}), 
        MakeState({+1,0,0,0,+1,0})
      }),
      false
  );
  CheckDiskEdgePath(
      CreateDiskEdgePath({
        MakeState({-1,0,0,0,-1,0}), 
        MakeState({+1,0,0,0,-1,0}), 
        MakeState({+1,0,0,0,-1,0}), 
        MakeState({+1,0,0,0,+1,0})
      }),
      false
  );
  CheckDiskEdgePath(
      CreateDiskEdgePath({
        MakeState({-1,0,0,0,-1,0}), 
        MakeState({+0,0,0,0,-1,0}), 
        MakeState({+1,0,0,0,-1,0}), 
        MakeState({+1,0,0,0,+1,0})
      }),
      false
  );
}
