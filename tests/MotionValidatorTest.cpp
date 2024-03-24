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
#include "TaskSpaceProjection.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"

const size_t kMaxConnections = 20;
const float kLineAccuracy = 0.1;
const float kIKSolutionAccuracy = 1e-5;

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
    const auto s1_tcp = robot->GetFK(random_s1).front();
    const auto s2_tcp = robot->GetFK(random_s2).front();

    ////////////////////////////////////////////////////////////////////////////////
    //Set obstacle to mid point along task space straight line
    ////////////////////////////////////////////////////////////////////////////////
    const auto mid = s1_tcp + 0.5 * (s2_tcp - s1_tcp);
    auto mid_config = MakeState({mid[0], mid[1], mid[2]});
    obstacle->GetSkeleton()->setConfiguration(mid_config.configuration);

    auto configs = motion_validator->propagateMotion(s1, s2);
    if(configs.empty()) {
      std::cout << "Could not make progress on " << random_s1 << std::endl;
      continue;
    }
    EXPECT_GT(configs.size(), 0);
    auto d_total = factor->distance(configs.front(), configs.back());
    std::cout << "Propagation Progress is " << d_total << "/" << factor->distance(s1, s2) << std::endl;
    EXPECT_LT(d_total, 0.5 * factor->distance(s1, s2));
    EXPECT_GE(d_total, 0.0);

    auto config = robot->StateToEigen(configs.back());
    std::cout << "Reached config " << config << std::endl;
    const auto s3_tcp = robot->GetFK(config).front();

    std::cout << "  Tcp[start]   " << s1_tcp << std::endl;
    std::cout << "  Tcp[goal]    " << s2_tcp << std::endl;
    std::cout << "  Tcp[reached] " << s3_tcp << std::endl;
    std::cout << "  Tcp[mid]     " << mid << std::endl;
    std::cout << std::string(80,'-') << std::endl;
    auto d_mid = LineDistance(s1_tcp, s2_tcp, mid_config.configuration);
    auto d = LineDistance(s1_tcp, s2_tcp, s3_tcp);
    EXPECT_LT(d_mid, kLineAccuracy);
    EXPECT_LT(d, kLineAccuracy);
  }

  factor->freeState(s2);
  factor->freeState(s1);
}
