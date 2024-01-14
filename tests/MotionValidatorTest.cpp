#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ompl/base/State.h>

#include "CollisionChecker.hpp"
#include "Common.hpp"
#include "DartHelper.hpp"
#include "KinematicsSolver.hpp"
#include "robots/KukaRobot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"

const size_t kMaxConnections = 20;

class TestSphereRobot : public SphereRobot {
  public:
    TestSphereRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override {
      return createSphere(0.1);
    }
};

TEST(MotionValidatorTest, RandomConfigConnectionTest) {
  dart::math::Random::setSeed(0);
  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto robot = MakeRobot<KukaRobot>(world);
  auto point = MakeRobot<TestSphereRobot>(world);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor = robot->GetSpaceInformation();
  auto motion_validator = factor->getMotionValidator();
  
  std::pair<ompl::base::State *, double> lastValid;
  lastValid.first = factor->allocState();

  auto s1 = factor->allocState();
  auto s2 = factor->allocState();

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
    point->GetSkeleton()->setConfiguration(mid_config);

    auto success = motion_validator->checkMotion(s1, s2, lastValid);
    std::cout << "Check Motion Progress is " << lastValid.second << "/" << factor->distance(s1, s2) << std::endl;
    EXPECT_LT(lastValid.second, 0.5 * factor->distance(s1, s2));
    EXPECT_GE(lastValid.second, 0.0);

    auto config = robot->StateToEigen(lastValid.first);
    std::cout << "Reached config " << config.format(CommaFmt) << std::endl;
    const auto s3_tcp = GetFK(robot->GetSkeleton(), config);

    std::cout << "  Tcp[start]   " << s1_tcp.format(CommaFmt) << std::endl;
    std::cout << "  Tcp[goal]    " << s2_tcp.format(CommaFmt) << std::endl;
    std::cout << "  Tcp[reached] " << s3_tcp.format(CommaFmt) << std::endl;
    std::cout << "  Tcp[mid]     " << mid.format(CommaFmt) << std::endl;
    std::cout << std::string(80,'-') << std::endl;
    auto d_mid = LineDistance(s1_tcp, s2_tcp, mid_config);
    auto d = LineDistance(s1_tcp, s2_tcp, s3_tcp);
    EXPECT_LT(d_mid, 1e-6);
    EXPECT_LT(d, 1e-6);
  }

  factor->freeState(s2);
  factor->freeState(s1);
  factor->freeState(lastValid.first);
}
