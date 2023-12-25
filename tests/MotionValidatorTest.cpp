#include <gtest/gtest.h>
#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include <ompl/base/State.h>

#include "CollisionChecker.hpp"
#include "Common.hpp"
#include "DartHelper.hpp"
#include "KinematicsSolver.hpp"
#include "MakeSpaceInformation.hpp"
#include "robots/KukaSkeleton.hpp"

const size_t kMaxConnections = 20;

TEST(MotionValidatorTest, RandomConfigConnectionTest) {
  dart::dynamics::SkeletonPtr manipulator = 
    createKukaSkeleton("/home/aorthey/git/FibrationTrees/data/robots/kuka_lwr/kuka.urdf");
  dart::dynamics::SkeletonPtr point = createSphere(0.01);

  dart::math::Random::setSeed(0);

  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator);
  world->addSkeleton(point);

  std::vector<dart::dynamics::SkeletonPtr> robot_vector = {manipulator};
  std::vector<dart::dynamics::SkeletonPtr> obstacle_vector = {point};
  auto collision_checker = std::make_shared<CollisionChecker>(world, robot_vector, obstacle_vector);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////

  auto factor = MakeTaskSpaceInformation(manipulator, world, kinematics_solver, collision_checker);
  auto motion_validator = factor->getMotionValidator();
  
  std::pair<ompl::base::State *, double> lastValid;
  lastValid.first = factor->allocState();

  auto s1 = factor->allocState();
  auto s2 = factor->allocState();

  for(size_t k = 0; k < kMaxConnections; k++) {
    std::cout << std::string(80,'-') << std::endl;
    auto random_s1 = GetRandomPosition(manipulator);
    auto random_s2 = GetRandomPosition(manipulator);
    EigenVectorXdToState(random_s1, s1);
    EigenVectorXdToState(random_s2, s2);
    factor->printState(s1);
    factor->printState(s2);
    const auto s1_tcp = kinematics_solver->solve_fk(random_s1).value();
    const auto s2_tcp = kinematics_solver->solve_fk(random_s2).value();

    ////////////////////////////////////////////////////////////////////////////////
    //Set obstacle to mid point along task space straight line
    ////////////////////////////////////////////////////////////////////////////////
    const auto mid = s1_tcp + 0.5 * (s2_tcp - s1_tcp);
    Eigen::VectorXd mid_config(3);
    mid_config << mid[0], mid[1], mid[2];
    point->setConfiguration(mid_config);

    auto success = motion_validator->checkMotion(s1, s2, lastValid);
    std::cout << "Check Motion Progress is " << lastValid.second << "/" << factor->distance(s1, s2) << std::endl;
    EXPECT_LT(lastValid.second, 0.5 * factor->distance(s1, s2));
    EXPECT_GE(lastValid.second, 0.0);

    auto config = StateToEigenVectorXd(factor, lastValid.first);
    std::cout << "Reached config " << config.format(CommaFmt) << std::endl;
    std::cout << std::string(80,'-') << std::endl;

    // EXPECT_TRUE(motion_validator->checkMotion(s1, lastValid.first));
  }

  factor->freeState(s2);
  factor->freeState(s1);
  factor->freeState(lastValid.first);
}
