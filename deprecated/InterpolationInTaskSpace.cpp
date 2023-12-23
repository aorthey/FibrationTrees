#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "TaskSpace.hpp"
#include "TaskSpaceGoal.hpp"
#include "TaskSpaceProjection.hpp"
#include "TaskSpaceMotionValidator.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "gui/Visualizer.hpp"
#include "robots/KukaSkeleton.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>

const float kTableHeight = 0.5;

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr manipulator = createKukaSkeleton();

  PrintSkeletonInfo(manipulator);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr floor = createFloor();

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator);
  world->addSkeleton(floor);
  addCoordinateFrameToWorld(world);
  world->setGravity(Eigen::Vector3d::Zero());

  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  ////////////////////////////////////////////////////////////////////////////////
  ////Collision checking
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> g1 = {manipulator};
  std::vector<dart::dynamics::SkeletonPtr> g2 = {floor};
  CollisionCheckerPtr collision_checker = std::make_shared<CollisionChecker>(world, g1, g2);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto numDofs = manipulator->getNumDofs();
  ompl::base::StateSpacePtr space(new TaskSpace(numDofs, kinematics_solver));
  ompl::base::RealVectorBounds bounds(numDofs);
  auto lb = manipulator->getPositionLowerLimits();
  auto ub = manipulator->getPositionUpperLimits();
  for(size_t k =0; k< numDofs; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));
  auto validity_checker = std::make_shared<DartWorldCollisionChecker>(factor, world, manipulator, collision_checker);
  factor->setStateValidityChecker(validity_checker);
  factor->setStateValidityCheckingResolution(0.001);
  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<TaskSpaceMotionValidator>(factor, kinematics_solver);
  factor->setMotionValidator(motion_validator);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create interpolated path
  ////////////////////////////////////////////////////////////////////////////////

  dart::math::Random::setSeed(0);
  Eigen::Vector3d start_frame = {0.6, 0.1, kTableHeight};
  Eigen::Vector3d goal_frame = {0.7, +0.4, kTableHeight};
  ompl::base::State* state = factor->allocState();

  Eigen::VectorXd start_config;

  const size_t max_repeats = 100;
  size_t counter = 0;
  while(true) {
    if(counter++ > max_repeats) {
      std::cout << "Could not find IK solution for " << start_frame << " after " << counter << " iterations." << std::endl;
      return 1;
    }
    auto maybe_start_config = kinematics_solver->solve_ik(start_frame, 100);
    if(!maybe_start_config.has_value()) {
      continue;
    }
    start_config = maybe_start_config.value();
    EigenVectorXdToState(start_config, state);
    if(!validity_checker->isValid(state)) {
      continue;
    }
    std::cout << start_config <<std::endl;
    break;
  }

  auto configs = kinematics_solver->solve_edge_ik(start_config, goal_frame);
  if(!kinematics_solver->lastSolveWasSuccessful()) {
    std::cout << "Found only " << configs.size() << std::endl;
  }
  world->addSimpleFrame(createSphereFrame(start_frame));
  world->addSimpleFrame(createSphereFrame(goal_frame));

  auto path = PathFromEigenVectors(configs, factor);

  ompl::geometric::PathGeometric &p1 = *static_cast<ompl::geometric::PathGeometric *>(path.get());

  ////////////////////////////////////////////////////////////////////////////////
  //Create second interpolated path
  ////////////////////////////////////////////////////////////////////////////////
  Eigen::Vector3d mid_frame = {0.7, -0.1, kTableHeight};
  configs = kinematics_solver->solve_edge_ik(configs.back(), mid_frame);
  world->addSimpleFrame(createSphereFrame(mid_frame));

  auto path2 = PathFromEigenVectors(configs, factor);

  ompl::geometric::PathGeometric &p2 = *static_cast<ompl::geometric::PathGeometric *>(path2.get());
  p1.append(p2);

  ////////////////////////////////////////////////////////////////////////////////
  //Create second interpolated path
  ////////////////////////////////////////////////////////////////////////////////
  Eigen::Vector3d end_frame = {+0.6, -0.2, kTableHeight};
  configs = kinematics_solver->solve_edge_ik(configs.back(), end_frame);
  world->addSimpleFrame(createSphereFrame(end_frame));

  auto path3 = PathFromEigenVectors(configs, factor);

  ompl::geometric::PathGeometric &p3 = *static_cast<ompl::geometric::PathGeometric *>(path3.get());
  p1.append(p3);
  p1.interpolate(700);
  p1.interpolate(800);
  p1.interpolate(900);
  p1.interpolate(1000);

  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  visualizer.AddPath(manipulator, path);
  visualizer.SetCollisionChecker(collision_checker);
  visualizer.Run();

  return 0;
}
