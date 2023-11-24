#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "TaskSpaceProjection.hpp"
#include "TaskStateSpace.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "DartEventHandler.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "robots/KukaSkeleton.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>

const Eigen::Vector3d kPathColorProjected = Eigen::Vector3d(0.8, 0.2, 0.8);

bool AddFKPathToWorld(const ompl::base::PathPtr& path, const KinematicsSolverPtr& kinematics_solver, const dart::simulation::WorldPtr& world) {
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  //pgeo.interpolate(20);
  OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());
  auto states = pgeo.getStates();
  int N = path->getSpaceInformation()->getStateDimension();
  for(size_t k =1; k < states.size(); k++) {
    auto s1 = states.at(k-1);
    auto s2 = states.at(k);
    auto v_s1 = StateToEigenVectorXd(N, s1);
    auto v_s2 = StateToEigenVectorXd(N, s2);
    auto maybe_v1 = kinematics_solver->solve_fk(v_s1);
    if(!maybe_v1.has_value()) {
      std::cout << "Could not solve FK." <<std::endl;
      return false;
    }
    auto maybe_v2 = kinematics_solver->solve_fk(v_s2);
    if(!maybe_v2.has_value()) {
      std::cout << "Could not solve FK." <<std::endl;
      return false;
    }
    auto v1 = maybe_v1.value();
    auto v2 = maybe_v2.value();
    world->addSimpleFrame(createLineSegmentFrame(v1, v2, kPathColorProjected));
    std::cout << "Edge " << v1 << " to " << v2 << "(length: " << (v2-v1).norm() << std::endl;
    auto length = (v2-v1).norm();
    if(length < 0.003) {
      world->addSimpleFrame(createSphereFrame(v2, 0.005, kPathColorProjected));
    }

  }
  return true;
}

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
  ompl::base::StateSpacePtr space(new TaskStateSpace(numDofs, kinematics_solver));
  ompl::base::RealVectorBounds bounds(numDofs);
  auto lb = manipulator->getPositionLowerLimits();
  auto ub = manipulator->getPositionUpperLimits();
  for(size_t k =0; k< numDofs; k++) {
    bounds.setLow(k, lb[k]);
    bounds.setHigh(k, ub[k]);
  }
  space->as<ompl::base::RealVectorStateSpace>()->setBounds(bounds);

  auto factor(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space));
  factor->setStateValidityChecker(std::make_shared<DartWorldCollisionChecker>(factor, world, manipulator, collision_checker));
  factor->setStateValidityCheckingResolution(0.001);
  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<TaskSpaceMotionValidator>(factor, kinematics_solver);
  factor->setMotionValidator(motion_validator);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create interpolated path
  ////////////////////////////////////////////////////////////////////////////////

  dart::math::Random::setSeed(0);
  Eigen::Vector3d start_frame = {0.6, 0.1, 0.3};
  Eigen::Vector3d goal_frame = {0.4, 0.5, 0.7};
  auto maybe_start_config = kinematics_solver->solve_ik(start_frame, 100);
  if(!maybe_start_config.has_value()) {
    std::cout << "Could not solve IK." <<std::endl;
    return 1;
  }
  auto start_config = maybe_start_config.value();
  std::cout << start_config <<std::endl;

  auto configs = kinematics_solver->solve_edge_ik(start_config, goal_frame);
  if(!kinematics_solver->lastSolveWasSuccessful()) {
    std::cout << "Found only " << configs.size() << std::endl;
    return 1;
  }
  world->addSimpleFrame(createSphereFrame(start_frame));
  world->addSimpleFrame(createSphereFrame(goal_frame));

  auto path = PathFromEigenVectors(configs, factor);
  AddFKPathToWorld(path, kinematics_solver, world);

  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  dart::gui::osg::Viewer viewer;

  osg::ref_ptr<PathReplayWorldNode> node = new PathReplayWorldNode(world, manipulator, path, collision_checker);
  viewer.addInstructionText("Press [s] to play planned path.\n");
  viewer.addEventHandler(new PathReplayEventHandler(node.get()));
  viewer.addWorldNode(node);

  viewer.setUpViewInWindow(0, 0, 640, 480);

  const auto& eye = ::osg::Vec3(3, 0, 2);
  const auto& center = ::osg::Vec3(0, 0, 0.5);
  const auto& up = ::osg::Vec3(0, 0, 1);

  viewer.getCameraManipulator()->setHomePosition(eye, center, up);
  viewer.setCameraManipulator(viewer.getCameraManipulator()); //update 

  viewer.simulate(true);
  viewer.run();

  return 0;
}
