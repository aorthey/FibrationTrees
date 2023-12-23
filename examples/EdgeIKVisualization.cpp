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

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr manipulator_visual = createKukaSkeleton();
  changeBodyColor(manipulator_visual, Eigen::Vector4d(0.8, 0.8, 0, 0.7));
  dart::dynamics::SkeletonPtr manipulator = createKukaSkeleton();

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr floor = createFloor();

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator_visual);
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
  //Create interpolated path
  ////////////////////////////////////////////////////////////////////////////////
  Eigen::VectorXd start(7);
  Eigen::VectorXd goal(7);
  start << -2.81856, 1.16738, 0.78956, -1.93341, 2.967, 1.64126, -1.88672;
  goal << 0.971826, -1.00951, 1.60224, -0.688863, 2.59801, 0.441576, 0.695603;

  start << -1.23524, -0.110832, -1.21279, -1.55001, 0.836169, 0.035207, 0.0108537;
  goal << 1.57652, 1.35352, 2.07744, -1.80144, -2.96705, -0.338235, -2.45317;

  // Cannot reach goal
  start << -0.503705, -1.08742, -1.14191, -1.98434, 0.363036, -1.08686, -0.0262207;
  goal << -1.39498, -1.55469, 2.28343, 1.95646, -1.09705, -0.305218, -1.31804;

  manipulator_visual->setConfiguration(goal);

  auto configs = kinematics_solver->solve_edge_ik_with_config(start, goal);
  auto path = PathFromEigenVectors(configs, factor);

  std::cout << "Start  (Actual) : " << configs.front().format(CommaFmt) << std::endl;
  std::cout << "Goal  (Desired) : " << goal.format(CommaFmt) << std::endl;
  std::cout << "Goal   (Actual) : " << configs.back().format(CommaFmt) << std::endl;

  ////////////////////////////////////////////////////////////////////////////////
  //// Visualize end points 
  ////////////////////////////////////////////////////////////////////////////////
  const auto maybe_tcp_start = kinematics_solver->solve_fk(start);
  if(!maybe_tcp_start.has_value()) {
    OMPL_ERROR("No start tcp");
    return 1;
  }
  auto start_frame = maybe_tcp_start.value();
  world->addSimpleFrame(createSphereFrame(start_frame));

  const auto maybe_tcp_goal = kinematics_solver->solve_fk(goal);
  if(!maybe_tcp_goal.has_value()) {
    OMPL_ERROR("No goal tcp");
    return 1;
  }
  auto goal_frame = maybe_tcp_goal.value();
  world->addSimpleFrame(createSphereFrame(goal_frame));

  world->addSimpleFrame(createLineSegmentFrame(start_frame, goal_frame, color_green));
  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  visualizer.AddPath(manipulator, path);
  visualizer.SetCollisionChecker(collision_checker);
  visualizer.Run();

  return 0;
}
