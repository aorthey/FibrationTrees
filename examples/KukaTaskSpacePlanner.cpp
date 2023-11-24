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

void AddOmplPathToWorld(const ompl::base::PathPtr& path, const ompl::multilevel::ProjectionPtr& projection, const dart::simulation::WorldPtr& world) {
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());
  auto states = pgeo.getStates();
  for(size_t k =1; k < states.size(); k++) {
    auto s1 = states.at(k-1);
    auto s2 = states.at(k);
    auto v1 = ProjectStateToEigenVector3d(projection, s1);
    auto v2 = ProjectStateToEigenVector3d(projection, s2);
    world->addSimpleFrame(createLineSegmentFrame(v1, v2));
  }
}

void AddR3PathToWorld(const ompl::base::PathPtr& path, const dart::simulation::WorldPtr& world) {
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  pgeo.interpolate(100);
  OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());
  auto states = pgeo.getStates();
  for(size_t k =1; k < states.size(); k++) {
    auto s1 = states.at(k-1);
    auto s2 = states.at(k);
    auto v1 = StateToEigenVector3d(s1);
    auto v2 = StateToEigenVector3d(s2);
    world->addSimpleFrame(createLineSegmentFrame(v1, v2, kPathColorProjected));
  }
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
  // dart::dynamics::SkeletonPtr c1 = createCylinder(Eigen::Vector3d(+0.5, +0.5, 0), 0.25, 1.5);
  // dart::dynamics::SkeletonPtr c2 = createCylinder(Eigen::Vector3d(-0.5, -0.5, 0), 0.25, 1.5);
  dart::dynamics::SkeletonPtr c1 = createCylinder(Eigen::Vector3d(+0.5, +0.5, 0), 0.1, 1.2);
  dart::dynamics::SkeletonPtr c2 = createCylinder(Eigen::Vector3d(-0.5, -0.5, 0), 0.1, 1.2);
  dart::dynamics::SkeletonPtr point = createSphere(Eigen::Vector3d(-0.5, -0.5, 0), 0.01);

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator);
  world->addSkeleton(floor);
  world->addSkeleton(c1);
  world->addSkeleton(c2);
  world->addSkeleton(point);
  world->setGravity(Eigen::Vector3d::Zero());

  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  ////////////////////////////////////////////////////////////////////////////////
  ////Collision checking
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> g1 = {manipulator};
  std::vector<dart::dynamics::SkeletonPtr> g2 = {floor, c1, c2};
  CollisionCheckerPtr collision_checker = std::make_shared<CollisionChecker>(world, g1, g2);
  std::vector<dart::dynamics::SkeletonPtr> g3 = {point};
  CollisionCheckerPtr collision_checker_point_robot = std::make_shared<CollisionChecker>(world, g3, g2);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto numDofs = manipulator->getNumDofs();
  std::cout << "OMPL version: " << OMPL_VERSION << std::endl;
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
  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<TaskSpaceMotionValidator>(factor,kinematics_solver);
  factor->setMotionValidator(motion_validator);

  auto numDofsPoint = 3;
  ompl::base::StateSpacePtr spaceR3(new ompl::base::RealVectorStateSpace(numDofsPoint));
  ompl::base::RealVectorBounds boundsWorkspace(numDofsPoint);
  boundsWorkspace.setLow(0, -2);
  boundsWorkspace.setHigh(0, +2);
  boundsWorkspace.setLow(1, -2);
  boundsWorkspace.setHigh(1, +2);
  boundsWorkspace.setLow(2, 0.25);
  boundsWorkspace.setHigh(2, 0.35);
  spaceR3->as<ompl::base::RealVectorStateSpace>()->setBounds(boundsWorkspace);

  auto child(std::make_shared<ompl::multilevel::FactoredSpaceInformation>(spaceR3));
  child->setStateValidityChecker(std::make_shared<DartTransformCollisionChecker>(factor, world, point, collision_checker_point_robot));
  child->setStateValidityCheckingResolution(0.001);

  ompl::multilevel::ProjectionPtr projection = std::make_shared<ProjectionJointSpaceToR3>(space, spaceR3, kinematics_solver);
  factor->addChild(child, projection);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::State *task_start = child->allocState();
  ompl::base::State *task_goal = child->allocState();
  double *start_angles = task_start->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double *goal_angles = task_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  start_angles[0] = +0.6;
  start_angles[1] = +0.1;
  start_angles[2] = 0.27;
  goal_angles[0] = +0.1;
  goal_angles[1] = +0.6;
  goal_angles[2] = 0.33;

  ompl::base::State *start = factor->allocState();
  ompl::base::State *goal = factor->allocState();

  const int kMaxResampleIteration = 1000;
  if(!SampleValidLift(projection, factor, kMaxResampleIteration, task_start, start)) {
    OMPL_ERROR("Could not find valid start state after %d samples.", kMaxResampleIteration);
    return 1;
  }
  if(!SampleValidLift(projection, factor, kMaxResampleIteration, task_goal, goal)) {
    OMPL_ERROR("Could not find valid goal state after %d samples.", kMaxResampleIteration);
    return 1;
  }

  factor->printState(start);
  factor->printState(goal);

  child->freeState(task_start);
  child->freeState(task_goal);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoalState(goal, 0.1);

  auto start_vector = ProjectStateToEigenVector3d(projection, start);
  auto goal_vector = ProjectStateToEigenVector3d(projection, goal);
  world->addSimpleFrame(createSphereFrame(start_vector));
  world->addSimpleFrame(createSphereFrame(goal_vector));

  factor->freeState(start);
  factor->freeState(goal);

  ////////////////////////////////////////////////////////////////////////////////
  ////Planning
  ////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(Inf);

  float timeout = 5.0;
  // float timeout = 0.001;
  ompl::base::PlannerStatus status = planner->Planner::solve(timeout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////

  osg::ref_ptr<dart::gui::osg::ImGuiViewer> viewer
    = new dart::gui::osg::ImGuiViewer();

  if (!(status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
      status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION ||
      pdef->hasApproximateSolution()))
  {
      OMPL_ERROR("Could not find a solution.");
      return 1;
  }

  auto pdefs = planner->getProblemDefinitions();

  auto path_R3 = pdefs.begin()->second->getSolutionPath();
  AddR3PathToWorld(path_R3, world);

  ompl::base::PathPtr path = pdef->getSolutionPath();
  osg::ref_ptr<PathReplayWorldNode> node = new PathReplayWorldNode(world, manipulator, path, collision_checker);
  viewer->addInstructionText("s: play planned path.\n");
  viewer->addInstructionText("r: reverse execution direction.\n");

  viewer->addEventHandler(new PathReplayEventHandler(node.get()));
  viewer->addWorldNode(node);
  AddOmplPathToWorld(path, projection, world);
  viewer->getImGuiHandler()->addWidget(
      std::make_shared<TextWidget>(viewer, node.get()));

  viewer->setUpViewInWindow(0, 0, 640, 480);

  const auto& eye = ::osg::Vec3(3, 0, 2);
  const auto& center = ::osg::Vec3(0, 0, 0.5);
  const auto& up = ::osg::Vec3(0, 0, 1);

  viewer->getCameraManipulator()->setHomePosition(eye, center, up);
  viewer->setCameraManipulator(viewer->getCameraManipulator()); //update 

  viewer->simulate(true);
  viewer->run();

  return 0;
}
