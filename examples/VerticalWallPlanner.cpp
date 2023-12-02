#include <dart/dart.hpp>

#include "TaskSpaceProjection.hpp"
#include "TaskSpace.hpp"
#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "MakeSpaceInformation.hpp"
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
  dart::dynamics::SkeletonPtr manipulator = createKukaSkeleton();

  PrintSkeletonInfo(manipulator);
  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr floor = createFloor();
  dart::dynamics::SkeletonPtr wall = createBox(Eigen::Vector3d(+0.5, +0.0, 0.75), 0.16, 2.0, 1.5);
  dart::dynamics::SkeletonPtr point = createSphere(Eigen::Vector3d(-0.5, -0.5, -0.5), 0.01);

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator);
  world->addSkeleton(floor);
  world->addSkeleton(wall);
  world->addSkeleton(point);
  world->setGravity(Eigen::Vector3d::Zero());

  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  ////////////////////////////////////////////////////////////////////////////////
  ////Collision checking
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> g1 = {manipulator};
  std::vector<dart::dynamics::SkeletonPtr> g2 = {floor, wall};
  CollisionCheckerPtr collision_checker = std::make_shared<CollisionChecker>(world, g1, g2);
  std::vector<dart::dynamics::SkeletonPtr> g3 = {point};
  CollisionCheckerPtr collision_checker_point_robot = std::make_shared<CollisionChecker>(world, g3, g2);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////

  auto factor = MakeTaskSpaceInformation(manipulator, world, kinematics_solver, collision_checker);
  auto child = Make3DPointSpaceInformation(point, world, collision_checker_point_robot);

  ompl::multilevel::ProjectionPtr projection = std::make_shared<ProjectionJointSpaceToR3>(factor->getStateSpace(), child->getStateSpace(), kinematics_solver);
  factor->addChild(child, projection);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::State *task_start = child->allocState();
  ompl::base::State *task_goal = child->allocState();
  double *start_angles = task_start->as<ompl::base::RealVectorStateSpace::StateType>()->values;
  double *goal_angles = task_goal->as<ompl::base::RealVectorStateSpace::StateType>()->values;

  start_angles[0] = +0.4;
  start_angles[1] = +0.5;
  start_angles[2] = 0.5;
  goal_angles[0] = +0.4; //0.4, 0.5, 0.8
  goal_angles[1] = -0.3;
  goal_angles[2] = 0.9;

  ompl::base::State *start = factor->allocState();
  ompl::base::State *goal = factor->allocState();

  const int kMaxResampleIteration = 100;
  if(!SampleValidLift(projection, factor, kMaxResampleIteration, task_start, start)) {
    OMPL_ERROR("Could not find valid start state after %d samples.", kMaxResampleIteration);
    return 1;
  }
  std::cout << "Found start state." << std::endl;
  factor->printState(start);
  if(!SampleValidLift(projection, factor, kMaxResampleIteration, task_goal, goal)) {
    OMPL_ERROR("Could not find valid goal state after %d samples.", kMaxResampleIteration);
    return 1;
  }
  std::cout << "Found goal state." << std::endl;
  auto goal_region = std::make_shared<TaskSpaceGoal>(factor, goal, projection);
  goal_region->setThreshold(0.1);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoal(goal_region);

  auto start_vector = ProjectStateToEigenVector3d(projection, start);
  auto goal_vector = StateToEigenVector3d(task_goal);
  world->addSimpleFrame(createSphereFrame(start_vector, 0.02));
  world->addSimpleFrame(createSphereFrame(goal_vector, 0.02));

  ////////////////////////////////////////////////////////////////////////////////
  ////Planning
  ////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(Inf);

  float timeout = 10.0;
  ompl::base::PlannerStatus status = planner->Planner::solve(timeout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  visualizer.AddPlanner(manipulator, planner);
  visualizer.Run();
  return 0;
}
