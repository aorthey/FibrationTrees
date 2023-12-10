#include "TaskSpace.hpp"
#include "TaskSpaceGoal.hpp"
#include "TaskSpaceProjection.hpp"
#include "TaskSpaceMotionValidator.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "MakeSpaceInformation.hpp"
#include "gui/Visualizer.hpp"
#include "robots/KukaSkeleton.hpp"

#include <dart/dart.hpp>

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

  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  obstacles.push_back(createFromURDF("/home/aorthey/git/FibrationTrees/data/objects/maze.urdf", Eigen::Vector3d(+0.55, +0.1, 0.85)));
  obstacles.push_back(createFloor());
  obstacles.push_back(createBox(Eigen::Vector3d(+0.5, +0.0, 0.75), 0.16, 2.0, 1.5));

  PrintSkeletonInfo(manipulator);
  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr point = createSphere(Eigen::Vector3d(0, 0, 0), 0.01);

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator);
  for(const auto& obstacle : obstacles) {
    world->addSkeleton(obstacle);
  }
  world->addSkeleton(point);
  world->setGravity(Eigen::Vector3d::Zero());

  KinematicsSolverPtr kinematics_solver = std::make_shared<KinematicsSolver>(manipulator);

  ////////////////////////////////////////////////////////////////////////////////
  ////Collision checking
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> g1 = {manipulator};
  CollisionCheckerPtr collision_checker = std::make_shared<CollisionChecker>(world, g1, obstacles);
  std::vector<dart::dynamics::SkeletonPtr> g3 = {point};
  CollisionCheckerPtr collision_checker_point_robot = std::make_shared<CollisionChecker>(world, g3, obstacles);

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

  EigenVector3dToState({0.4, 0.35, 0.95}, task_start);
  EigenVector3dToState({0.4, -0.25, 0.95}, task_goal);

  ompl::base::State *start = factor->allocState();
  ompl::base::State *goal = factor->allocState();

  const int kMaxResampleIteration = 100;
  bool has_solution = true;

  Visualizer visualizer(world);

  if(!SampleValidLift(projection, factor, kMaxResampleIteration, task_start, start)) {
    OMPL_ERROR("Could not find valid start state after %d samples.", kMaxResampleIteration);
    child->printState(task_start);
    has_solution = false;
  }
  if(has_solution && !SampleValidLift(projection, factor, kMaxResampleIteration, task_goal, goal)) {
    OMPL_ERROR("Could not find valid goal state after %d samples.", kMaxResampleIteration);
    child->printState(task_goal);
    has_solution = false;
  }

  if(has_solution) {
    auto goal_region = std::make_shared<TaskSpaceGoal>(factor, goal, projection);
    goal_region->setThreshold(0.05);

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

    float timeout = 100.0;
    ompl::base::PlannerStatus status = planner->Planner::solve(timeout);
    visualizer.AddPlanner(manipulator, planner);
    visualizer.AddPath(point, planner->getProblemDefinition(child->getName())->getSolutionPath(), Eigen::Vector3d(1, 1, 0));
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  visualizer.Run();
  return 0;
}
