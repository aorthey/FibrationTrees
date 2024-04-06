#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "projections/ProjectionTaskSpace.hpp"
#include "gui/Visualizer.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MobileKukaRobotTaskSpace.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"

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
  ////Creating obstacles
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  obstacles.push_back(createFromURDF("/home/aorthey/git/FibrationTrees/data/objects/maze.urdf", State3d(+0.55, +0.1, 0.85)));
  obstacles.push_back(createFloor());
  obstacles.push_back(createBox(State3d(+0.5, +0.0, 0.75), 0.15, 2.0, 1.5));

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle : obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(State3d::Zero());
  addCoordinateFrameToWorld(world);

  auto robot = MakeRobot<KukaRobotTaskSpace>(world, obstacles);
  auto point = MakeRobot<SphereRobot>(world, obstacles);

  const auto limits = std::make_pair(State3d(0.39, -0.4, 0), State3d(0.43, +0.4, 2));
  point->SetLimits(limits);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor = robot->GetSpaceInformation();
  auto child = point->GetSpaceInformation();

  ompl::multilevel::ProjectionPtr projection = std::make_shared<ProjectionTaskSpace>(factor, child, robot);//, kinematics_solver);
  factor->addChild(child, projection);

  factor->printFactorization(std::cout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////
  ompl::base::State *task_start = child->allocState();
  ompl::base::State *task_goal = child->allocState();

  point->EigenToState(MakeState({0.4, +0.35, 0.95}), task_start);
  point->EigenToState(MakeState({0.4, -0.25, 0.95}), task_goal);

  ompl::base::State *start = factor->allocState();
  ompl::base::State *goal = factor->allocState();

  const int kMaxResampleIteration = 100;
  bool has_solution = true;

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
    goal_region->setThreshold(0.01);

    ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
    pdef->addStartState(start);
    pdef->setGoal(goal_region);

    auto start_vector = point->StateToEigen(task_start);
    auto goal_vector = point->StateToEigen(task_goal);
    world->addSimpleFrame(createSphereFrame(start_vector.configuration, 0.01));
    world->addSimpleFrame(createSphereFrame(goal_vector.configuration, 0.01));

    ////////////////////////////////////////////////////////////////////////////////
    ////Planning
    ////////////////////////////////////////////////////////////////////////////////
    auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
    planner->setProblemDefinition(pdef);
    planner->setup();
    planner->setRange(+Inf);

    float timeout = 100.0;

    auto ptc = ompl::base::plannerOrTerminationCondition(
            ompl::base::exactSolnPlannerTerminationCondition(pdef),
            ompl::base::timedPlannerTerminationCondition(timeout)
        );

    ompl::base::PlannerStatus status = planner->solve(ptc);

    Visualizer visualizer(world);
    visualizer.AddPlanner(robot, planner);
    visualizer.SetCollisionChecker(robot->GetCollisionChecker());
    visualizer.AddPath(point, planner->getProblemDefinition(child->getName())->getSolutionPath(), State3d(1, 1, 0));
    visualizer.Run();
  }
  return 0;
}
