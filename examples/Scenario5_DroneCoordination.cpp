#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "robots/ZeppelinRobot.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/ZeppelinInnerSphereRobot.hpp"
#include "robots/RobotFactory.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RNSO2_RN.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/base/goals/FactoredGoal.h>

#include <ranges>

const auto Nrobots = 2;

ompl::base::ProblemDefinitionPtr CreateMultiDroneProblemDefinition(
  const ompl::multilevel::FactoredSpaceInformationPtr& factor, 
  const std::vector<RobotPtr>& robots,
  size_t Nrobots) 
{
  auto children = factor->getChildren();
  float y_step = 0.3;

  float x = 1.5;
  const float yoffset = 0.5 * (Nrobots - 1) * y_step;
  float y = -yoffset;
  float z = 0.1;
  float yaw = 0.57;

  auto startStates = factor->allocChildStates();
  auto goalStates = factor->allocChildStates();

  for(const auto& robot : robots) {
    const auto name = robot->GetSpaceInformation()->getName();

    if(y > 0) {
      z = 0.3;
      y = -yoffset;
    }
    auto start = MakeEigen({x, y + 0.5 *yoffset, z, yaw});
    auto start_state = AllocStateFromEigen(robot, start);
    startStates[name] = start_state;

    auto goal = MakeEigen({-1.5, y + 0.5 *yoffset, z, yaw});
    auto goal_state = AllocStateFromEigen(robot, goal);
    goalStates[name] = goal_state;

    y += y_step;
  }

  OMPL_INFORM("Lifted start and goal states:");
  auto start = factor->allocState();
  factor->lift(startStates, start);
  factor->printState(start);

  auto goal = factor->allocState();
  factor->lift(goalStates, goal);
  factor->printState(goal);

  auto goal_region = std::make_shared<ompl::base::GoalState>(factor);
  goal_region->setState(goal);
  goal_region->setThreshold(0.05);
 
  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoal(goal_region);
  return pdef;
}

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  dart::dynamics::SkeletonPtr floor = createFloor(-0.25);
  dart::dynamics::SkeletonPtr o1 = createCylinder(Eigen::Vector3d(-0.5, 0.0, 0.0), 0.2, 1.5);
  dart::dynamics::SkeletonPtr o2 = createCylinder(Eigen::Vector3d(-0.0, 0.2, 0.0), 0.15, 1.0);
  dart::dynamics::SkeletonPtr o3 = createCylinder(Eigen::Vector3d(-0.5, -0.3, 0.0), 0.3, 1.5);
  dart::dynamics::SkeletonPtr o4 = createCylinder(Eigen::Vector3d(-1.0, 0.8, 0.0), 0.05, 2.5);
  dart::dynamics::SkeletonPtr o5 = createCylinder(Eigen::Vector3d(-0.6, 1.2, 0.0), 0.08, 0.8);
  obstacles.push_back(floor);
  obstacles.push_back(o1);
  obstacles.push_back(o2);
  obstacles.push_back(o3);
  obstacles.push_back(o4);
  obstacles.push_back(o5);

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle: obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(Eigen::Vector3d::Zero());
  addCoordinateFrameToWorld(world);

  ////////////////////////////////////////////////////////////////////////////////
  ////Setup state spaces
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<ompl::multilevel::FactoredSpaceInformationPtr> factors;
  std::vector<RobotPtr> robots;
  std::vector<RobotPtr> child_robots;
  for(size_t k = 0; k < Nrobots; k++) {
    auto robot = MakeRobot<ZeppelinRobot>(world, obstacles);
    auto factor = robot->GetSpaceInformation();

    //Create lower-dimensional abstraction
    auto sphere_robot = MakeRobot<ZeppelinInnerSphereRobot>(world, obstacles);
    hide(sphere_robot->GetSkeleton());
    auto child = sphere_robot->GetSpaceInformation();
    auto projection = std::make_shared<ompl::multilevel::Projection_RNSO2_RN>(factor->getStateSpace(), child->getStateSpace());
    factor->addChild(child, projection);
    child_robots.push_back(sphere_robot);

    factors.push_back(factor);
    robots.push_back(robot);
  }

  auto multi_robot = MultiRobot::MakeMultiRobot(robots);
  auto root = multi_robot->GetSpaceInformation();

  const bool computer_fiber_space = false;
  for(size_t k = 0; k < factors.size(); k++) {
    auto factor = factors.at(k);
    auto projection = std::make_shared<ompl::multilevel::Projection_Subspace>(root->getStateSpace(), factor->getStateSpace(), k);
    ReturnOnFalse(root->addChild(factor, projection, computer_fiber_space), 1);
  }

  auto pairwise_collision_checker = std::make_shared<DartMultiRobotCollisionChecker>(root, world, robots);
  root->setStateValidityChecker(pairwise_collision_checker);
  root->setStateValidityCheckingResolution(0.0001);

  root->printFactorization(std::cout);
  //////////////////////////////////////////////////////////////////////////////////
  //////Create problem definition
  //////////////////////////////////////////////////////////////////////////////////
  auto pdef = CreateMultiDroneProblemDefinition(root, robots, Nrobots);
 
  //////////////////////////////////////////////////////////////////////////////////////
  //////////Planning
  //////////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(root);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(Inf);
  planner->setSmoothIntermediateSolutions(true);
 
  float timeout = 1000.0;
  ompl::base::PlannerStatus status = planner->Planner::solve(timeout);
 
  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  // visualizer.SetCollisionChecker(pairwise_collision_checker->GetCollisionChecker());
  // visualizer.AddPlanner(robot, planner);
  visualizer.AddMultiRobotPlanner(robots, planner);

  visualizer.Run();

  return 0;
}

