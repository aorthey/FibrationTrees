#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "robots/ZeppelinRobot.hpp"
#include "robots/ZeppelinInnerSphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "MakeSpaceInformation.hpp"
#include "Utils.hpp"

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
  float x = 0.0;
  float y = 0.0;
  float z = 0.1;
  float yaw = 0.57;

  float y_step = 0.2;

  auto startStates = factor->allocChildStates();
  auto goalStates = factor->allocChildStates();

  for(const auto& robot : robots) {
    const auto name = robot->GetSpaceInformation()->getName();

    auto start = MakeEigen({x, y, z, yaw});
    auto start_state = AllocStateFromEigen(robot, start);
    startStates[name] = start_state;

    auto goal = MakeEigen({x - 1.5, y, z, yaw});
    auto goal_state = AllocStateFromEigen(robot, goal);
    goalStates[name] = goal_state;

    y += y_step;
  }

  auto start = factor->allocState();
  factor->lift(startStates, start);

  auto goal = factor->allocState();
  factor->lift(goalStates, goal);

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
  dart::dynamics::SkeletonPtr o1 = createCylinder(Eigen::Vector3d(-1.0, 0.0, 0.0), 0.3, 1.5);
  obstacles.push_back(floor);
  obstacles.push_back(o1);

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
  for(size_t k = 0; k < Nrobots; k++) {
    auto robot = MakeRobot<ZeppelinRobot>(world, obstacles);
    auto sphere_robot = MakeRobot<ZeppelinInnerSphereRobot>(world, obstacles);

    hide(sphere_robot->GetSkeleton());

    auto factor = robot->GetSpaceInformation();
    auto child = sphere_robot->GetSpaceInformation();

    auto projection = std::make_shared<ompl::multilevel::Projection_RNSO2_RN>(factor->getStateSpace(), child->getStateSpace());
    factor->addChild(child, projection);
    factors.push_back(factor);
    robots.push_back(robot);
  }

  auto root = MakeMultiRobotSpaceInformation(factors);

  const bool computer_fiber_space = false;
  for(size_t k = 0; k < factors.size(); k++) {
    auto factor = factors.at(k);
    auto projection = std::make_shared<ompl::multilevel::Projection_Subspace>(root->getStateSpace(), factor->getStateSpace(), k);
    ReturnOnFalse(root->addChild(factor, projection, computer_fiber_space), 1);
  }

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
  planner->setRange(0.1);
 
  float timeout = 10.0;
  ompl::base::PlannerStatus status = planner->Planner::solve(timeout);
 
  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  // visualizer.SetCollisionChecker(robot->GetCollisionChecker());
  // visualizer.AddPlanner(robot, planner);
  visualizer.AddMultiRobotPlanner(robots, planner);
  visualizer.Run();

  return 0;
}

