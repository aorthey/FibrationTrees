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
#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>

std::optional<ompl::base::State*> TaskToTotal(const RobotPtr& root_robot, 
  const RobotPtr& leaf_robot, const StateXd& vector) {
  auto root = root_robot->GetSpaceInformation();
  auto leaf = leaf_robot->GetSpaceInformation();
  ompl::base::State *leaf_state = leaf->allocState();
  leaf_robot->EigenToState(vector, leaf_state);
  //leaf->printState(leaf_state);
  std::unordered_map<std::string, ompl::base::State*> leaf_states;
  leaf_states[leaf->getName()] = leaf_state;
  auto maybe_state = ComputeValidTotalState(root, leaf_states);
  leaf->freeState(leaf_state);
  return maybe_state;
}

const float kZheight = 0.5;
const float kAccuracy = 0.02;

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating obstacles
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  dart::dynamics::SkeletonPtr floor = createFloor(-0.255);
  dart::dynamics::SkeletonPtr box = createBox(State3d(0,0,0), 0.3, 0.3, 0.8);
  obstacles.push_back(floor);
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

  auto robot_in_time = MakeRobot<TimeBasedMobileKukaRobotTaskSpace>(world, obstacles);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor1 = robot_in_time->GetSpaceInformation();
  factor1->printFactorization(std::cout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////

  auto start = factor1->allocState();
  auto goal = factor1->allocState();

  robot_in_time->EigenToState(GetRandomPosition(robot_in_time->GetSkeleton()), start);
  robot_in_time->EigenToState(GetRandomPosition(robot_in_time->GetSkeleton()), goal);

  robot_in_time->TimeToState(0.0, start);
  robot_in_time->TimeToState(5.0, goal);

  factor1->printState(start);
  factor1->printState(goal);

  world->addSimpleFrame(createSphereFrame(robot_in_time->GetFK(start).front(), 0.01));
  world->addSimpleFrame(createSphereFrame(robot_in_time->GetFK(goal).front(), 0.01));

  factor1->printSettings(std::cout);

  std::cout << factor1->distance(start, goal) << std::endl;

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor1);
  pdef->setStartAndGoalStates(start, goal, 0.1);

   ////////////////////////////////////////////////////////////////////////////////
   ////Planning
   ////////////////////////////////////////////////////////////////////////////////
   auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor1);
   //auto planner = std::make_shared<ompl::geometric::RRTConnect>(factor1);
   planner->setProblemDefinition(pdef);
   planner->setup();
   planner->setRange(+Inf);
   planner->setSmoothIntermediateSolutions();

   float timeout = 100.0;

   auto ptc = ompl::base::plannerOrTerminationCondition(
           ompl::base::exactSolnPlannerTerminationCondition(pdef),
           ompl::base::timedPlannerTerminationCondition(timeout)
       );

   ompl::base::PlannerStatus status = planner->solve(ptc);

  Visualizer visualizer(world);
  visualizer.AddPlanner(robot_in_time, planner);
  visualizer.SetCollisionChecker(robot_in_time->GetCollisionChecker());
  visualizer.Run();

  factor1->freeState(start);
  factor1->freeState(goal);
  return 0;
}
