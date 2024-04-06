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
#include <ompl/multilevel/datastructures/projections/TimeBasedProjection.h>

std::optional<ompl::base::State*> TaskToTotal(const RobotPtr& root_robot, 
  const RobotPtr& leaf_robot, const StateXd& vector) {
  auto root = root_robot->GetSpaceInformation();
  auto leaf = leaf_robot->GetSpaceInformation();
  ompl::base::State *leaf_state = leaf->allocState();
  leaf_robot->EigenToState(vector, leaf_state);
  std::unordered_map<std::string, ompl::base::State*> leaf_states;
  leaf_states[leaf->getName()] = leaf_state;
  auto maybe_state = ComputeValidTotalState(root, leaf_states);
  leaf->freeState(leaf_state);
  return maybe_state;
}

const float kZheight = 0.7;
const float kHeightPadding = 0.05;

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating obstacles
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  dart::dynamics::SkeletonPtr floor = createFloor(-0.255);
  //dart::dynamics::SkeletonPtr ceiling = createFloor(+kZheight+kHeightPadding);
  dart::dynamics::SkeletonPtr box = createBox(State3d(0,0,0), 0.3, 0.3, 0.8);
  obstacles.push_back(floor);
  obstacles.push_back(box);
  //obstacles.push_back(ceiling);
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
  auto robot = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles);
  auto tcp_robot = MakeRobot<SphereRobot>(world, obstacles);

  hide(robot->GetSkeleton());
  hide(tcp_robot->GetSkeleton());

  const auto limits = std::make_pair(State3d(-1.5, -1.5, kZheight-kHeightPadding), State3d(+1.5, +1.5, kZheight+kHeightPadding));
  tcp_robot->SetLimits(limits);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor1 = robot_in_time->GetSpaceInformation();
  auto factor2 = robot->GetSpaceInformation();
  auto factor3 = tcp_robot->GetSpaceInformation();

  auto projection_time_dimension = std::make_shared<ompl::multilevel::Projection_TimeBased>(factor1->getStateSpace(), factor2->getStateSpace());
  factor1->addChild(factor2, projection_time_dimension);

  auto projection_to_tcp_robot = std::make_shared<ProjectionTaskSpace>(factor2, factor3, robot);
  factor2->addChild(factor3, projection_to_tcp_robot);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////
  auto maybe_total_start = TaskToTotal(robot_in_time, tcp_robot, MakeState({-1.0, -1.0, kZheight-kHeightPadding}));
  if(!maybe_total_start.has_value()) {
    return 1;
  }
  auto maybe_total_goal = TaskToTotal(robot_in_time, tcp_robot, MakeState({+1.0, +1.0, kZheight-kHeightPadding}));
  if(!maybe_total_goal.has_value()) {
    return 1;
  }
  auto start = maybe_total_start.value();
  auto goal = maybe_total_goal.value();

  robot_in_time->TimeToState(0.0, start);
  robot_in_time->TimeToState(5.0, goal);

  factor1->printState(start);
  factor1->printState(goal);

  auto fk_start = robot_in_time->GetFK(start).front();
  auto fk_goal = robot_in_time->GetFK(goal).front();

  world->addSimpleFrame(createSphereFrame(fk_start, 0.01));
  world->addSimpleFrame(createSphereFrame(fk_goal, 0.01));

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor1);
  pdef->setStartAndGoalStates(start, goal, 0.1);

  factor1->printFactorization(std::cout);

   ////////////////////////////////////////////////////////////////////////////////
   ////Planning
   ////////////////////////////////////////////////////////////////////////////////
   auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor1);
   planner->setProblemDefinition(pdef);
   planner->setup();
   planner->setRange(+Inf);
   //planner->setSmoothIntermediateSolutions();

   float timeout = 10.0;

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

