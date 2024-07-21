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
#include "robots/TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "TimeGoal.hpp"
#include "TimeOrSolutionTerminationCondition.hpp"
#include "RunBenchmark.hpp"

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

const State3d kObstacleColor = State3d(0.8, 0.8, 0.8);
const Eigen::Vector4d kObstacleColor4d(0.8, 0.8, 0.8, 1.0);

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

std::pair<RobotPtr, ompl::base::PathPtr> MakeDynamicObstacle(
    const std::vector<double>& start_xy,
    const std::vector<double>& goal_xy,
    const dart::simulation::WorldPtr& world, 
    const std::vector<dart::dynamics::SkeletonPtr>& static_obstacles
    ) {
  auto robot = MakeRobot<TimeBasedMobileKukaRobotTaskSpace>(world, static_obstacles);

  auto state1 = MakeState({start_xy.at(0), start_xy.at(1), start_xy.at(2), -0.5, 0.0, +0.57, +1, 2, 0.24, -0.21});
  state1.time = 0.0;
  auto state2 = MakeState({goal_xy.at(0), goal_xy.at(1), goal_xy.at(2), +0.5, 0.0, -0.57, +1, 2, 0.24, +0.21});
  state2.time = 20.0;

  auto si = robot->GetSpaceInformation();
  auto start = si->allocState();
  auto goal = si->allocState();

  robot->EigenToState(state1, start);
  robot->EigenToState(state2, goal);

  auto path = std::make_shared<ompl::geometric::PathGeometric>(si, start, goal);

  //ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
  //pdef->setStartAndGoalStates(start, goal, 0.5);

  //auto planner = std::make_shared<ompl::geometric::RRTConnect>(si);
  //planner->setProblemDefinition(pdef);
  //planner->setup();
  //planner->setRange(+Inf);

  //float timeout = 10.0;
  //auto ptc = TimeOrSolutionTerminationCondition(pdef, timeout);

  //ompl::base::PlannerStatus status = planner->solve(ptc);

  //if(!pdef->hasApproximateSolution() &&
  //   !pdef->hasExactSolution())
  //{
  //  throw "Invalid";
  //}
  //auto path = pdef->getSolutionPath();
  //auto geom_path = path->as<ompl::geometric::PathGeometric>();
  //auto path_simplifier = std::make_shared<ompl::geometric::PathSimplifier>(si);
  //path_simplifier->simplifyMax(*geom_path);
  //path_simplifier->smoothBSpline(*geom_path);

  ////DEBUG
  path->interpolate(100);
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
   OMPL_INFORM("Found path with %d states.", path->getStateCount());
   for(const auto& state : path->getStates()) {
     if(!robot->IsValid(state)) {
       OMPL_ERROR("%s", std::string(40, '-').c_str());
       OMPL_ERROR("Invalid state");
       si->printState(state);
       OMPL_ERROR("%s", std::string(40, '-').c_str());
       throw "Invalid";
     }
   }
  return std::make_pair(robot, path);
}

const float kZheight = 0.7;
const float kHeightPadding = 0.05;

int main(int argc, char* argv[]) {

  ////////////////////////////////////////////////////////////////////////////////
  ////Create static_obstacles
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> static_obstacles;
  const float kObstacleSize = 0.3;
  static_obstacles.push_back(createBox(State3d(-1.1,0,0), kObstacleSize, kObstacleSize, 0.8));
  static_obstacles.push_back(createBox(State3d(+0.0,0,0), kObstacleSize, kObstacleSize, 0.8));
  //static_obstacles.push_back(createBox(State3d(+1.1,0,0), kObstacleSize, kObstacleSize, 0.8));

  auto floor = createFloor(-0.255);
  static_obstacles.push_back(floor);
  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle : static_obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(State3d::Zero());
  addCoordinateFrameToWorld(world);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create dynamic_obstacles
  ////////////////////////////////////////////////////////////////////////////////

  std::vector<std::pair<RobotPtr, ompl::base::PathPtr>> dynamic_obstacles;
  //dynamic_obstacles.push_back(MakeDynamicObstacle({-0.75, +1.0}, {-0.75, -1.0}, world, static_obstacles));
  dynamic_obstacles.push_back(MakeDynamicObstacle({+0.5, +1.0, -0.5*M_PI}, {+0.5, -1.0, +0.5*M_PI}, world, static_obstacles));
  dynamic_obstacles.push_back(MakeDynamicObstacle({-0.5, +1.0, 0.5*M_PI}, {-0.5, -1.0, -0.25*M_PI}, world, static_obstacles));

  std::vector<dart::dynamics::SkeletonPtr> all_obstacles;
  for(const auto& obstacle : static_obstacles) {
    all_obstacles.push_back(obstacle);
  }
  for(const auto& obstacle : dynamic_obstacles) {
    all_obstacles.push_back(obstacle.first->GetSkeleton());
  }

  ////////////////////////////////////////////////////////////////////////////////
  ////Create Robots
  ////////////////////////////////////////////////////////////////////////////////

  auto robot_in_time = MakeRobot<TimeBasedMobileKukaRobotTaskSpaceWithDynamicalConstraints>(world, all_obstacles);
  for(const auto& dynamic_obstacle : dynamic_obstacles) {
    robot_in_time->AddDynamicalObstacle(dynamic_obstacle);
  }
  auto robot = MakeRobot<MobileKukaRobotTaskSpace>(world, static_obstacles);
  auto tcp_robot = MakeRobot<SphereRobot>(world, static_obstacles);

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
  auto maybe_total_start = TaskToTotal(robot_in_time, tcp_robot, MakeState({-1.0, -1.0, kZheight-0.5*kHeightPadding}));
  if(!maybe_total_start.has_value()) {
    return 1;
  }
  auto maybe_total_goal = TaskToTotal(robot_in_time, tcp_robot, MakeState({+1.0, +1.0, kZheight-0.5*kHeightPadding}));
  if(!maybe_total_goal.has_value()) {
    return 1;
  }
  auto start = maybe_total_start.value();
  auto goal = maybe_total_goal.value();

  robot_in_time->TimeToState(0.0, start);
  robot_in_time->TimeToState(robot_in_time->GetTMax(), goal);

  factor1->printState(start);
  factor1->printState(goal);

  auto fk_start = robot_in_time->GetFK(robot_in_time->StateToEigen(start)).front();
  auto fk_goal = robot_in_time->GetFK(robot_in_time->StateToEigen(goal)).front();

  world->addSimpleFrame(createSphereFrame(fk_start, 0.01));
  world->addSimpleFrame(createSphereFrame(fk_goal, 0.01));

  auto time_goal = std::make_shared<TimeGoal>(robot_in_time, robot_in_time->GetVMax(), start, goal);
  time_goal->setThreshold(0.5);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor1);
  pdef->addStartState(start);
  pdef->setGoal(time_goal);

  factor1->printFactorization(std::cout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Planning
  ////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor1, 1.0);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(+Inf);
  planner->setSmoothIntermediateSolutions(false);
  float timeout = 300.0;

  //////////////////////////////////////////////////////////////////////////////////////
  //////////Benchmark
  //////////////////////////////////////////////////////////////////////////////////////
  // auto planner2 = std::make_shared<ompl::geometric::RRTtask>(factor1);
  // planner2->setProblemDefinition(pdef);
  // planner2->setup();

  // size_t run_count = 10;
  // auto name = "Scenario6";
  // ompl::base::ScopedState<> scoped_start_state(factor1);
  // scoped_start_state = pdef->getStartState(0);
  // auto benchmark = RunBenchmark(name, factor1, scoped_start_state, pdef->getGoal(), timeout, run_count, {planner, planner2});
  // SaveBenchmarkToDatabase(name, benchmark);
  // return 0;

  auto ptc = TimeOrSolutionTerminationCondition(pdef, timeout);

  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_INFO);
  ompl::base::PlannerStatus status = planner->solve(ptc);

  Visualizer visualizer(world);
  visualizer.AddPlanner(robot_in_time, planner);

  for(const auto& dynamic_obstacle : dynamic_obstacles) {
    visualizer.AddPath(dynamic_obstacle.first, dynamic_obstacle.second, kObstacleColor);
    changeBodyColor(dynamic_obstacle.first->GetSkeleton(), kObstacleColor4d);
  }

  if(pdef) 
  {
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    auto end_point = pgeo.getStates().back();
    auto end_time = robot_in_time->StateToTime(end_point);
    OMPL_INFORM("Found path with end time %f", end_time);
    visualizer.SetEndTime(end_time);
  }

  visualizer.SetCollisionChecker(robot_in_time->GetCollisionChecker());
  visualizer.Run();

  factor1->freeState(start);
  factor1->freeState(goal);
  return 0;
}

