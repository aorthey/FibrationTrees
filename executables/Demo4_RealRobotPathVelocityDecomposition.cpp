#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "TaskSpaceProjection.hpp"
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
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>

std::optional<ompl::base::State*> TaskToTotal(const RobotPtr& root_robot, 
  const RobotPtr& leaf_robot, const StateXd& vector) {
  auto root = root_robot->GetSpaceInformation();
  auto leaf = leaf_robot->GetSpaceInformation();
  ompl::base::State *leaf_state = leaf->allocState();
  leaf_robot->EigenToState(vector, leaf_state);
  leaf->printState(leaf_state);
  std::unordered_map<std::string, ompl::base::State*> leaf_states;
  leaf_states[leaf->getName()] = leaf_state;
  auto maybe_state = ComputeValidTotalState(root, leaf_states);
  leaf->freeState(leaf_state);
  return maybe_state;
}

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating obstacles
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  dart::dynamics::SkeletonPtr floor = createFloor(-0.255);
  dart::dynamics::SkeletonPtr box = createBox(State3d(0,0,0), 0.3, 0.3, 0.5);
  obstacles.push_back(floor);
  obstacles.push_back(box);
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

  auto robot = MakeRobot<MobileKukaRobotTaskSpace>(world, obstacles);

  auto robot_in_time = MakeRobot<TimeBasedMobileKukaRobotTaskSpace>(world, obstacles);
  robot_in_time->SetSpaceInformationFromRobot(robot, world, obstacles);
  auto point = MakeRobot<SphereRobot>(world, obstacles);

  const auto limits = std::make_pair(State3d(-1.5, -1.5, 0.5-Epsilon), State3d(1.5, +1.5, 0.5+Epsilon));
  point->SetLimits(limits);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  ////////////////////////////////////////////////////////////////////////////////
  auto factor1 = robot_in_time->GetSpaceInformation();
  auto factor2 = robot->GetSpaceInformation();
  auto factor3 = point->GetSpaceInformation();

  auto projection_1_2 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor1, factor2, 0);
  auto projection_2_3 = std::make_shared<TaskSpaceProjection>(factor2, factor3, robot);

  ReturnIntOnFalse(factor1->addChild(factor2, projection_1_2));
  ReturnIntOnFalse(factor2->addChild(factor3, projection_2_3));

  factor1->printFactorization(std::cout);

  ////////////////////////////////////////////////////////////////////////////////
  ////Create planning problem
  ////////////////////////////////////////////////////////////////////////////////
  //ompl::base::State *task_goal = factor3->allocState();

  //ompl::base::State *start = factor1->allocState();
  //ompl::base::State *goal = factor1->allocState();

  auto point_start = MakeState({1.3, 1.3, 0.5});
  auto point_goal = MakeState({-1.3, -1.3, 0.5});
  ValueOrReturnInt(goal, TaskToTotal(robot_in_time, point, point_goal));
  ValueOrReturnInt(start, TaskToTotal(robot_in_time, point, point_start));

  factor1->printState(start);
  factor1->printState(goal);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor1);
  pdef->setStartAndGoalStates(start, goal, 0.01);

  world->addSimpleFrame(createSphereFrame(point_start, 0.01));
  world->addSimpleFrame(createSphereFrame(point_goal, 0.01));

  //  ////////////////////////////////////////////////////////////////////////////////
  //  ////Planning
  //  ////////////////////////////////////////////////////////////////////////////////
  //  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor1);
  //  planner->setProblemDefinition(pdef);
  //  planner->setup();
  //  planner->setRange(+Inf);

  //  float timeout = 100.0;

  //  auto ptc = ompl::base::plannerOrTerminationCondition(
  //          ompl::base::exactSolnPlannerTerminationCondition(pdef),
  //          ompl::base::timedPlannerTerminationCondition(timeout)
  //      );

  //  ompl::base::PlannerStatus status = planner->solve(ptc);

  Visualizer visualizer(world);
  //visualizer.AddPlanner(robot, planner);
  visualizer.SetCollisionChecker(robot_in_time->GetCollisionChecker());
  //visualizer.AddPath(point, planner->getProblemDefinition(factor3->getName())->getSolutionPath(), State3d(1, 1, 0));
  visualizer.Run();
  //}
  return 0;
}
