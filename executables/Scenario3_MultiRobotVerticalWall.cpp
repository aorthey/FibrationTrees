#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "spaces/TaskSpace.hpp"
#include "TaskSpaceGoal.hpp"
#include "projections/ProjectionTaskSpace.hpp"
#include "validators/MotionValidatorTaskSpaceMultiRobot.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "gui/Visualizer.hpp"
#include "OmplHelper.hpp"
#include "samplers/TaskSpaceMultiRobotSampler.hpp"
#include "RunBenchmark.hpp"
#include "robots/KukaRobotTaskSpace.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/FactoredGoal.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>

const float kRobotRobotDistance = 0.3;

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  obstacles.push_back(createFloor());
  obstacles.push_back(createBox(State3d(+0.5, +0.0, 0.75), 0.16, 2.0, 1.5));

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  for(const auto& obstacle : obstacles) {
    world->addSkeleton(obstacle);
  }
  world->setGravity(State3d::Zero());

  auto robot1 = MakeRobot<KukaRobotTaskSpace>(world, obstacles);
  auto robot2 = MakeRobot<KukaRobotTaskSpace>(world, obstacles);
  auto point1 = MakeRobot<SphereRobot>(world, obstacles);
  auto point2 = MakeRobot<SphereRobot>(world, obstacles);

  Eigen::Isometry3d transform1(Eigen::Isometry3d::Identity());
  transform1.translation() = State3d{0.0, -0.5*kRobotRobotDistance, 0};
  robot1->GetSkeleton()->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(transform1);

  Eigen::Isometry3d transform2(Eigen::Isometry3d::Identity());
  transform2.translation() = State3d{0.0, +0.5*kRobotRobotDistance, 0};
  robot2->GetSkeleton()->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(transform2);

  const auto limits1 = std::make_pair(State3d(0.38, -0.5, 0.0), State3d(0.42, +0.5, 2.0));
  const auto limits2 = std::make_pair(State3d(0.38, -0.5, 0.0), State3d(0.42, +0.5, 2.0));
  point1->SetLimits(limits1);
  point2->SetLimits(limits2);
  hide(point1->GetSkeleton());
  hide(point2->GetSkeleton());

  std::vector<RobotPtr> robots = {robot1, robot2};

  auto multi_robot = MultiRobot::MakeMultiRobot(robots);

  ////////////////////////////////////////////////////////////////////////////////
  ////OMPL Setup
  //
  //       factor (both robots)
  //         /            \
  //       /                \
  //   factor1 (robot1)   factor2 (robot2)
  //      |                  |
  //   child1 (point1)    child2 (point2)
  //
  ////////////////////////////////////////////////////////////////////////////////

  auto factor1 = robot1->GetSpaceInformation();
  auto factor2 = robot2->GetSpaceInformation();

  auto child1 = point1->GetSpaceInformation();
  auto child2 = point2->GetSpaceInformation();

  ompl::multilevel::ProjectionPtr projection_child1 = std::make_shared<ProjectionTaskSpace>(factor1, child1, robot1);
  factor1->addChild(child1, projection_child1);

  ompl::multilevel::ProjectionPtr projection_child2 = std::make_shared<ProjectionTaskSpace>(factor2, child2, robot2);
  factor2->addChild(child2, projection_child2);

  auto factor = multi_robot->GetSpaceInformation();

  auto projection1 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor->getStateSpace(), factor1->getStateSpace(), 0);
  auto projection2 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor->getStateSpace(), factor2->getStateSpace(), 1);

  bool computer_fiber_space = false;
  ReturnOnFalse(factor->addChild(factor1, projection1, computer_fiber_space), 1);
  ReturnOnFalse(factor->addChild(factor2, projection2, computer_fiber_space), 1);

  ompl::base::MotionValidatorPtr motion_validator = std::make_shared<MotionValidatorTaskSpaceMultiRobot>(factor);
  factor->setMotionValidator(motion_validator);

  auto pairwise_collision_checker = std::make_shared<DartMultiRobotCollisionChecker>(factor, world, robots);
  factor->setStateValidityChecker(pairwise_collision_checker);
  factor->setStateValidityCheckingResolution(0.001);

  //////////////////////////////////////////////////////////////////////////////////
  //////Create start/goal states and propagate them upwards (lift through the
  //complete fibration trees hierarchy)
  //////////////////////////////////////////////////////////////////////////////////
  auto task_start1_eigen = MakeState({0.4, -0.2, 1.0});
  auto task_goal1_eigen = MakeState({0.4, +0.1, 0.7});
  auto task_start2_eigen = MakeState({0.4, +0.0, 1.0});
  auto task_goal2_eigen = MakeState({0.4, -0.2, 0.6});

  auto task_start1 = child1->allocState();
  auto task_start2 = child2->allocState();
  auto task_goal1 = child1->allocState();
  auto task_goal2 = child2->allocState();

  point1->EigenToState(task_start1_eigen, task_start1);
  point2->EigenToState(task_start2_eigen, task_start2);
  point1->EigenToState(task_goal1_eigen, task_goal1);
  point2->EigenToState(task_goal2_eigen, task_goal2);

  std::unordered_map<std::string, ompl::base::State*> task_space_start_states;
  task_space_start_states[child1->getName()] = task_start1;
  task_space_start_states[child2->getName()] = task_start2;

  auto maybe_start = ComputeValidTotalState(factor, task_space_start_states);
  if(!maybe_start.has_value()){
    OMPL_ERROR("Could not compute valid start.");
    return 1;
  }
  auto start = maybe_start.value();
  
  OMPL_INFORM("Found start state:");
  factor->printState(start);

  std::unordered_map<std::string, ompl::base::State*> task_space_goal_states;
  task_space_goal_states[child1->getName()] = task_goal1;
  task_space_goal_states[child2->getName()] = task_goal2;

  auto maybe_goal = ComputeValidTotalState(factor, task_space_goal_states);
  if(!maybe_goal.has_value()){
    OMPL_ERROR("Could not compute valid goal.");
    return 1;
  }
  auto goal = maybe_goal.value();
  OMPL_INFORM("Found goal state:");
  factor->printState(goal);

  const float kGoalRegionRadius = 0.02;
  const float kGoalRegionHeight = 0.001;
  const auto kGoalRegionRotation = State3d(0.0, M_PI*0.5, 0.0);

  world->addSimpleFrame(createCylinderFrame(task_goal1_eigen, kGoalRegionRotation, kGoalRegionRadius, kGoalRegionHeight, State4d(1.0, 0.4, 0.4, 0.5)));
  world->addSimpleFrame(createCylinderFrame(task_goal2_eigen, kGoalRegionRotation, kGoalRegionRadius, kGoalRegionHeight, State4d(0.4, 1.0, 0.4, 0.5)));
  world->addSimpleFrame(createCylinderFrame(task_start1_eigen, kGoalRegionRotation, 0.01, kGoalRegionHeight, State4d(0.8, 0.1, 0.1, 1.0)));
  world->addSimpleFrame(createCylinderFrame(task_start2_eigen, kGoalRegionRotation, 0.01, kGoalRegionHeight, State4d(0.1, 0.8, 0.1, 1.0)));

  //////////////////////////////////////////////////////////////////////////////////
  //////Create factored task space goals
  //////////////////////////////////////////////////////////////////////////////////
  auto goalStates = factor->allocChildStates();
  factor->project(goal, goalStates);
  auto goal1 = goalStates.at(factor1->getName());
  auto goal2 = goalStates.at(factor2->getName());

  auto goal_region1 = std::make_shared<TaskSpaceGoal>(factor1, goal1, projection_child1);
  goal_region1->setThreshold(0.1);
  auto goal_region2 = std::make_shared<TaskSpaceGoal>(factor2, goal2, projection_child2);
  goal_region2->setThreshold(0.1);

  std::unordered_map<std::string, ompl::base::GoalSampleableRegionPtr> goal_regions;
  goal_regions[factor1->getName()] = goal_region1;
  goal_regions[factor2->getName()] = goal_region2;

  auto goal_region = std::make_shared<ompl::base::FactoredGoal>(factor, goal_regions);
  goal_region->setThreshold(0.1);
  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoal(goal_region);

  //////////////////////////////////////////////////////////////////////////////////
  //////Planning
  //////////////////////////////////////////////////////////////////////////////////
  factor->printFactorization(std::cout);

  std::vector<std::pair<State3d, State3d>> limits;
  limits.push_back(limits1);
  limits.push_back(limits2);

  factor->getStateSpace()->setStateSamplerAllocator(
          ///std::bind(&allocateTaskSpaceMultiRobotSampler, multi_robot, robots, limits));
          std::bind(&allocateTaskSpaceMultiRobotSampler, multi_robot, robots, limits));

  float timeout = 600.0;

  auto planner1 = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner1->setProblemDefinition(pdef);
  planner1->setup();
  planner1->setRange(Inf);
  planner1->setSmoothIntermediateSolutions(false);

  // auto planner2 = std::make_shared<ompl::geometric::RRTtask>(factor);
  // planner2->setProblemDefinition(pdef);
  // planner2->setup();

  // size_t run_count = 10;
  // auto name = "Scenario3";
  // ompl::base::ScopedState<> scoped_start_state(factor);
  // scoped_start_state = start;
  // auto benchmark = RunBenchmark(name, factor, scoped_start_state, goal_region, timeout, run_count, {planner1, planner2});
  // SaveBenchmarkToDatabase(name, benchmark);
  // return 0;

  auto ptc = TimeOrSolutionPtc(pdef, timeout);
  ompl::base::PlannerStatus status = planner1->solve(ptc);

  //////////////////////////////////////////////////////////////////////////////////
  //////Visualize
  //////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  visualizer.SetCollisionChecker(pairwise_collision_checker->GetCollisionChecker());
  visualizer.AddPlanner(multi_robot, planner1);

  visualizer.Run();
  return 0;
}
