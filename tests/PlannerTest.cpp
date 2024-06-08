#include <gtest/gtest.h>

#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "Common.hpp"
#include "DartHelper.hpp"
#include "TimeOrSolutionTerminationCondition.hpp"

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/tools/benchmark/Benchmark.h>

TEST(PlannerTest, PlanClearPlanTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto robot = MakeRobot<SphereRobot>(world);
  const auto limits = std::make_pair(State3d(-1.0, -1.0, 0.0), State3d(+1.0, +1.0, 0.0));
  robot->SetLimits(limits);
  auto factor = robot->GetSpaceInformation();

   ////////////////////////////////////////////////////////////////////////////////
   ////Create problem structure
   ////////////////////////////////////////////////////////////////////////////////
  auto start_state = factor->allocState();
  auto goal_state = factor->allocState();

  robot->EigenToState(MakeState({-1.0, -1.0, 0.0}), start_state);
  robot->EigenToState(MakeState({+1.0, +1.0, 0.0}), goal_state);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->setStartAndGoalStates(start_state, goal_state);

   ////////////////////////////////////////////////////////////////////////////////
   ////Planning
   ////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(+Inf);

  float timeout = 1.0;
  dart::math::Random::setSeed(0);

  EXPECT_TRUE(planner->solve(TimeOrSolutionTerminationCondition(pdef, timeout)));
  planner->clear();
  pdef->clearSolutionPaths();
  EXPECT_TRUE(planner->solve(TimeOrSolutionTerminationCondition(pdef, timeout)));
}

TEST(PlannerTest, BenchmarkTest) {
  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::simulation::WorldPtr world(new dart::simulation::World);
  auto obstacle = createBox(State3d(+0.5, +0.0, 0.75), 1.0, 1.0, 0.1);
  world->addSkeleton(obstacle);

  dart::math::Random::setSeed(0);
  auto robot = MakeRobot<SphereRobot>(world);
  const auto limits = std::make_pair(State3d(-1.0, -1.0, 0.0), State3d(+1.0, +1.0, 0.0));
  robot->SetLimits(limits);
  auto factor = robot->GetSpaceInformation();

   ////////////////////////////////////////////////////////////////////////////////
   ////Create problem structure
   ////////////////////////////////////////////////////////////////////////////////

  ompl::base::ScopedState<> start_state(factor);
  auto goal_state = factor->allocState();

  robot->EigenToState(MakeState({-1.0, -1.0, 0.0}), start_state.get());
  robot->EigenToState(MakeState({+1.0, +1.0, 0.0}), goal_state);

  factor->printState(start_state.get());
  auto goal = std::make_shared<ompl::base::GoalState>(factor);
  goal->setState(goal_state);

   ////////////////////////////////////////////////////////////////////////////////
   ////Benchmarking
   ////////////////////////////////////////////////////////////////////////////////

  ompl::geometric::SimpleSetup setup(factor);
  setup.setStartState(start_state);
  setup.setGoal(goal);

  float timeout = 1.0;
  double memory_limit = 4096;
  int run_count = 2;

  ompl::tools::Benchmark::Request request(timeout, memory_limit, run_count);
  request.simplify = false;
  request.displayProgress = false;

  ompl::tools::Benchmark benchmark(setup);

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setRange(+Inf);
  benchmark.addPlanner(planner);

  benchmark.benchmark(request);
  auto result = benchmark.getRecordedExperimentData();

  EXPECT_EQ(result.planners.size(), 1u);
  EXPECT_EQ(result.planners.front().runs.size(), 2u);
  auto run1 = result.planners.front().runs.at(0);
  auto run2 = result.planners.front().runs.at(1);

  for(const auto& value : run1) {
    std::cout << value.first << ": " << value.second << std::endl;
  }
  std::cout << std::string(80, '-') << std::endl;
  for(const auto& value : run2) {
    std::cout << value.first << ": " << value.second << std::endl;
  }

  using StatusType = ompl::base::PlannerStatus::StatusType;
  auto expected_result = StatusType::EXACT_SOLUTION;
  EXPECT_EQ(static_cast<StatusType>(std::stoi(run1["status ENUM"])), expected_result);
  EXPECT_EQ(static_cast<StatusType>(std::stoi(run2["status ENUM"])), expected_result);
  EXPECT_TRUE(static_cast<bool>(std::stoi(run1["solved BOOLEAN"])));
  EXPECT_TRUE(static_cast<bool>(std::stoi(run2["solved BOOLEAN"])));
  EXPECT_LT(static_cast<double>(std::atof(run1["time REAL"].c_str())), 0.5 * timeout);
  EXPECT_LT(static_cast<double>(std::atof(run2["time REAL"].c_str())), 0.5 * timeout);
}
