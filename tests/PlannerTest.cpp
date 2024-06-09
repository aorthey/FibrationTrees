#include <gtest/gtest.h>

#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "Common.hpp"
#include "DartHelper.hpp"
#include "RunBenchmark.hpp"
#include "TimeOrSolutionTerminationCondition.hpp"

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/Relaxation.h>
#include <ompl/multilevel/datastructures/projections/RN_RM.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/tools/benchmark/Benchmark.h>

class PlannerTest : public testing::Test {
 protected:
  PlannerTest()
  {
    dart::math::Random::setSeed(0);

    world = std::make_shared<dart::simulation::World>();

    auto obstacle = createBox(State3d(+0.5, +0.0, 0.75), 1.0, 1.0, 0.1);
    world->addSkeleton(obstacle);

    robot = MakeRobot<SphereRobot>(world);
    limits = std::make_pair(State3d(-1.0, -1.0, 0.0), State3d(+1.0, +1.0, 0.0));
    robot->SetLimits(limits);
    factor = robot->GetSpaceInformation();

     ////////////////////////////////////////////////////////////////////////////////
     ////Create problem structure
     ////////////////////////////////////////////////////////////////////////////////
    start_state = factor->allocState();
    goal_state = factor->allocState();

    robot->EigenToState(MakeState({-1.0, -1.0, 0.0}), start_state);
    robot->EigenToState(MakeState({+1.0, +1.0, 0.0}), goal_state);

    pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
    pdef->setStartAndGoalStates(start_state, goal_state);

    goal = std::make_shared<ompl::base::GoalState>(factor);
    goal->setState(goal_state);
  }

  dart::simulation::WorldPtr world;
  ompl::base::State* start_state;
  ompl::base::State* goal_state;
  std::shared_ptr<ompl::base::GoalState> goal;
  std::pair<State3d, State3d> limits;
  std::shared_ptr<SphereRobot> robot;
  ompl::base::ProblemDefinitionPtr pdef;
  ompl::multilevel::FactoredSpaceInformationPtr factor;
};

void PrintPlannerOutput(const ompl::tools::Benchmark::PlannerExperiment& p) {
  std::cout << std::string(80,'-') << std::endl;
  std::cout << p.name << std::endl;
  for(const auto& run : p.runs) {
    for(const auto& value : run) {
      std::cout << value.first << ": " << value.second << std::endl;
    }
    std::cout << std::string(20,'-') << std::endl;
  }
  std::cout << " [ProgressPropertyNames]" 
    << std::endl;
  for(const auto& value : p.progressPropertyNames) {
    std::cout << value << std::endl;
  }
  std::cout << " [RunsProgressData]"
    << std::endl;
  for(const auto& progressData : p.runsProgressData) {
    for(const auto& runMap : progressData) {
      for(const auto& value : runMap) {
        std::cout << value.first << ": " << value.second << std::endl;
      }
    }
  }
  for(const auto& value : p.common) {
    std::cout << value.first << ": " << value.second << std::endl;
  }
}

TEST_F(PlannerTest, PlanClearPlanTest) {
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

TEST_F(PlannerTest, BenchmarkTest) {
  ompl::geometric::SimpleSetup setup(factor);

  ompl::base::ScopedState<> scoped_start_state(factor);
  scoped_start_state = start_state;
  setup.setStartState(scoped_start_state);
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

TEST_F(PlannerTest, BenchmarkOutputStringTest) {
  ompl::base::ScopedState<> scoped_start_state(factor);
  scoped_start_state = start_state;

  ompl::geometric::SimpleSetup setup(factor);
  setup.setStartState(scoped_start_state);
  setup.setGoal(goal);

  float timeout = 1.0;
  double memory_limit = 4096;
  int run_count = 3;

  ompl::tools::Benchmark::Request request(timeout, memory_limit, run_count);
  request.simplify = false;
  request.displayProgress = false;

  ompl::tools::Benchmark benchmark(setup);

  auto planner = std::make_shared<ompl::geometric::RRT>(factor);
  planner->setRange(+Inf);
  benchmark.addPlanner(planner);

  auto planner2 = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner2->setRange(+Inf);
  benchmark.addPlanner(planner2);

  benchmark.benchmark(request);
  auto result = benchmark.getRecordedExperimentData();

  EXPECT_EQ(result.planners.size(), 2u);
  EXPECT_EQ(result.planners.front().runs.size(), run_count);
  EXPECT_EQ(result.planners.back().runs.size(), run_count);
  auto p1 = result.planners.front();
  auto p2 = result.planners.back();
  auto p1_run = p1.runs.front();
  auto p2_run = p2.runs.front();

  PrintPlannerOutput(p1);
  PrintPlannerOutput(p2);

  EXPECT_EQ(p1_run.size(), p2_run.size());

  using StatusType = ompl::base::PlannerStatus::StatusType;
  auto expected_result = StatusType::EXACT_SOLUTION;

  EXPECT_EQ(static_cast<StatusType>(std::stoi(p1_run["status ENUM"])), expected_result);
  EXPECT_EQ(static_cast<StatusType>(std::stoi(p2_run["status ENUM"])), expected_result);

  EXPECT_TRUE(static_cast<bool>(std::stoi(p1_run["solved BOOLEAN"])));
  EXPECT_TRUE(static_cast<bool>(std::stoi(p2_run["solved BOOLEAN"])));

  EXPECT_LT(static_cast<double>(std::atof(p1_run["time REAL"].c_str())), 0.5 * timeout);
  EXPECT_LT(static_cast<double>(std::atof(p2_run["time REAL"].c_str())), 0.5 * timeout);


  EXPECT_LT(static_cast<double>(std::atof(p1_run["time REAL"].c_str())), 0.5 * timeout);
  EXPECT_LT(static_cast<double>(std::atof(p2_run["time REAL"].c_str())), 0.5 * timeout);

  EXPECT_GT(static_cast<double>(std::atof(p1_run["solution length REAL"].c_str())), 0.0);
  EXPECT_GT(static_cast<double>(std::atof(p2_run["solution length REAL"].c_str())), 0.0);
  EXPECT_LT(static_cast<double>(std::atof(p1_run["solution length REAL"].c_str())), 10.0);
  EXPECT_LT(static_cast<double>(std::atof(p2_run["solution length REAL"].c_str())), 10.0);
}

TEST_F(PlannerTest, RunBenchmarkWithProblemDefinitionTest) {
  ompl::base::ScopedState<> scoped_start_state(factor);
  scoped_start_state = start_state;

  float timeout = 1.0;
  size_t run_count = 1;

  factor = robot->GetSpaceInformation();

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setRange(+Inf);

  auto benchmark = RunBenchmark("TestBenchmark", factor, scoped_start_state, goal, timeout, run_count, {planner});
  auto result = benchmark.getRecordedExperimentData();

  EXPECT_EQ(result.planners.size(), 1u);
  EXPECT_EQ(result.planners.front().runs.size(), run_count);
  auto p1 = result.planners.front();
  auto p1_run = p1.runs.front();

  using StatusType = ompl::base::PlannerStatus::StatusType;
  auto expected_result = StatusType::EXACT_SOLUTION;

  EXPECT_EQ(static_cast<StatusType>(std::stoi(p1_run["status ENUM"])), expected_result);
  EXPECT_TRUE(static_cast<bool>(std::stoi(p1_run["solved BOOLEAN"])));
  EXPECT_GT(static_cast<double>(std::atof(p1_run["solution length REAL"].c_str())), 0.0);
  EXPECT_LT(static_cast<double>(std::atof(p1_run["solution length REAL"].c_str())), 10.0);
}
