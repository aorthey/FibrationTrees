#include "RunBenchmark.hpp"

#include "Common.hpp"
#include "FilePath.hpp"

#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/base/Planner.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/multilevel/planners/RRTtask.h>
#include <ompl/multilevel/planners/FibrationRRT.h>

ompl::tools::Benchmark RunBenchmark(
  const std::string name,
  const ompl::multilevel::FactoredSpaceInformationPtr& factor,
  const ompl::base::ScopedState<>& start,
  const ompl::base::GoalPtr& goal,
  double timeout,
  size_t run_count) {

  auto planner1 = std::make_shared<ompl::multilevel::RRTtask>(factor);
  auto planner2 = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner2->setRange(+Inf);
  return RunBenchmark(name, factor, start, goal, timeout, run_count, {planner1, planner2});
}

ompl::tools::Benchmark RunBenchmark(
  const std::string name,
  const ompl::multilevel::FactoredSpaceInformationPtr& factor,
  const ompl::base::ScopedState<>& start,
  const ompl::base::GoalPtr& goal,
  double timeout,
  size_t run_count,
  const std::vector<ompl::base::PlannerPtr>& planners,
  const std::optional<ompl::tools::Benchmark::PreSetupEvent> pre_setup_event) {

  ompl::geometric::SimpleSetup setup(factor);
  setup.setStartState(start);
  setup.setGoal(goal);

  double runtime_limit = timeout;
  double memory_limit = 4096*1e4;

  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_DEV2);
  ompl::tools::Benchmark::Request request(runtime_limit, memory_limit, run_count);
  request.simplify = false;
  request.timeBetweenUpdates = 0.01;
  request.displayProgress = true;

  ompl::tools::Benchmark benchmark(setup, name);

  if(pre_setup_event.has_value()) {
    benchmark.setPreRunEvent(pre_setup_event.value());
  }

  for(const auto& planner : planners) {
    benchmark.addPlanner(planner);
  }

  benchmark.benchmark(request);
  return benchmark;
}

void SaveBenchmarkToDatabase(const std::string& name, const ompl::tools::Benchmark& benchmark) {
  std::string filename = GetDataFolder() + "logs/"+name+".log";
  benchmark.saveResultsToFile(filename.c_str());

  std::string db_filename = GetDataFolder() + "logs/"+name+".db";

  auto cmd_log_to_db = "python3 " + GetMainFolder() + "scripts/ompl_benchmark_statistics.py "+filename+" -d "+db_filename;
  system(cmd_log_to_db.c_str());
}
