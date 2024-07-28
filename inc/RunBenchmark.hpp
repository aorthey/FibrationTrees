#pragma once
#include <string>
#include <optional>

#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Goal.h>
#include <ompl/tools/benchmark/Benchmark.h>

ompl::tools::Benchmark RunBenchmark(
  const std::string name,
  const ompl::multilevel::FactoredSpaceInformationPtr& factor,
  const ompl::base::ScopedState<>& start,
  const ompl::base::GoalPtr& goal,
  double timeout,
  size_t run_count);

ompl::tools::Benchmark RunBenchmark(
  const std::string name,
  const ompl::multilevel::FactoredSpaceInformationPtr& factor,
  const ompl::base::ScopedState<>& start,
  const ompl::base::GoalPtr& goal,
  double timeout,
  size_t run_count,
  const std::initializer_list<ompl::base::PlannerPtr>& planners,
  const std::optional<ompl::tools::Benchmark::PreSetupEvent> pre_setup_event = std::nullopt);

void SaveBenchmarkToDatabase(const std::string& name, const ompl::tools::Benchmark& benchmark);
