#pragma once
#include <string>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/base/Planner.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/Goal.h>

void RunBenchmark(
  const std::string name,
  const ompl::multilevel::FactoredSpaceInformationPtr& factor,
  const ompl::base::ScopedState<>& start,
  const ompl::base::GoalPtr& goal,
  double timeout);

void RunBenchmark(
  const std::string name,
  const ompl::multilevel::FactoredSpaceInformationPtr& factor,
  const ompl::base::ScopedState<>& start,
  const ompl::base::GoalPtr& goal,
  double timeout,
  const std::initializer_list<ompl::base::PlannerPtr>& planners);
