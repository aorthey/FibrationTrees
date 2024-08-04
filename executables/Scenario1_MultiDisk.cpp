#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/planners/FibrationRRT.h>
#include <ompl/multilevel/planners/RRTtask.h>

#include <iostream>
#include <fstream>
#include <boost/math/constants/constants.hpp>
#include <boost/format.hpp>

#include "robots/MultiRobotDiskEnvironment.hpp"
#include "RunBenchmark.hpp"

//Problem: All robots start as disks with certain radius. Start position is
//equally distributed around the unit circle in R2 workspace.
//Goal position is to reach antipodal points for each robot.

const size_t kNumberOfDiskRobots = 5;
const float kRadiusDiskRobots = 0.15;
const size_t kMaximumIterations = 10000;
// const size_t kMaximumIterations = 3;
const size_t kDefaultSeed = 23;

int main()
{
  MultiRobotDiskEnvironment env(kNumberOfDiskRobots, kRadiusDiskRobots);

  //auto factor = env.constructDecompositionFactorTree();
  auto factor = env.constructPrioritizedFactorTree();

  auto start = env.CreateStartStates(factor);
  auto goal = env.CreateGoalStates(factor);

  ProblemDefinitionPtr pdef = std::make_shared<ProblemDefinition>(factor);
  pdef->setStartAndGoalStates(start, goal);

  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setSeed(kDefaultSeed);

  //////////////////////////////////////////////////////////////////////////////////////
  //////////Benchmark
  //////////////////////////////////////////////////////////////////////////////////////
  auto planner2 = std::make_shared<ompl::multilevel::RRTtask>(factor);
  planner2->setup();

  float timeout = 10.0;
  size_t run_count = 10;
  auto name = "Scenario1";
  ompl::base::ScopedState<> scoped_start_state(factor);
  scoped_start_state = pdef->getStartState(0);
  auto benchmark = RunBenchmark(name, factor, scoped_start_state, pdef->getGoal(), timeout, run_count, {planner, planner2});
  SaveBenchmarkToDatabase(name, benchmark);
  return 0;

  //////////////////////////////////////////////////////////////////////////////////////
  //////////Planning
  //////////////////////////////////////////////////////////////////////////////////////
  ompl::base::IterationTerminationCondition itc(kMaximumIterations);
  auto ptc = ompl::base::plannerOrTerminationCondition(itc, exactSolnPlannerTerminationCondition(pdef));

  PlannerStatus solved = planner->solve(ptc);

  if(solved == ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION) {
    auto path = pdef->getSolutionPath();
    path->print(std::cout);

    auto geom_path = path->as<ompl::geometric::PathGeometric>();
    auto path_simplifier = std::make_shared<ompl::geometric::PathSimplifier>(factor);
    path_simplifier->simplifyMax(*geom_path);
    path_simplifier->smoothBSpline(*geom_path);

    geom_path->print(std::cout);

    auto filename = "multi_robot_disk_path.dat";
    std::ofstream pathfile(boost::str(boost::format(filename)).c_str());
    geom_path->printAsMatrix(pathfile);
    OMPL_INFORM("Wrote solution to file %s", filename);
    OMPL_INFORM("Generate video using `python3 scripts/Scenario1_plot_disks.py %s`", filename);
  } else {
    auto path = pdef->getSolutionPath();
    if(path) {
      path->print(std::cout);
      OMPL_INFORM("Found approximate path");
    }
  }
  return 0;
}


