#include <gtest/gtest.h>

#include "robots/TimeBasedSphereRobot.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/RobotFactory.hpp"
#include "DartHelper.hpp"
#include "Common.hpp"

#include <ompl/multilevel/datastructures/projections/TimeBasedProjection.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

class TimeBasedDiskRobot : public TimeBasedSphereRobot {
  public:
    TimeBasedDiskRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override {
      return createSphere(0.2);
    }
};
class DiskRobot : public SphereRobot {
  public:
    DiskRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override {
      return createSphere(0.2);
    }
};

TEST(TimeBasedPlanning, Simple) {
  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto robot_in_time = MakeRobot<TimeBasedDiskRobot>(world);
  auto robot = MakeRobot<DiskRobot>(world);

  const auto limits = std::make_pair(State3d(-1.0, -0.0, 0.0), State3d(1.0, 0.0, 0.0));
  robot->SetLimits(limits);
  robot_in_time->SetLimits(limits);

  auto factor1 = robot_in_time->GetSpaceInformation();
  auto factor2 = robot->GetSpaceInformation();

  auto projection_time_dimension = std::make_shared<ompl::multilevel::Projection_TimeBased>(factor1->getStateSpace(), factor2->getStateSpace());
  factor1->addChild(factor2, projection_time_dimension);

  factor1->printFactorization(std::cout);

   ////////////////////////////////////////////////////////////////////////////////
   ////Create problem structure
   ////////////////////////////////////////////////////////////////////////////////
  auto start = factor1->allocState();
  auto goal = factor1->allocState();

  robot_in_time->EigenToState(MakeState({-1.0,0.0,0}), start);
  robot_in_time->EigenToState(MakeState({+1.0,0.0,0}), goal);

  robot_in_time->TimeToState(0.0, start);
  robot_in_time->TimeToState(5.0, goal);

  factor1->printState(start);
  factor1->printState(goal);

  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor1);
  pdef->setStartAndGoalStates(start, goal, 0.1);

   ////////////////////////////////////////////////////////////////////////////////
   ////Planning
   ////////////////////////////////////////////////////////////////////////////////
  //auto planner = std::make_shared<ompl::geometric::RRTConnect>(factor1);
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor1);
  planner->setProblemDefinition(pdef);
  planner->setup();
  //planner->setRange(+Inf);

  float timeout = 10.0;
  timeout = 1.0;

  auto ptc = ompl::base::plannerOrTerminationCondition(
         ompl::base::exactSolnPlannerTerminationCondition(pdef),
         ompl::base::timedPlannerTerminationCondition(timeout)
     );

  std::cout << "Start planning" << std::endl;
  ompl::base::PlannerStatus status = planner->solve(ptc);
  EXPECT_TRUE(status);
}
