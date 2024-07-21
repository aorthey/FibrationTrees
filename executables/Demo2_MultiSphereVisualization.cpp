#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "gui/Visualizer.hpp"
#include "robots/SphereRobot.hpp"
#include "robots/MultiRobot.hpp"
#include "robots/RobotFactory.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RNSO2_RN.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>
#include <ompl/base/goals/FactoredGoal.h>

#include <ranges>

class BallRobot : public SphereRobot {
  public:
    BallRobot() = default;
    dart::dynamics::SkeletonPtr MakeSkeleton() override {
      return createSphere(0.25);
    }
};

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->setGravity(State3d::Zero());
  addCoordinateFrameToWorld(world);

  auto robot1 = MakeRobot<BallRobot>(world);
  const auto limits1 = std::make_pair(State3d(-1.0, 0.0, 0.0), State3d(1.0, 0.0, 0.0));
  robot1->SetLimits(limits1);
  auto space1 = robot1->GetSpaceInformation();

  auto robot2 = MakeRobot<BallRobot>(world);
  const auto limits2 = std::make_pair(State3d(0.0, -1.0, 0.0), State3d(0.0, 1.0, 0.0));
  robot2->SetLimits(limits2);
  auto space2 = robot2->GetSpaceInformation();

  auto robot3 = MakeRobot<BallRobot>(world);
  const auto limits3 = std::make_pair(State3d(0.0, 0.0, -1.0), State3d(0.0, 0.0, 1.0));
  robot3->SetLimits(limits3);
  auto space3 = robot3->GetSpaceInformation();

  ////////////////////////////////////////////////////////////////////////////////
  // Setup multi robot space
  ////////////////////////////////////////////////////////////////////////////////
  std::vector<RobotPtr> robots = {robot1, robot2, robot3};
  auto robot = MultiRobot::MakeMultiRobot(robots);
  auto factor = robot->GetSpaceInformation();
  auto projection_to_1 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor, space1, 0);
  auto projection_to_2 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor, space2, 1);
  auto projection_to_3 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor, space3, 2);

  bool computer_fiber_space = false;
  ReturnOnFalse(factor->addChild(space1, projection_to_1, computer_fiber_space), 1);
  ReturnOnFalse(factor->addChild(space2, projection_to_2, computer_fiber_space), 1);
  ReturnOnFalse(factor->addChild(space3, projection_to_3, computer_fiber_space), 1);

  auto collision_checker = std::make_shared<DartMultiRobotCollisionChecker>(factor, world, robots);
  factor->setStateValidityChecker(collision_checker);
  factor->printFactorization(std::cout);

  //////////////////////////////////////////////////////////////////////////////////
  //////Create problem definition
  //////////////////////////////////////////////////////////////////////////////////
  auto start_config = MakeState({-1,0,0,0,-1,0,0,0,-1});
  auto goal_config = MakeState({+1,0,0,0,+1,0,0,0,+1});
  auto start = factor->allocState();
  auto goal = factor->allocState();
  robot->EigenToState(start_config, start);
  robot->EigenToState(goal_config, goal);
  auto pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->setStartAndGoalStates(start, goal);
 
  //////////////////////////////////////////////////////////////////////////////////////
  //////////Planning
  //////////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(Inf);
  planner->setSmoothIntermediateSolutions(true);
  planner->setSmoothIntermediateSolutions(factor->getName(), false);
 
  float timeout = 10;

  auto ptc = TimeOrSolutionPtc(pdef, timeout);
  ompl::base::PlannerStatus status = planner->solve(ptc);
 
  ////////////////////////////////////////////////////////////////////////////////
  ////Visualize
  ////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);
  visualizer.SetCollisionChecker(collision_checker->GetCollisionChecker());
  visualizer.AddPlanner(robot, planner);

  visualizer.Run();

  return 0;
}

