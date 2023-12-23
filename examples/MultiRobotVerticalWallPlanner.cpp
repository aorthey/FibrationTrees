#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "TaskSpace.hpp"
#include "TaskSpaceGoal.hpp"
#include "TaskSpaceProjection.hpp"
#include "TaskSpaceMotionValidator.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "gui/Visualizer.hpp"
#include "OmplHelper.hpp"
#include "MakeSpaceInformation.hpp"
#include "KinematicsSolver.hpp"
#include "Utils.hpp"
#include "robots/KukaSkeleton.hpp"

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/goals/FactoredGoal.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/projections/RN_RM.h>
#include <ompl/multilevel/datastructures/projections/SubspaceProjection.h>
#include <ompl/multilevel/planners/factor/FibrationRRT.h>

const float kRobotRobotDistance = 0.3;

int main(int argc, char* argv[]) {
  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr manipulator1 = createKukaSkeleton();
  dart::dynamics::SkeletonPtr manipulator2 = createKukaSkeleton();

  Eigen::Isometry3d transform1(Eigen::Isometry3d::Identity());
  transform1.translation() = Eigen::Vector3d{0.0, -0.5*kRobotRobotDistance, 0};
  manipulator1->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(transform1);

  Eigen::Isometry3d transform2(Eigen::Isometry3d::Identity());
  transform2.translation() = Eigen::Vector3d{0.0, +0.5*kRobotRobotDistance, 0};
  manipulator2->getRootBodyNode()->getParentJoint()->setTransformFromParentBodyNode(transform2);

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr floor = createFloor();
  dart::dynamics::SkeletonPtr wall = createBox(Eigen::Vector3d(+0.5, +0.0, 0.75), 0.16, 2.0, 1.5); //0.16
  dart::dynamics::SkeletonPtr point1 = createSphere(Eigen::Vector3d(-0.5, -0.5, -1), 0.01);
  dart::dynamics::SkeletonPtr point2 = createSphere(Eigen::Vector3d(-0.5, -0.5, -2), 0.01);

  dart::simulation::WorldPtr world(new dart::simulation::World);
  world->addSkeleton(manipulator1);
  world->addSkeleton(manipulator2);
  world->addSkeleton(floor);
  world->addSkeleton(wall);
  world->addSkeleton(point1);
  world->addSkeleton(point2);
  world->setGravity(Eigen::Vector3d::Zero());

  ////////////////////////////////////////////////////////////////////////////////
  ////Collision checking
  ////////////////////////////////////////////////////////////////////////////////
  KinematicsSolverPtr kinematics_solver1 = std::make_shared<KinematicsSolver>(manipulator1);
  KinematicsSolverPtr kinematics_solver2 = std::make_shared<KinematicsSolver>(manipulator2);

  std::vector<dart::dynamics::SkeletonPtr> collision_environment = {floor, wall};
  std::vector<dart::dynamics::SkeletonPtr> collision_robot1 = {manipulator1};
  std::vector<dart::dynamics::SkeletonPtr> collision_robot2 = {manipulator2};
  std::vector<dart::dynamics::SkeletonPtr> collision_point1 = {point1};
  std::vector<dart::dynamics::SkeletonPtr> collision_point2 = {point2};

  CollisionCheckerPtr collision_checker1 = std::make_shared<CollisionChecker>(world, collision_robot1, collision_environment);
  CollisionCheckerPtr collision_checker2 = std::make_shared<CollisionChecker>(world, collision_robot2, collision_environment);
  CollisionCheckerPtr collision_checker_robot_robot = std::make_shared<CollisionChecker>(world, collision_robot1, collision_robot2);

  CollisionCheckerPtr collision_checker_multi_robot = std::make_shared<MultiCollisionChecker>(world, 
      std::vector<CollisionCheckerPtr>({collision_checker_robot_robot, collision_checker1, collision_checker2}));

  CollisionCheckerPtr collision_checker_point1 = std::make_shared<CollisionChecker>(world, collision_point1, collision_environment);
  CollisionCheckerPtr collision_checker_point2 = std::make_shared<CollisionChecker>(world, collision_point2, collision_environment);

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

  auto factor1 = MakeTaskSpaceInformation(manipulator1, world, kinematics_solver1, collision_checker1);
  auto factor2 = MakeTaskSpaceInformation(manipulator2, world, kinematics_solver2, collision_checker2);

  const auto limits = std::make_pair(Eigen::Vector3d(0.39, -0.4, 0), Eigen::Vector3d(0.43, +0.4, 2));
  auto child1 = Make3DPointSpaceInformation(point1, world, collision_checker_point1, limits);
  auto child2 = Make3DPointSpaceInformation(point2, world, collision_checker_point2, limits);

  ompl::multilevel::ProjectionPtr projection_child1 = std::make_shared<ProjectionJointSpaceToR3>(factor1->getStateSpace(), child1->getStateSpace(), kinematics_solver1);
  factor1->addChild(child1, projection_child1);

  ompl::multilevel::ProjectionPtr projection_child2 = std::make_shared<ProjectionJointSpaceToR3>(factor2->getStateSpace(), child2->getStateSpace(), kinematics_solver2);
  factor2->addChild(child2, projection_child2);

  const std::vector<ompl::base::StateSpacePtr> task_spaces = {factor1->getStateSpace(), factor2->getStateSpace()};
  auto factor = MakeMultiRobotSpaceInformation(task_spaces);

  auto projection1 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor->getStateSpace(), factor1->getStateSpace(), 0);
  auto projection2 = std::make_shared<ompl::multilevel::Projection_Subspace>(factor->getStateSpace(), factor2->getStateSpace(), 1);

  bool computer_fiber_space = false;
  ReturnOnFalse(factor->addChild(factor1, projection1, computer_fiber_space), 1);
  ReturnOnFalse(factor->addChild(factor2, projection2, computer_fiber_space), 1);

  std::unordered_map<std::string, dart::dynamics::SkeletonPtr> manipulators;
  manipulators[factor1->getName()] = manipulator1;
  manipulators[factor2->getName()] = manipulator2;

  factor->setStateValidityChecker(std::make_shared<DartMultiRobotCollisionChecker>(factor, world, manipulators, collision_checker_multi_robot));
  // factor->setStateValidityCheckingResolution(0.001);

  auto motion_validator = std::make_shared<TaskSpaceMultiRobotMotionValidator>(factor);
  factor->setMotionValidator(motion_validator);

  //////////////////////////////////////////////////////////////////////////////////
  //////Create start/goal states and propagate them upwards (lift through the
  //complete fibration trees hierarchy)
  //////////////////////////////////////////////////////////////////////////////////
  auto task_start1_eigen = Eigen::Vector3d(0.4, -0.2, 1.0);
  auto task_goal1_eigen = Eigen::Vector3d(0.4, +0.1, 0.7); //was -0.3
  auto task_start2_eigen = Eigen::Vector3d(0.4, +0.0, 1.0);
  auto task_goal2_eigen = Eigen::Vector3d(0.4, -0.2, 0.6); //was +0.5

  auto task_start1 = child1->allocState();
  auto task_start2 = child2->allocState();
  auto task_goal1 = child1->allocState();
  auto task_goal2 = child2->allocState();

  EigenVector3dToState(task_start1_eigen, task_start1);
  EigenVector3dToState(task_start2_eigen, task_start2);
  EigenVector3dToState(task_goal1_eigen, task_goal1);
  EigenVector3dToState(task_goal2_eigen, task_goal2);

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

  world->addSimpleFrame(createSphereFrame(task_start1_eigen, 0.02, color_red));
  world->addSimpleFrame(createSphereFrame(task_goal1_eigen, 0.02, color_red_light));
  world->addSimpleFrame(createSphereFrame(task_start2_eigen, 0.02, color_green));
  world->addSimpleFrame(createSphereFrame(task_goal2_eigen, 0.02, color_green_light));

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
  goal_region2->setThreshold(0.2);

  std::unordered_map<std::string, ompl::base::GoalSampleableRegionPtr> goal_regions;
  goal_regions[factor1->getName()] = goal_region1;
  goal_regions[factor2->getName()] = goal_region2;

  auto goal_region = std::make_shared<ompl::base::FactoredGoal>(factor, goal_regions);
  ompl::base::ProblemDefinitionPtr pdef = std::make_shared<ompl::base::ProblemDefinition>(factor);
  pdef->addStartState(start);
  pdef->setGoal(goal_region);

  //////////////////////////////////////////////////////////////////////////////////
  //////Planning
  //////////////////////////////////////////////////////////////////////////////////
  auto planner = std::make_shared<ompl::multilevel::FibrationRRT>(factor);
  planner->setProblemDefinition(pdef);
  planner->setup();
  planner->setRange(Inf);

  float timeout = 100.0;
  ompl::base::PlannerStatus status = planner->Planner::solve(timeout);

  // if(pdef->hasApproximateSolution() ||
  //    pdef->hasExactSolution())
  // {
  //   auto simplifier = std::make_shared<ompl::geometric::PathSimplifier>(factor, pdef->getGoal());
  //   auto path = pdef->getSolutionPath();
  //   ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  //   simplifier->simplifyMax(pgeo);
  // }

  //////////////////////////////////////////////////////////////////////////////////
  //////Visualize
  //////////////////////////////////////////////////////////////////////////////////
  Visualizer visualizer(world);

  visualizer.SetCollisionChecker(collision_checker_multi_robot);

  visualizer.AddMultiRobotPlanner(manipulators, planner);

  visualizer.Run();
  return 0;
}
