#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/urdf/urdf.hpp>

#include "TaskSpaceProjection.hpp"
#include "TaskSpace.hpp"
#include "TaskSpaceGoal.hpp"
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
#include <ompl/multilevel/planners/factor/FibrationRRT.h>

const float kRobotRobotDistance = 0.4;

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
  dart::dynamics::SkeletonPtr wall = createBox(Eigen::Vector3d(+0.5, +0.0, 0.75), 0.16, 2.0, 1.5);
  dart::dynamics::SkeletonPtr point1 = createSphere(Eigen::Vector3d(-0.5, -0.5, -0.5), 0.01);
  dart::dynamics::SkeletonPtr point2 = createSphere(Eigen::Vector3d(-0.5, -0.5, -0.5), 0.01);

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
  std::vector<dart::dynamics::SkeletonPtr> collision_robots = {manipulator1, manipulator2};
  std::vector<dart::dynamics::SkeletonPtr> collision_robot1 = {manipulator1};
  std::vector<dart::dynamics::SkeletonPtr> collision_robot2 = {manipulator2};
  std::vector<dart::dynamics::SkeletonPtr> collision_point1 = {point1};
  std::vector<dart::dynamics::SkeletonPtr> collision_point2 = {point2};

  CollisionCheckerPtr collision_checker1 = std::make_shared<CollisionChecker>(world, collision_robot1, collision_environment);
  CollisionCheckerPtr collision_checker2 = std::make_shared<CollisionChecker>(world, collision_robot2, collision_environment);
  CollisionCheckerPtr collision_checker_all = std::make_shared<CollisionChecker>(world, collision_robots, collision_environment);
  CollisionCheckerPtr collision_checker_robot_robot = std::make_shared<CollisionChecker>(world, collision_robot1, collision_robot2);

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

  auto child1 = Make3DPointSpaceInformation(point1, world, collision_checker_point1);
  auto child2 = Make3DPointSpaceInformation(point2, world, collision_checker_point2);

  ompl::multilevel::ProjectionPtr projection_child1 = std::make_shared<ProjectionJointSpaceToR3>(factor1->getStateSpace(), child1->getStateSpace(), kinematics_solver1);
  factor1->addChild(child1, projection_child1);

  ompl::multilevel::ProjectionPtr projection_child2 = std::make_shared<ProjectionJointSpaceToR3>(factor2->getStateSpace(), child2->getStateSpace(), kinematics_solver2);
  factor2->addChild(child2, projection_child2);

  std::vector<dart::dynamics::SkeletonPtr> robots = {manipulator1, manipulator2};
  std::vector<KinematicsSolverPtr> kinematics_solvers = {kinematics_solver1, kinematics_solver2};

  auto factor = MakeMultiRobotSpaceInformation(world, robots, kinematics_solvers, collision_checker_robot_robot);

  //TODO: Make multi-robot collision checker to avoid robot-environment collisions
  //TODO: Add collision checker to avoid robot-robot collisions
  //TODO: Add motion validator to stay on wall in MultiSpace

  std::vector<size_t> indices1(manipulator1->getNumDofs());
  std::iota (std::begin(indices1), std::end(indices1), 0);
  std::vector<size_t> indices2(manipulator2->getNumDofs());
  std::iota (std::begin(indices2), std::end(indices2), manipulator1->getNumDofs());

  auto projection1 = std::make_shared<ompl::multilevel::Projection_RN_RM>(factor->getStateSpace(), factor1->getStateSpace(), indices1);
  auto projection2 = std::make_shared<ompl::multilevel::Projection_RN_RM>(factor->getStateSpace(), factor2->getStateSpace(), indices2);

  ReturnOnFalse(factor->addChild(factor1, projection1), 1);
  ReturnOnFalse(factor->addChild(factor2, projection2), 1);

  std::unordered_map<std::string, dart::dynamics::SkeletonPtr> manipulators;
  manipulators[factor1->getName()] = manipulator1;
  manipulators[factor2->getName()] = manipulator2;

  factor->setStateValidityChecker(std::make_shared<DartMultiRobotCollisionChecker>(factor, world, manipulators, collision_checker_robot_robot));
  // factor->setStateValidityCheckingResolution(0.001);
  // ompl::base::MotionValidatorPtr motion_validator = std::make_shared<TaskSpaceMotionValidator>(factor, kinematics_solver);
  // factor->setMotionValidator(motion_validator);

  auto task_start1 = child1->allocState();
  auto task_start2 = child2->allocState();
  auto task_goal1 = child1->allocState();
  auto task_goal2 = child2->allocState();

  EigenVector3dToState(Eigen::Vector3d(0.4, +0.3 + 0.5*kRobotRobotDistance, 0.9), task_start2);
  EigenVector3dToState(Eigen::Vector3d(0.4, -0.3 - 0.5*kRobotRobotDistance, 0.9), task_start1);
  EigenVector3dToState(Eigen::Vector3d(0.4, -0.1 + 0.5*kRobotRobotDistance, 0.5), task_goal1);
  EigenVector3dToState(Eigen::Vector3d(0.4, +0.1 - 0.5*kRobotRobotDistance, 0.5), task_goal2);

  std::unordered_map<std::string, ompl::base::State*> task_space_start_states;
  task_space_start_states[child1->getName()] = task_start1;
  task_space_start_states[child2->getName()] = task_start2;

  auto startX = ComputeValidTotalState(factor, task_space_start_states);
  if(startX.has_value()){
    factor->printState(startX.value());
  }
  std::unordered_map<std::string, ompl::base::State*> task_space_goal_states;
  task_space_goal_states[child1->getName()] = task_goal1;
  task_space_goal_states[child2->getName()] = task_goal2;

  auto goalX = ComputeValidTotalState(factor, task_space_goal_states);
  if(goalX.has_value()){
    factor->printState(goalX.value());
  }


  exit(0);

  //////////////////////////////////////////////////////////////////////////////////
  //////Create individual planning problem
  //////////////////////////////////////////////////////////////////////////////////

  ValueOrReturn(start2, ComputeValidIKState(factor2, projection_child2, Eigen::Vector3d(0.4, +0.3 + 0.5*kRobotRobotDistance, 0.9)), 1);
  auto start2_vector = ProjectStateToEigenVector3d(projection_child2, start2);
  std::cout << start2_vector << std::endl;
  world->addSimpleFrame(createSphereFrame(start2_vector, 0.02));

  ValueOrReturn(start1, ComputeValidIKState(factor1, projection_child1, Eigen::Vector3d(0.4, -0.1 - 0.5*kRobotRobotDistance, 0.9)), 1);
  auto start1_vector = ProjectStateToEigenVector3d(projection_child1, start1);
  std::cout << start1_vector << std::endl;
  world->addSimpleFrame(createSphereFrame(start1_vector, 0.02));

  ValueOrReturn(goal1, ComputeValidIKState(factor1, projection_child1, Eigen::Vector3d(0.4, 0.5 - 0.5*kRobotRobotDistance, 0.5)), 1);
  auto goal1_vector = ProjectStateToEigenVector3d(projection_child1, goal1);
  std::cout << goal1_vector << std::endl;
  world->addSimpleFrame(createSphereFrame(goal1_vector, 0.02));

  ValueOrReturn(goal2, ComputeValidIKState(factor2, projection_child2, Eigen::Vector3d(0.4, -0.3 + 0.5*kRobotRobotDistance, 0.5)), 1);
  auto goal2_vector = ProjectStateToEigenVector3d(projection_child2, goal2);
  std::cout << goal2_vector << std::endl;
  world->addSimpleFrame(createSphereFrame(goal2_vector, 0.02));

  //////////////////////////////////////////////////////////////////////////////////
  //////Lift start/goal states
  //////////////////////////////////////////////////////////////////////////////////
  std::unordered_map<std::string, ompl::base::State*> startStates;
  startStates[factor1->getName()] = start1;
  startStates[factor2->getName()] = start2;

  std::unordered_map<std::string, ompl::base::State*> goalStates;
  goalStates[factor1->getName()] = goal1;
  goalStates[factor2->getName()] = goal2;

  auto start = factor->allocState();
  auto goal = factor->allocState();

  factor->lift(startStates, start); //immersion mapping
  factor->lift(goalStates, goal);

  std::cout << "START" << std::endl;
  factor->printState(start);
  std::cout << "GOAL" << std::endl;
  factor->printState(goal);

  //////////////////////////////////////////////////////////////////////////////////
  //////Create factored task space goals
  //////////////////////////////////////////////////////////////////////////////////
  auto goal_region1 = std::make_shared<TaskSpaceGoal>(factor1, goal1, projection_child1);
  goal_region1->setThreshold(0.1);
  auto goal_region2 = std::make_shared<TaskSpaceGoal>(factor2, goal2, projection_child2);
  goal_region2->setThreshold(0.1);

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

  Visualizer visualizer(world);

  //////////////////////////////////////////////////////////////////////////////////
  //////Convert planner result to individual paths
  //////////////////////////////////////////////////////////////////////////////////
  if(pdef->hasApproximateSolution() ||
     pdef->hasExactSolution())
  {
    auto path = pdef->getSolutionPath();
    ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
    pgeo.interpolate(100);

    typedef std::unordered_map<std::string, ompl::base::State*> SplitConfig;

    std::vector<SplitConfig> configs;

    for(const auto& state : pgeo.getStates()) {
      path->getSpaceInformation()->printState(state);
      auto childStates = factor->allocChildStates();
      factor->project(state, childStates);
      configs.push_back(childStates);
    }

    std::vector<const ompl::base::State*> states1;
    std::vector<const ompl::base::State*> states2;
    for(const auto& config : configs) {
      auto it1 = config.find(factor1->getName());
      if(it1 != config.end()){
        states1.push_back(it1->second); 
      }
      auto it2 = config.find(factor2->getName());
      if(it2 != config.end()){
        states2.push_back(it2->second); 
      }
    }
    ompl::geometric::PathGeometricPtr path1 = std::make_shared<ompl::geometric::PathGeometric>(factor1, states1);
    visualizer.AddPath(manipulator1, path1);
    ompl::geometric::PathGeometricPtr path2 = std::make_shared<ompl::geometric::PathGeometric>(factor2, states2);
    visualizer.AddPath(manipulator2, path2);
  }

  //////////////////////////////////////////////////////////////////////////////////
  //////Visualize
  //////////////////////////////////////////////////////////////////////////////////
  // visualizer.AddPath(manipulator2, path2);
  visualizer.Run();
  return 0;
}
