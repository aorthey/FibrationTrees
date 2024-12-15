#include "TaskSpaceGoal.hpp"
#include "Common.hpp"
#include "CollisionChecker.hpp"
#include "DartHelper.hpp"
#include "OmplHelper.hpp"
#include "KinematicsSolver.hpp"
#include "gui/Visualizer.hpp"
#include "robots/RobotFactory.hpp"
#include "robots/KukaRobot.hpp"
#include "FilePath.hpp"

#include <dart/dart.hpp>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/terminationconditions/IterationTerminationCondition.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/planners/FibrationRRT.h>

int main(int argc, char* argv[]) {

  ////////////////////////////////////////////////////////////////////////////////
  ////Create world with obstacles
  ////////////////////////////////////////////////////////////////////////////////

  dart::simulation::WorldPtr world(new dart::simulation::World);

  std::vector<dart::dynamics::SkeletonPtr> obstacles;
  obstacles.push_back(createFromURDF(GetDataFolder() + "objects/maze.urdf", State3d(+0.55, +0.1, 0.85)));
  obstacles.push_back(createFloor());
  obstacles.push_back(createBox(State3d(+0.5, +0.0, 0.75), 1.05, 2.0, 1.5)); //0.15, 2.0, 1.5
  for(const auto& obstacle : obstacles) {
    world->addSkeleton(obstacle);
  }

  dart::math::Random::setSeed(0);

  ////////////////////////////////////////////////////////////////////////////////
  ////Creating manipulator
  ////////////////////////////////////////////////////////////////////////////////
  auto kuka_robot = MakeRobot<KukaRobot>(world, obstacles);

  ////////////////////////////////////////////////////////////////////////////////
  ////World creation
  ////////////////////////////////////////////////////////////////////////////////
  dart::dynamics::SkeletonPtr point = createSphere(0.01);
  world->addSkeleton(point);

  world->addSkeleton(kuka_robot->GetSkeleton());
  world->setGravity(State3d::Zero());

  Visualizer visualizer(world);
  visualizer.SetCollisionChecker(kuka_robot->GetCollisionChecker());
  visualizer.Run();
  return 0;
}
