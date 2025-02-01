#include <gtest/gtest.h>

#include "gui/PathReplayWorldNode.hpp"
#include "robots/MobileKukaRobot.hpp"
#include "robots/RobotFactory.hpp"

const auto kColor = MakeState3d({1, 0, 0});

TEST(PathReplayWorldNodeTest, AddZeroLengthPathTest) {
  dart::simulation::WorldPtr world(new dart::simulation::World);

  auto node = PathReplayWorldNode(world);

  auto robot = MakeRobot<MobileKukaRobot>();
  auto si = robot->GetSpaceInformation();
  auto sampler = si->allocStateSampler();

  auto stateA = si->allocState();
  auto stateB = si->allocState();

  sampler->sampleUniform(stateA);
  si->copyState(stateB, stateA);

  std::vector<const ompl::base::State*> path_states;
  path_states.push_back(stateA);
  path_states.push_back(stateB);
  auto path = std::make_shared<ompl::geometric::PathGeometric>(si, path_states);

  EXPECT_NO_THROW(node.AddPath(robot, path, kColor););

  si->freeState(stateA);
  si->freeState(stateB);
}

