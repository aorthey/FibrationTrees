#include "samplers/TaskSpaceSampler.hpp"

#include "KinematicsSolver.hpp"
#include "DartHelper.hpp"

TaskSpaceSampler::TaskSpaceSampler(const RobotPtr& robot, const std::pair<State3d, State3d>& limits)
  : ompl::base::StateSampler(robot->GetSpaceInformation()->getStateSpace().get()), robot_(robot), limits_(limits) {
  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
}

void TaskSpaceSampler::sampleUniform(ompl::base::State *state) {
  auto frame = dart::math::Random::uniform(limits_.first, limits_.second);
  auto maybe_ik_solution = kinematics_solver_->solve_ik(frame);
  if(!maybe_ik_solution.has_value()) {
    auto skeleton = robot_->GetSkeleton();
    auto invalid_state = GetRandomPosition(skeleton);
    auto lb = skeleton->getPositionLowerLimits();
    for(size_t k = 0; k < invalid_state.configuration.size(); k++) {
      invalid_state.configuration[k] = lb[k] - 1.0;
    }
    robot_->EigenToState(invalid_state, state);
  } else {
    robot_->EigenToState(maybe_ik_solution.value(), state);
  }
}

void TaskSpaceSampler::sampleUniformNear(ompl::base::State *state, const ompl::base::State *near, double distance) {
  throw "NYI";
}
void TaskSpaceSampler::sampleGaussian(ompl::base::State *state, const ompl::base::State *mean, double stdDev) {
  throw "NYI";
}

ompl::base::StateSamplerPtr allocateTaskSpaceSampler(const RobotPtr& robot, const std::pair<State3d, State3d>& limits) {
  return std::make_shared<TaskSpaceSampler>(robot, limits);
}

