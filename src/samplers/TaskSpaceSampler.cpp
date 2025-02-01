#include "samplers/TaskSpaceSampler.hpp"

#include "KinematicsSolver.hpp"
#include "DartHelper.hpp"
#include "Common.hpp"

TaskSpaceSampler::TaskSpaceSampler(const RobotPtr& robot, const std::pair<State3d, State3d>& limits)
  : ompl::base::StateSampler(robot->GetSpaceInformation()->getStateSpace().get()), robot_(robot), limits_(limits) {
  kinematics_solver_ = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
}

void TaskSpaceSampler::sampleUniform(ompl::base::State *state) {
  auto frame = dart::math::Random::uniform(limits_.first, limits_.second);
  auto maybe_ik_solution = kinematics_solver_->solve_ik(frame);
  if(!maybe_ik_solution.has_value()) {
    auto skeleton = robot_->GetSkeleton();
    auto invalid_state = MakeConstantState(skeleton->getNumDofs(), -Inf);
    robot_->EigenToState(invalid_state, state);
  } else {
    robot_->EigenToState(maybe_ik_solution.value(), state);
  }
}

void TaskSpaceSampler::sampleUniformNear(ompl::base::State*, const ompl::base::State*, double) {
  throw std::runtime_error("Not yet implemented");
}
void TaskSpaceSampler::sampleGaussian(ompl::base::State*, const ompl::base::State*, double) {
  throw std::runtime_error("Not yet implemented");
}

ompl::base::StateSamplerPtr allocateTaskSpaceSampler(const RobotPtr& robot, const std::pair<State3d, State3d>& limits) {
  return std::make_shared<TaskSpaceSampler>(robot, limits);
}

