#include "samplers/TaskSpaceMultiRobotSampler.hpp"

#include "KinematicsSolver.hpp"
#include "DartHelper.hpp"

TaskSpaceMultiRobotSampler::TaskSpaceMultiRobotSampler(const RobotPtr& joint_robot, const std::vector<RobotPtr>& robots, const std::vector<std::pair<State3d, State3d>>& limits)  
  : ompl::base::StateSampler(joint_robot->GetSpaceInformation()->getStateSpace().get()), joint_robot_(joint_robot), robots_(robots), limits_(limits) {

  for(const auto& robot : robots) {
    auto kinematics_solver = std::make_shared<KinematicsSolver>(robot->GetSkeleton());
    kinematics_solvers_.push_back(kinematics_solver);
  }
}

void TaskSpaceMultiRobotSampler::sampleUniform(ompl::base::State *state) {
  auto eigen_vectors = MakeConstantState(joint_robot_->GetDimension(), 0);
  auto current_dimension = 0;
  for(size_t k = 0; k < robots_.size(); k++) {
    auto limits = limits_.at(k);
    auto robot = robots_.at(k);
    auto Nrobot = robot->GetDimension();

    auto frame = dart::math::Random::uniform(limits.first, limits.second);
    auto maybe_ik_solution = kinematics_solvers_.at(k)->solve_ik(frame);
    if(!maybe_ik_solution.has_value()) {
      auto skeleton = robot->GetSkeleton();
      auto invalid_state = GetRandomPosition(skeleton);
      auto lb = skeleton->getPositionLowerLimits();
      for(size_t k = 0; k < (size_t) invalid_state.configuration.size(); k++) {
        eigen_vectors.configuration[current_dimension + k] = lb[k] - 1.0;
      }
    } else {
      auto q = maybe_ik_solution.value().configuration;
      for(size_t k = 0; k < (size_t) q.size(); k++) {
        eigen_vectors.configuration[current_dimension + k] = q[k];
      }
    }
    current_dimension+= Nrobot;
  }
  joint_robot_->EigenToState(eigen_vectors, state);
}

void TaskSpaceMultiRobotSampler::sampleUniformNear(ompl::base::State*, const ompl::base::State*, double) {
  throw std::runtime_error("Not yet implemented");
}
void TaskSpaceMultiRobotSampler::sampleGaussian(ompl::base::State*, const ompl::base::State*, double) {
  throw std::runtime_error("Not yet implemented");
}

ompl::base::StateSamplerPtr allocateTaskSpaceMultiRobotSampler(const RobotPtr& joint_robot, const std::vector<RobotPtr>& robots, const std::vector<std::pair<State3d, State3d>>& limits) {
  return std::make_shared<TaskSpaceMultiRobotSampler>(joint_robot, robots, limits);
}

