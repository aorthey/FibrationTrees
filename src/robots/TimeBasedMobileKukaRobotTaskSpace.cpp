#include "robots/TimeBasedMobileKukaRobotTaskSpace.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SpaceTimeStateSpace.h>
#include "TranslationTaskSpaceMotionValidator.hpp"
#include "KinematicsSolver.hpp"
#include "TaskSpaceMobile.hpp"
#include "TaskSpace.hpp"
#include "Common.hpp"

ompl::multilevel::FactoredSpaceInformationPtr TimeBasedMobileKukaRobotTaskSpace::MakeSpaceInformation(const RobotPtr& robot) {
  ompl::base::StateSpacePtr space(new TaskSpaceMobile(robot));
  ompl::base::StateSpacePtr space_time(new ompl::base::SpaceTimeStateSpace(space));
  return std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space_time);
}

void TimeBasedMobileKukaRobotTaskSpace::SetSpaceInformationFromRobot(const RobotPtr& robot, 
    const dart::simulation::WorldPtr& world, const std::vector<dart::dynamics::SkeletonPtr>& obstacles) {
  ompl::base::StateSpacePtr space_time(new ompl::base::SpaceTimeStateSpace(robot->GetSpaceInformation()->getStateSpace()));
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space_time);
  SetSpaceInformation(factor);
  auto collision_checker = MakeCollisionChecker(factor, world, obstacles);
  SetCollisionChecker(collision_checker);
  // auto motion_validator = robot->GetMotionValidator();
  // factor->setMotionValidator(motion_validator);
}

// ompl::base::MotionValidatorPtr TimeBasedMobileKukaRobotTaskSpace::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
//   return std::make_shared<TranslationTaskSpaceMotionValidator>(factor, robot);
// }

StateXd TimeBasedMobileKukaRobotTaskSpace::StateToEigen(const ompl::base::State* state) const {
  auto N = GetDimension();
  Eigen::VectorXd v(N-1);

  const auto *state_position = state->as<ompl::base::CompoundState>()->as<ompl::base::State>(0)->as<ompl::base::CompoundState>();
  const auto *state_R2 = state_position->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const auto *state_SO2 = state_position->as<ompl::base::SO2StateSpace::StateType>(1);
  const auto *state_RN = state_position->as<ompl::base::RealVectorStateSpace::StateType>(2);

  v[0] = state_R2->values[0];
  v[1] = state_R2->values[1];
  v[2] = state_SO2->value;

  for (unsigned int k = 0; k < N-4; k++)
  {
      v[k + 3] = state_RN->values[k];
  }

  // const double time = std::static_pointer_cast<ompl::base::SpaceTimeStateSpace>(factor_->getStateSpace())->getStateTime(state);
  // v[N-1] = time;
  std::cout << "StateToEigen: " << v << std::endl;
  return MakeState(v);
}

void TimeBasedMobileKukaRobotTaskSpace::EigenToState(const StateXd& v, ompl::base::State* state) const {
  std::cout << "EigenToState" << std::endl;
  auto N = v.configuration.size();

  auto *state_position = state->as<ompl::base::CompoundState>()->as<ompl::base::State>(0)->as<ompl::base::CompoundState>();
  auto *state_R2 = state_position->as<ompl::base::RealVectorStateSpace::StateType>(0);
  auto *state_SO2 = state_position->as<ompl::base::SO2StateSpace::StateType>(1);
  auto *state_RN = state_position->as<ompl::base::RealVectorStateSpace::StateType>(2);
  // auto *state_R2 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  // auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1);
  // auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(2);

  state_R2->values[0] = v.configuration[0];
  state_R2->values[1] = v.configuration[1];
  state_SO2->value = v.configuration[2];

  for (unsigned int k = 0; k < N-4; k++)
  {
      state_RN->values[k] = v.configuration[k + 3];
  }

  state->as<ompl::base::CompoundState>()->as<ompl::base::TimeStateSpace::StateType>(1)->position = v[N-1];
  std::cout << "EigenToState" << std::endl;
}
