#include "robots/MobileKukaRobotTaskSpace.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include "TranslationTaskSpaceMotionValidator.hpp"
#include "KinematicsSolver.hpp"
#include "TaskSpaceMobile.hpp"
#include "TaskSpace.hpp"

ompl::multilevel::FactoredSpaceInformationPtr MobileKukaRobotTaskSpace::MakeSpaceInformation(const RobotPtr& robot) {
  ompl::base::StateSpacePtr space(new TaskSpaceMobile(robot));
  auto factor = std::make_shared<ompl::multilevel::FactoredSpaceInformation>(space);
  return factor;
}

ompl::base::MotionValidatorPtr MobileKukaRobotTaskSpace::MakeMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& factor, const RobotPtr& robot) {
  return std::make_shared<TranslationTaskSpaceMotionValidator>(factor, robot);
}

Eigen::VectorXd MobileKukaRobotTaskSpace::StateToEigen(const ompl::base::State* state) const {
  auto N = GetDimension();
  Eigen::VectorXd v(N);
  const auto *state_R2 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  const auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1);
  const auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(2);

  v[0] = state_R2->values[0];
  v[1] = state_R2->values[1];
  v[2] = state_SO2->value;

  for (unsigned int k = 0; k < N-3; k++)
  {
      v[k + 3] = state_RN->values[k];
  }
  return v;
}

void MobileKukaRobotTaskSpace::EigenToState(const Eigen::VectorXd& v, ompl::base::State* state) const {
  auto N = v.size();
  auto *state_R2 = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(0);
  auto *state_SO2 = state->as<ompl::base::CompoundState>()->as<ompl::base::SO2StateSpace::StateType>(1);
  auto *state_RN = state->as<ompl::base::CompoundState>()->as<ompl::base::RealVectorStateSpace::StateType>(2);

  state_R2->values[0] = v[0];
  state_R2->values[1] = v[1];
  state_SO2->value = v[2];

  for (unsigned int k = 0; k < N-3; k++)
  {
      state_RN->values[k] = v[k + 3];
  }
}
