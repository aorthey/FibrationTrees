#pragma once

#include <ompl/base/State.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>

#include "KinematicsSolver.hpp"

class TaskSpaceMotionValidator : public ompl::base::DiscreteMotionValidator
{
public:
    TaskSpaceMotionValidator() = delete;
    explicit TaskSpaceMotionValidator(ompl::base::SpaceInformation *si, const KinematicsSolverPtr& kinematics_solver);
    explicit TaskSpaceMotionValidator(const ompl::base::SpaceInformationPtr &si, const KinematicsSolverPtr& kinematics_solver);
    ~TaskSpaceMotionValidator() override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

    bool FillLastStateOnNoProgressAndReturn(const ompl::base::State *state, std::pair<ompl::base::State *, double> &lastValid) const;

private:
  KinematicsSolverPtr kinematics_solver_;
  ompl::base::State* tmpState_;
  ompl::base::State* lastValidState_;
};

class TaskSpaceMultiRobotMotionValidator : public ompl::base::DiscreteMotionValidator
{
public:
    TaskSpaceMultiRobotMotionValidator() = delete;
    explicit TaskSpaceMultiRobotMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& si);
    ~TaskSpaceMultiRobotMotionValidator() override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

private:
    std::vector<ompl::base::MotionValidatorPtr> motion_validators_;
    std::vector<ompl::base::State*> lastValids_;
    ompl::base::State* tmpStateOnTotalSpace_;
};
