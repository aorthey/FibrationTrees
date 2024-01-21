#pragma once

#include <ompl/base/State.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>

#include "KinematicsSolver.hpp"

class TaskSpaceMultiRobotMotionValidator : public ompl::multilevel::TaskSpaceMotionValidator
{
public:
    TaskSpaceMultiRobotMotionValidator() = delete;
    explicit TaskSpaceMultiRobotMotionValidator(const ompl::multilevel::FactoredSpaceInformationPtr& si);
    ~TaskSpaceMultiRobotMotionValidator() override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

    std::vector<ompl::base::State*> propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
private:
    std::vector<ompl::base::MotionValidatorPtr> motion_validators_;
    std::vector<ompl::base::State*> lastValids_;
    ompl::base::State* tmpStateOnTotalSpace_;
};
