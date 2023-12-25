#pragma once

#include <ompl/base/State.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>

#include "KinematicsSolver.hpp"

class TranslationTaskSpaceMotionValidator : public ompl::multilevel::TaskSpaceMotionValidator
{
public:
    TranslationTaskSpaceMotionValidator() = delete;
    explicit TranslationTaskSpaceMotionValidator(ompl::base::SpaceInformation *si, const KinematicsSolverPtr& kinematics_solver);
    explicit TranslationTaskSpaceMotionValidator(const ompl::base::SpaceInformationPtr &si, const KinematicsSolverPtr& kinematics_solver);
    virtual ~TranslationTaskSpaceMotionValidator() override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

    std::vector<ompl::base::State*> propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;

    bool FillLastStateOnNoProgressAndReturn(const ompl::base::State *state, std::pair<ompl::base::State *, double> &lastValid) const;

private:
  KinematicsSolverPtr kinematics_solver_;
  ompl::base::State* tmpState_;
  ompl::base::State* lastValidState_;
};
