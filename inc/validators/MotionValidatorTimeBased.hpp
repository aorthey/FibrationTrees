#pragma once

#include <ompl/base/State.h>
#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/multilevel/datastructures/FactoredSpaceInformation.h>
#include <ompl/multilevel/datastructures/TaskSpaceMotionValidator.h>

#include "validators/MotionValidatorTaskSpaceTranslation.hpp"
#include "robots/Robot.hpp"

class MotionValidatorTimeBased : public MotionValidatorTaskSpaceTranslation
{
  public:
    MotionValidatorTimeBased() = delete;
    explicit MotionValidatorTimeBased(const ompl::base::SpaceInformationPtr &si, const RobotPtr& robot, double vMax);
    virtual ~MotionValidatorTimeBased() override;

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;
    std::vector<ompl::base::State*> propagateMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
  protected:
    double vMax_{0.0};
};
