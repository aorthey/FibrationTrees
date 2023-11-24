#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "TaskSpaceProjection.hpp"
#include "OmplHelper.hpp"

class TaskStateSpace : public ompl::base::RealVectorStateSpace {

 public:

  TaskStateSpace(unsigned int dim, const KinematicsSolverPtr& kinematics_solver);

  ~TaskStateSpace();

  void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

 private:
  KinematicsSolverPtr kinematics_solver_;
};

class TaskSpaceMotionValidator : public ompl::base::MotionValidator
{
public:
    TaskSpaceMotionValidator(ompl::base::SpaceInformation *si, const KinematicsSolverPtr& kinematics_solver) : ompl::base::MotionValidator(si), kinematics_solver_(kinematics_solver)
    {
      tmpState_ = si->allocState();
      lastValidState_ = si->allocState();
    }
    TaskSpaceMotionValidator(const ompl::base::SpaceInformationPtr &si, const KinematicsSolverPtr& kinematics_solver) : ompl::base::MotionValidator(si), kinematics_solver_(kinematics_solver)
    {
      tmpState_ = si->allocState();
      lastValidState_ = si->allocState();
    }
    ~TaskSpaceMotionValidator() override {
      si_->freeState(tmpState_);
      si_->freeState(lastValidState_);
    }

    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2) const override;
    bool checkMotion(const ompl::base::State *s1, const ompl::base::State *s2, std::pair<ompl::base::State *, double> &lastValid) const override;

private:
  KinematicsSolverPtr kinematics_solver_;
  ompl::base::State* tmpState_;
  ompl::base::State* lastValidState_;
};

