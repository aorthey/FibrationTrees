#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "TaskSpaceProjection.hpp"
#include "OmplHelper.hpp"

class TaskSpace : public ompl::base::RealVectorStateSpace {

 public:

  TaskSpace(unsigned int dim, const KinematicsSolverPtr& kinematics_solver);

  ~TaskSpace();

  void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

 private:
  KinematicsSolverPtr kinematics_solver_;
};

class TaskSpaceMotionValidator : public ompl::base::MotionValidator
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

