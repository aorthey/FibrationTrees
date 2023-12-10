#pragma once

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "TaskSpaceProjection.hpp"
#include "OmplHelper.hpp"

OMPL_CLASS_FORWARD(TaskSpace);

class TaskSpace : public ompl::base::RealVectorStateSpace {
 public:

  explicit TaskSpace(unsigned int dim, const KinematicsSolverPtr& kinematics_solver);

  ~TaskSpace();

  void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

 private:
  KinematicsSolverPtr kinematics_solver_;
};

// class TaskSpaceMultiRobot : public ompl::base::RealVectorStateSpace {
//  public:

//   TaskSpaceMultiRobot() = delete;

//   TaskSpaceMultiRobot(unsigned int dim, const std::vector<TaskSpacePtr>& task_spaces, 
//     const std::vector<dart::dynamics::SkeletonPtr>& manipulators);

//   ~TaskSpaceMultiRobot();

//   void interpolate(const ompl::base::State *from, const ompl::base::State *to, double t, ompl::base::State *state) const override;

//  private:
//   std::vector<TaskSpacePtr> task_spaces_;
//   std::vector<ompl::base::State*> component_from_;
//   std::vector<ompl::base::State*> component_to_;
//   std::vector<ompl::base::State*> component_state_;
// };
