#pragma once

#include <dart/dart.hpp>
#include <ompl/util/ClassForward.h>
#include <optional>

#include "State.hpp"

OMPL_CLASS_FORWARD(KinematicsSolver);

const bool kDebugInfo = false;
const double kStepSize = 0.05; //was 0.1. Best results for 0.01
const double kIntermediateStatesStepSize = 0.01; //was 0.01
const size_t kMaxNonIncreasingIterations = 10;
const double kOutOfBoundsJointLimitPadding = 0.0;//1e-2;
const size_t kDefaultMaxIKIterations = 10;

class KinematicsSolver {
  public:
    KinematicsSolver(const dart::dynamics::SkeletonPtr& skeleton);

    std::optional<StateXd> solve_ik(const State3d& frame, size_t max_iterations = kDefaultMaxIKIterations);
    std::optional<State3d> solve_fk(const StateXd& config);
    std::vector<StateXd> solve_edge_ik_with_config(const StateXd& start_config, const StateXd& goal_config);

    bool lastSolveWasSuccessful();
    StateXd ForwardStep(const StateXd& config, const TangentVector& dx) const;

    TangentVector ComputeJointLimitForce(const StateXd& config) const;
    bool AddConfig(const StateXd& config, std::vector<StateXd>& configs);

  private:
    dart::dynamics::SkeletonPtr skeleton_;
    std::string endeffector_;
    bool success_;
};
