#pragma once

#include <dart/dart.hpp>
#include <ompl/util/ClassForward.h>
#include <optional>

OMPL_CLASS_FORWARD(KinematicsSolver);

const bool kDebugInfo = false;
const float kStepSize = 0.1; //was 0.1. Best results for 0.01
const float kIntermediateStatesStepSize = 0.01; //was 0.01
const size_t kMaxNonIncreasingIterations = 10;
const float kOutOfBoundsJointLimitPadding = 0.0;//1e-2;
const size_t kDefaultMaxIKIterations = 10;

class KinematicsSolver {
  public:
    KinematicsSolver(const dart::dynamics::SkeletonPtr& skeleton);

    std::optional<Eigen::VectorXd> solve_ik(const Eigen::Vector3d& frame, size_t max_iterations = kDefaultMaxIKIterations);
    std::optional<Eigen::Vector3d> solve_fk(const Eigen::VectorXd& config);
    std::vector<Eigen::VectorXd> solve_edge_ik_with_config(const Eigen::VectorXd& start_config, const Eigen::VectorXd& goal_config);

    bool lastSolveWasSuccessful();
    Eigen::VectorXd ForwardStep(const Eigen::VectorXd& config, const Eigen::VectorXd& dx) const;

    Eigen::VectorXd ComputeJointLimitForce(const Eigen::VectorXd& config) const;
    bool AddConfig(const Eigen::VectorXd& config, std::vector<Eigen::VectorXd>& configs);

  private:
    dart::dynamics::SkeletonPtr skeleton_;
    std::string endeffector_;
    bool success_;
};
