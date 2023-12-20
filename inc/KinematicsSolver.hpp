#pragma once

#include <dart/dart.hpp>
#include <ompl/util/ClassForward.h>
#include <optional>

OMPL_CLASS_FORWARD(KinematicsSolver);

class KinematicsSolver {
  public:
    KinematicsSolver(const dart::dynamics::SkeletonPtr& skeleton);

    std::optional<Eigen::VectorXd> solve_ik(const Eigen::Vector3d& frame, size_t max_iterations = 10);
    std::optional<Eigen::Vector3d> solve_fk(const Eigen::VectorXd& config);

    std::vector<Eigen::VectorXd> solve_edge_ik(const Eigen::VectorXd& start_config, const Eigen::Vector3d& goal_tcp);
    std::vector<Eigen::VectorXd> solve_edge_ik_with_config(const Eigen::VectorXd& start_config, const Eigen::VectorXd& goal_config);

    bool lastSolveWasSuccessful();
    Eigen::VectorXd ForwardStep(const Eigen::VectorXd& config, const Eigen::VectorXd& dx) const;

  private:
    dart::dynamics::SkeletonPtr skeleton_;
    std::string endeffector_;
    bool success_;
};
