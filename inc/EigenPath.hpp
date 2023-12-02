#pragma once

#include <Eigen/Dense>

#include <ompl/util/ClassForward.h>
#include <ompl/geometric/PathGeometric.h>

OMPL_CLASS_FORWARD(EigenPath);

class EigenPath {
  public:
    explicit EigenPath(const std::vector<Eigen::VectorXd>& configs);
    explicit EigenPath(const ompl::base::PathPtr& path);

    /*\brief Get configuration along path at position s in [0,1] */
    Eigen::VectorXd GetConfigAt(float s);

    float GetLength() const;

    void InitLengthFromConfigs(const std::vector<Eigen::VectorXd>& configs);

  private:
    std::vector<Eigen::VectorXd> configs_;
    std::vector<float> lengths_;
    float total_length_;
};
