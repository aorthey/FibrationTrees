#pragma once

#include <Eigen/Dense>

#include <ompl/util/ClassForward.h>

OMPL_CLASS_FORWARD(EigenPath);

class EigenPath {
  public:
    explicit EigenPath(const std::vector<Eigen::VectorXd>& configs);

    /*\brief Get configuration along path at position s in [0,1] */
    Eigen::VectorXd GetConfigAt(float s);

    float GetLength() const;

  private:
    std::vector<Eigen::VectorXd> configs_;
    std::vector<float> lengths_;
    float total_length_;
};
