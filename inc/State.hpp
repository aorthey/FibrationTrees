#pragma once

#include <Eigen/Dense>
#include <iostream>

typedef Eigen::Vector3d State3d;
typedef Eigen::VectorXd TangentVector;

struct StateXd {
  Eigen::VectorXd configuration;
  float time{0.0f};

  Eigen::VectorXd::Scalar& operator[](std::size_t idx);
  const Eigen::VectorXd::Scalar& operator[](std::size_t idx) const;
  std::size_t size() const;
};

namespace std {
  std::string to_string(const StateXd& state);
}

////////////////////////////////////////////////////////////////////////////////
// std::vector to State
////////////////////////////////////////////////////////////////////////////////
StateXd MakeState(std::initializer_list<double> const &init_values);
StateXd MakeState(std::vector<double> values);
StateXd MakeState(const Eigen::VectorXd& config);
StateXd MakeState(const size_t dimension, const double *values);
StateXd MakeConstantState(const size_t dimension, const double value);

StateXd CwiseMin(const StateXd& lhs, const StateXd& rhs);
StateXd CwiseMax(const StateXd& lhs, const StateXd& rhs);

////////////////////////////////////////////////////////////////////////////////
// State operators
////////////////////////////////////////////////////////////////////////////////
std::ostream& operator << (std::ostream& os, const StateXd& state);

float Distance(const State3d& lhs, const State3d& rhs);
float Distance(const StateXd& lhs, const StateXd& rhs);
StateXd operator + (const StateXd& lhs, const TangentVector& rhs);
StateXd operator + (const StateXd& lhs, const StateXd& rhs);
StateXd operator - (const StateXd& lhs, const StateXd& rhs);
StateXd operator * (const float& value, const StateXd& state);
StateXd operator * (const StateXd& state, const float& value);
