#include "State.hpp"

#include <iomanip>
#include "Common.hpp"

const Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

std::size_t StateXd::size() const {
  return configuration.size();
}

Eigen::VectorXd::Scalar& StateXd::operator[](std::size_t idx) { 
  return configuration[idx]; 
}

const Eigen::VectorXd::Scalar& StateXd::operator[](std::size_t idx) const { 
  return configuration[idx]; 
}

std::ostream & operator << (std::ostream& os, const StateXd& state) {
  os << "[";
  os << std::fixed << std::setprecision(2) << 0.0;
  os << "] ";
  os << state.configuration.format(CommaFmt);
  return os;
}

std::ostream & operator << (std::ostream& os, const State3d& state) {
  os << std::fixed << std::setprecision(2);
  os << state.format(CommaFmt);
  return os;
}

std::ostream & operator << (std::ostream& os, const Eigen::Vector3f& state) {
  os << std::fixed << std::setprecision(2);
  os << state.format(CommaFmt);
  return os;
}

StateXd MakeState(std::initializer_list<double> const &init_values) {
  std::vector<double> values{init_values};
  return MakeState(values);
}

StateXd MakeState(std::vector<double> values) {
  StateXd v;
  v.configuration = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(values.data(), values.size());
  return v;
}

StateXd MakeState(const Eigen::VectorXd& config) {
  StateXd v;
  v.configuration = config;
  return v;
}

StateXd MakeState(const size_t dimension, const double *values) {
  Eigen::VectorXd v(dimension);
  for(size_t k = 0; k < dimension; k++) {
    v[k] = values[k];
  }
  return MakeState(v);
}

StateXd MakeConstantState(const size_t dimension, const double value) {
  Eigen::VectorXd v(dimension);
  for(size_t k = 0; k < dimension; k++) {
    v[k] = value;
  }
  return MakeState(v);
}

State3d MakeState3d(std::initializer_list<double> const &init_values) {
  std::vector<double> values{init_values};
  return MakeState3d(values);
}

State3d MakeState3d(std::vector<double> values) {
  if(values.size() != 3) {
    throw std::out_of_range("Size must be three.");
  }
  return State3d(values.at(0), values.at(1), values.at(2));
}

State2d MakeState2d(std::initializer_list<double> const &init_values) {
  std::vector<double> values{init_values};
  return MakeState2d(values);
}

State2d MakeState2d(std::vector<double> values) {
  if(values.size() != 2) {
    throw std::out_of_range("Size must be three.");
  }
  return State2d(values.at(0), values.at(1));
}

StateXd CwiseMin(const StateXd& lhs, const StateXd& rhs) {
  return MakeState(lhs.configuration.cwiseMin(rhs.configuration));
}
StateXd CwiseMax(const StateXd& lhs, const StateXd& rhs) {
  return MakeState(lhs.configuration.cwiseMax(rhs.configuration));
}

namespace std {
  std::string to_string(const StateXd& state) {
    std::stringstream tmp;
    tmp << state;
    return tmp.str();
  }
  std::string to_string(const Eigen::VectorXd& configuration) {
    std::stringstream tmp;
    tmp << configuration;
    return tmp.str();
  }
};

double Distance(const StateXd& lhs, const StateXd& rhs) {
  return (lhs.configuration - rhs.configuration).norm();
}

double Distance(const State3d& lhs, const State3d& rhs) {
  return (lhs - rhs).norm();
}

double GetMinimumReachableTime(const StateXd& s1, const StateXd& s2, double vMax) {
  return Distance(s1, s2) / vMax;
}

bool IsReachableInTime(const StateXd& s1, const StateXd& s2, double vMax) {
  auto deltaTime = s2.time - s1.time;
  auto deltaSpace = Distance(s1, s2);
  if (deltaSpace / vMax > deltaTime + Epsilon) {
    //Cannot physically reach goal
    return false;
  }
  return true;
}

StateXd operator + (const StateXd& lhs, const TangentVector& rhs) {
  return MakeState(lhs.configuration + rhs);
}
StateXd operator + (const StateXd& lhs, const StateXd& rhs) {
  return MakeState(lhs.configuration + rhs.configuration);
}
StateXd operator - (const StateXd& lhs, const StateXd& rhs) {
  return MakeState(lhs.configuration - rhs.configuration);
}
StateXd operator * (const double& value, const StateXd& state) {
  return MakeState(value * state.configuration);
}
StateXd operator * (const StateXd& state, const double& value) {
  return value * state;
}

