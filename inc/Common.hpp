#ifndef COMMON_CONSTANTS_HPP
#define COMMON_CONSTANTS_HPP

#include <numeric>
#include <Eigen/Dense>

const auto Epsilon = std::numeric_limits<double>::epsilon();
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

double LineDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p);


#endif
