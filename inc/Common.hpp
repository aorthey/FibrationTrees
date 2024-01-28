#ifndef COMMON_CONSTANTS_HPP
#define COMMON_CONSTANTS_HPP

#include <numeric>
#include <Eigen/Dense>

const auto Epsilon = std::numeric_limits<double>::epsilon();
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

double LineDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p);

#define ValueOrReturn(name, function, return_value) \
  auto maybe_##name = function; \
  if(!maybe_##name.has_value()) { \
    std::cout << "Could not find value" << std::endl; \
    return return_value; \
  } \
  auto name = maybe_##name.value(); \

#define ReturnOnFalse(function, return_value) \
  if(!function) { \
    return return_value; \
  } 

#endif
