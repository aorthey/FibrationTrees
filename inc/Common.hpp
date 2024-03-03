#ifndef COMMON_CONSTANTS_HPP
#define COMMON_CONSTANTS_HPP

#include <numeric>
#include "State.hpp"

const auto Epsilon = std::numeric_limits<double>::epsilon();
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();

double LineDistance(const State3d& a, const State3d& b, const State3d& p);

#define ValueOrReturn(name, function, return_value) \
  auto maybe_##name = function; \
  if(!maybe_##name.has_value()) { \
    std::cout << "Could not find value" << std::endl; \
    return return_value; \
  } \
  auto name = maybe_##name.value(); \

#define ValueOrReturnInt(name, function) ValueOrReturn(name, function, 1)

#define ReturnOnFalse(function, return_value) \
  if(!function) { \
    return return_value; \
  } 

#define ReturnIntOnFalse(function) ReturnOnFalse(function, 1) \

#endif
