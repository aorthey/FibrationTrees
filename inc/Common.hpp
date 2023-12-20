#ifndef COMMON_CONSTANTS_HPP
#define COMMON_CONSTANTS_HPP

const auto Epsilon = std::numeric_limits<double>::epsilon();
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");

#endif
