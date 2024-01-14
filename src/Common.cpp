#include "Common.hpp"

double LineDistance(const Eigen::Vector3d& a, const Eigen::Vector3d& b, const Eigen::Vector3d& p) {
  const auto& n = (b-a).normalized();
  auto s = (p-a).dot(n);
  auto d = ((p-a)-s*n).norm();
  return d;
}
