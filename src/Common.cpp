#include "Common.hpp"

double LineDistance(const State3d& a, const State3d& b, const State3d& p) {
  const auto& n = (b-a).normalized();
  auto s = (p-a).dot(n);
  auto d = ((p-a)-s*n).norm();
  return d;
}
