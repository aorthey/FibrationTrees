#include "ToString.hpp"

std::string ToString(const std::vector<std::string>& input) {
  std::stringstream ss;

  ss << "[";
  std::string delimiter = "";
  for(const auto& item : input) {
    ss << delimiter << item;
    delimiter = ", ";
  }
  ss << "]";
  return ss.str();
}

std::string ToString(const std::vector<RobotPtr>& input) {
  std::vector<std::string> names;
  for(const auto& item : input) {
    names.push_back(item->GetName());
  }
  return ToString(names);
}

std::string ToString(const Eigen::MatrixXd& mat) {
    std::stringstream ss;
    ss << "(";
    for (int i = 0; i < mat.size(); ++i) {
        ss << mat(i);
        if (i < mat.size() - 1) {
            ss << ", ";
        }
    }
    ss << ")";
    return ss.str();
}
