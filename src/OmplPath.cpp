#include "OmplPath.hpp"

#include <iostream>
#include <numeric>
#include <ompl/base/State.h>
#include <ompl/geometric/PathGeometric.h>

#include "OmplHelper.hpp"

OmplPath::OmplPath(const ompl::base::PathPtr& path) : path_(path) {
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  auto states = pgeo.getStates();
  const auto& si = path_->getSpaceInformation();
  std::cout << "Path lengths: ";
  for(size_t k = 1; k < states.size(); k++) {
    auto s1 = states.at(k-1);
    auto s2 = states.at(k);
    auto d = si->distance(s1, s2);
    lengths_.push_back(d);
    std::cout << d << ", ";
  }
  std::cout << std::endl;
  total_length_ = std::accumulate(lengths_.begin(), lengths_.end(), 0.0f);
  tmpState_ = si->allocState();
}

OmplPath::~OmplPath() {
  const auto& si = path_->getSpaceInformation();
  si->freeState(tmpState_);
}

Eigen::VectorXd OmplPath::GetConfigAt(float s) {
  const auto& si = path_->getSpaceInformation();
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path_.get());
  auto states = pgeo.getStates();
  if(states.empty()) {
    std::cout << "Error: Path contains no configs." << std::endl;
    return Eigen::VectorXd::Zero(0);
  }
  if(s <= 0.0f) {
    return StateToEigenVectorXd(si, states.front());
  }
  if(s >= 1.0f) {
    return StateToEigenVectorXd(si, states.back());
  }
  const float position = s * total_length_;
  float current_position = 0.0f;
  size_t index = 0;

  while(index < lengths_.size()) {
    current_position+= lengths_.at(index);
    if(position < current_position) {
      const auto s1 = states.at(index);
      const auto s2 = states.at(index+1);
      const auto s = (position - (current_position - lengths_.at(index))) / lengths_.at(index);
      si->getStateSpace()->interpolate(s1, s2, s, tmpState_);
      return StateToEigenVectorXd(si, tmpState_);
    }
    index++;
  }
  return StateToEigenVectorXd(si, states.back());
}

float OmplPath::GetLength() const {
  return total_length_;
}
