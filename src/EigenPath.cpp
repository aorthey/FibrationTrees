#include "EigenPath.hpp"

#include <iostream>
#include <numeric>

#include "OmplHelper.hpp"

EigenPath::EigenPath(const ompl::base::PathPtr& path) {
  const auto& si = path->getSpaceInformation();
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  auto states = pgeo.getStates();
  configs_.clear();
  for(size_t k = 0; k < states.size(); k++) {
    Eigen::VectorXd config = StateToEigenVectorXd(si, states.at(k));
    configs_.push_back(config);
  }
  InitLengthFromConfigs(configs_);
}

EigenPath::EigenPath(const std::vector<Eigen::VectorXd>& configs) : configs_(configs) {
  InitLengthFromConfigs(configs_);
}

void EigenPath::InitLengthFromConfigs(const std::vector<Eigen::VectorXd>& configs) {
  for(size_t k = 1; k < configs.size(); k++) {
    auto v1 = configs.at(k-1);
    auto v2 = configs.at(k);
    auto d = (v2 - v1).norm();
    lengths_.push_back(d);
  }
  total_length_ = std::accumulate(lengths_.begin(), lengths_.end(), 0.0f);
}

Eigen::VectorXd EigenPath::GetConfigAt(float s) {
  if(configs_.empty()) {
    std::cout << "Error: Path contains no configs." << std::endl;
    return Eigen::VectorXd::Zero(0);
  }
  if(s <= 0.0f) {
    return configs_.front();
  }
  if(s >= 1.0f) {
    return configs_.back();
  }
  const float position = s * total_length_;
  float current_position = 0.0f;
  size_t index = 0;

  while(index < lengths_.size()) {
    current_position+= lengths_.at(index);
    if(position < current_position) {
      
      auto v1 = configs_.at(index);
      auto v2 = configs_.at(index+1);
      auto s = (position - (current_position - lengths_.at(index))) / lengths_.at(index);
      return v1 + s*(v2 - v1);
    }
    index++;
  }
  return configs_.back();
}

float EigenPath::GetLength() const {
  return total_length_;
}
