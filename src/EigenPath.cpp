#include "EigenPath.hpp"
#include <iostream>
#include <numeric>

EigenPath::EigenPath(const std::vector<Eigen::VectorXd>& configs) : configs_(configs) {
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
