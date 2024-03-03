#include "EigenPath.hpp"

#include <iostream>
#include <numeric>

#include "OmplHelper.hpp"
#include "Common.hpp"

EigenPath::EigenPath() {
}

EigenPath::EigenPath(const RobotPtr& robot, const ompl::base::PathPtr& path) {
  const auto& si = path->getSpaceInformation();
  ompl::geometric::PathGeometric &pgeo = *static_cast<ompl::geometric::PathGeometric *>(path.get());
  auto states = pgeo.getStates();
  configs_.clear();

  std::cout << "Create EigenPath from " << states.size() << " states." << std::endl;
  for(size_t k = 0; k < states.size(); k++) {
    StateXd config = robot->StateToEigen(states.at(k));
    if(k > 0) {
      auto d = robot->GetSpaceInformation()->distance(states.at(k), states.at(k-1));
      if(d > M_PI) {
        OMPL_ERROR("Configs are far apart.");
        std::cout << "Last     config: " << configs_.back() << std::endl;
        std::cout << "Current  config: " << config << std::endl;
        continue;
      }
    }
    configs_.push_back(config);
  }
  InitLengthFromConfigs(configs_);
}

EigenPath::EigenPath(const std::vector<StateXd>& configs) : configs_(configs) {
  InitLengthFromConfigs(configs_);
}

void EigenPath::InitLengthFromConfigs(const std::vector<StateXd>& configs) {
  if(configs.size() < 1) {
    throw std::length_error("Cannot initialize an empty path.");
  }
  for(size_t k = 1; k < configs.size(); k++) {
    auto v1 = configs.at(k-1).configuration;
    auto v2 = configs.at(k).configuration;
    auto d = (v2 - v1).norm();
    lengths_.push_back(d);
  }
  total_length_ = std::accumulate(lengths_.begin(), lengths_.end(), 0.0f);
}

StateXd EigenPath::GetConfigAt(float s) {
  if(configs_.empty()) {
    std::cout << "Error: Path contains no configs." << std::endl;
    throw "InvalidPath";
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

void EigenPath::Save(const std::string& filename) {
  // std::ofstream out(filename);
  // boost::archive::text_oarchive oa(out);
  // oa << *this;
  OMPL_ERROR("NYI");
}

void EigenPath::Load(const std::string& filename) {
  // std::ifstream input(filename);
  // boost::archive::text_iarchive ia(input);
  // ia >> *this;
  OMPL_ERROR("NYI");
}
EigenPath EigenPath::FromFile(const std::string& filename) {
  EigenPath path;
  // {
  //   std::ifstream input(filename);
  //   boost::archive::text_iarchive ia(input);
  //   ia >> path;
  // }
  OMPL_ERROR("NYI");
  return path;
}
