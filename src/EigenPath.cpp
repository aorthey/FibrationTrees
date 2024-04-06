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
  time_at_configs_.clear();

  for(size_t k = 0; k < states.size(); k++) {
    StateXd config = robot->StateToEigen(states.at(k));

    if(k > 0) {
      auto d = robot->GetSpaceInformation()->distance(states.at(k), states.at(k-1));
      if(d > M_PI) {
        OMPL_ERROR("Configs are far apart.");
        std::cout << "Last     config: " << configs_.back() << std::endl;
        std::cout << "Current  config: " << config << std::endl;
        //continue;
      }
    }

    float time = robot->StateToTime(states.at(k));
    if(time < 0) {
      time_at_configs_.push_back((float)k/((float)states.size()-1.0f));
    } else {
      if(time_at_configs_.empty() && time > 0.0) {
        std::cout << "Error: First time must be zero, but is " << time << std::endl;
        throw "InvalidTime";
      }
      time_at_configs_.push_back(time);
    }
    configs_.push_back(config);
  }
  if(!time_at_configs_.empty()) {
    std::cout << "Created path with " << states.size() << " states and time in [" << time_at_configs_.front() <<
       ", " << time_at_configs_.back() << "]" << std::endl;
  }
  InitLengthFromConfigs(configs_);

  if(configs_.size() != time_at_configs_.size()) {
    std::cout << "created " << configs_.size() << " states, but " << time_at_configs_.size() << " timings." << std::endl;
  }
  assert(configs_.size() == time_at_configs_.size());
}

EigenPath::EigenPath(const std::vector<StateXd>& configs) : configs_(configs) {
  for(size_t k = 0; k < configs.size(); k++) {
    time_at_configs_.push_back((float)k/((float)configs.size()-1.0f));
  }
  InitLengthFromConfigs(configs_);
}

void EigenPath::InitLengthFromConfigs(const std::vector<StateXd>& configs) {
  if(configs.size() < 1) {
    throw std::length_error("Cannot initialize an empty path.");
  }

  std::vector<float> lengths;
  for(size_t k = 1; k < configs.size(); k++) {
    auto v1 = configs.at(k-1).configuration;
    auto v2 = configs.at(k).configuration;
    auto d = (v2 - v1).norm();
    lengths.push_back(d);
  }
  total_length_ = std::accumulate(lengths.begin(), lengths.end(), 0.0f);
}

StateXd EigenPath::GetConfigAt(float s) {
  const auto max_time = time_at_configs_.back();
  if(time_at_configs_.empty()) {
    std::cout << "Error: Path contains no configs." << std::endl;
    throw "InvalidPath";
  }

  if(s <= 0.0f) {
    return configs_.front();
  }
  if(s >= 1.0f) {
    return configs_.back();
  }

  const float position = s * max_time;
  float current_position = 0.0f;
  size_t index = 1;

  while(index < time_at_configs_.size()) {
    current_position = time_at_configs_.at(index);
    if(position < current_position) {
      auto v1 = configs_.at(index - 1);
      auto v2 = configs_.at(index);

      auto t1 = time_at_configs_.at(index - 1);
      auto t2 = current_position;
      //----Ti-1------------x-------------Ti-----------
      //      |             |              |
      //      |             |              |
      //      |             |              |
      //      t1         position          t2
      //      |_____________|
      //            dp
      //      |____________________________|
      //                   dt
      //-----------------------------------------
      auto dp = position - t1;
      auto dt = t2 - t1;

      auto s = dp / dt;

      auto state =  v1 + s*(v2 - v1);
      state.time = position;
      return state;
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
