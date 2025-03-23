#pragma once

#include <ompl/util/ClassForward.h>
#include <ompl/geometric/PathGeometric.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <fstream>

#include "robots/Robot.hpp"
#include "State.hpp"

OMPL_CLASS_FORWARD(EigenPath);

const std::string kDefaultFileName = "eigen_path.txt";
class EigenPath {
  public:

    explicit EigenPath();
    explicit EigenPath(const std::vector<StateXd>& configs);
    explicit EigenPath(const RobotPtr& robot, const ompl::base::PathPtr& path);

    /*\brief Get configuration along path at position s in [0,1] */
    StateXd GetConfigAt(const double position) const;

    double GetLength() const;

    double GetStartTime() const;
    double GetEndTime() const;

    void InitLengthFromConfigs(const std::vector<StateXd>& configs);

    void Save(const std::string& filename = kDefaultFileName);
    void Load(const std::string& filename = kDefaultFileName);
    static EigenPath FromFile(const std::string& filename = kDefaultFileName);

  private:
    std::vector<StateXd> configs_;
    std::vector<double> time_at_configs_;
    double total_length_{0.0f};

    double start_time_{0.0};
    double end_time_{1.0};
};
