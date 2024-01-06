#pragma once

#include <Eigen/Dense>

#include <ompl/util/ClassForward.h>
#include <ompl/geometric/PathGeometric.h>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <fstream>

#include "robots/Robot.hpp"

OMPL_CLASS_FORWARD(EigenPath);

const std::string kDefaultFileName = "eigen_path.txt";
namespace boost{
    namespace serialization{

        template<   class Archive,
                    class S,
                    int Rows_,
                    int Cols_,
                    int Ops_,
                    int MaxRows_,
                    int MaxCols_>
        inline void save(
            Archive & ar,
            const Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
            const unsigned int version)
            {
                int rows = g.rows();
                int cols = g.cols();

                ar & rows;
                ar & cols;
                ar & boost::serialization::make_array(g.data(), rows * cols);
            }

        template<   class Archive,
                    class S,
                    int Rows_,
                    int Cols_,
                    int Ops_,
                    int MaxRows_,
                    int MaxCols_>
        inline void load(
            Archive & ar,
            Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
            const unsigned int version)
        {
            int rows, cols;
            ar & rows;
            ar & cols;
            g.resize(rows, cols);
            ar & boost::serialization::make_array(g.data(), rows * cols);
        }

        template<   class Archive,
                    class S,
                    int Rows_,
                    int Cols_,
                    int Ops_,
                    int MaxRows_,
                    int MaxCols_>
        inline void serialize(
            Archive & ar,
            Eigen::Matrix<S, Rows_, Cols_, Ops_, MaxRows_, MaxCols_> & g,
            const unsigned int version)
        {
            split_free(ar, g, version);
        }
    } // namespace serialization
} // namespace boost

class EigenPath {
  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    ar & configs_;
    ar & lengths_;
    ar & total_length_;
  }

  public:

    explicit EigenPath();
    explicit EigenPath(const std::vector<Eigen::VectorXd>& configs);
    //TODO Deprecated
    explicit EigenPath(const ompl::base::PathPtr& path);
    explicit EigenPath(const RobotPtr& robot, const ompl::base::PathPtr& path);

    /*\brief Get configuration along path at position s in [0,1] */
    Eigen::VectorXd GetConfigAt(float s);

    float GetLength() const;

    void InitLengthFromConfigs(const std::vector<Eigen::VectorXd>& configs);

    void Save(const std::string& filename = kDefaultFileName);
    void Load(const std::string& filename = kDefaultFileName);
    static EigenPath FromFile(const std::string& filename = kDefaultFileName);

  private:
    std::vector<Eigen::VectorXd> configs_;
    std::vector<float> lengths_;
    float total_length_{0.0f};
};
