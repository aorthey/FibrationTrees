#include <string>

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/jacobian.hpp>


class PinocchioInterface {
  public:
    PinocchioInterface();

    bool LoadRobot(const std::string& filename);

    Eigen::VectorXd GetRandomConfiguration();

    bool IsInCollision(const Eigen::VectorXd& q);

    int Visualize(const Eigen::VectorXd& q);

    //std::vector<osg::Matrix> ConfigToFrames(const Eigen::VectorXd& q);

  private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel collision_model_;
    pinocchio::GeometryData collision_data_;
    pinocchio::GeometryModel visual_model_;
    pinocchio::GeometryData visual_data_;
};
