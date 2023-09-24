#pragma once
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

    Eigen::VectorXd ComputeInverseKinematics(const Eigen::VectorXd& seed, const pinocchio::SE3& pose, const pinocchio::GeomIndex& index);

    bool IsInCollision(const Eigen::VectorXd& q);

    void UpdateModel(const Eigen::VectorXd& q);

    const pinocchio::GeometryModel& GetCollisionModel() const;
    const pinocchio::GeometryData& GetCollisionData() const;

    std::vector<pinocchio::GeomIndex> GetContactLinkIndices();

  private:
    pinocchio::Model model_;
    pinocchio::Data data_;
    pinocchio::GeometryModel collision_model_;
    pinocchio::GeometryData collision_data_;
    pinocchio::GeometryModel visual_model_;
    pinocchio::GeometryData visual_data_;
};
