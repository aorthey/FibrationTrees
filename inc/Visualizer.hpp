#pragma once

#include "PinocchioInterface.hpp"
#include <unordered_map>

#include <osg/Group>

class Visualizer {
  public:
    Visualizer(const PinocchioInterface& pinocchio_interface);

    int Run();

    PinocchioInterface& GetPinocchioInterface();

    void UpdateTransforms(const Eigen::VectorXd& q);

    void VisualizeIK(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation_rpy, const pinocchio::GeomIndex& index);
  private:
    PinocchioInterface pinocchio_interface_;
    osg::Group* root_;
    typedef std::unordered_map<osg::Transform*, pinocchio::GeomIndex> TransformToLinkIndex;
    TransformToLinkIndex transform_to_link_index_;
};

