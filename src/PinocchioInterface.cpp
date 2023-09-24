#include "PinocchioInterface.hpp"

#include <osg/Node>
#include <osg/Group>
#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/Material>
#include <osg/LineWidth>
#include <osg/Texture2D>
#include <osg/MatrixTransform>
#include <osg/io_utils>
#include <osgDB/ReadFile>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

using namespace pinocchio;

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODELS_DIR "/home/aorthey/git/FibrationTrees/data/nasa_valkyrie_model/"
#endif

PinocchioInterface::PinocchioInterface() {
}

const pinocchio::GeometryModel& PinocchioInterface::GetCollisionModel() const {
  return collision_model_;
}
const pinocchio::GeometryData& PinocchioInterface::GetCollisionData() const {
  return collision_data_;
}

bool PinocchioInterface::LoadRobot(const std::string& urdf_filename) {
  const std::string robots_model_path = PINOCCHIO_MODELS_DIR;
  pinocchio::urdf::buildModel(urdf_filename, pinocchio::JointModelFreeFlyer(), model_);
  std::cout << "Loaded model: " << model_.name << std::endl;
  data_ = Data(model_);

  pinocchio::urdf::buildGeom(model_, urdf_filename, pinocchio::COLLISION, collision_model_, robots_model_path);
  collision_model_.addAllCollisionPairs();
  collision_data_ = GeometryData(collision_model_);

  pinocchio::urdf::buildGeom(model_, urdf_filename, pinocchio::VISUAL, visual_model_, robots_model_path);
  visual_model_.addAllCollisionPairs();
  visual_data_ = GeometryData(visual_model_);
  return true;
}

bool PinocchioInterface::IsInCollision(const Eigen::VectorXd& q) {
  // std::cout << "Collision check of q: " << q.transpose() << std::endl;
  computeCollisions(model_, data_, collision_model_, collision_data_, q);

  for(size_t k = 0; k < collision_model_.collisionPairs.size(); ++k)
  {
    const CollisionPair & cp = collision_model_.collisionPairs.at(k);
    const hpp::fcl::CollisionResult & cr = collision_data_.collisionResults.at(k);

    // std::cout << "collision pair: " << cp.first << " , " << cp.second << " - collision: ";
    // std::cout << (cr.isCollision() ? "yes" : "no") << std::endl;
    if(cr.isCollision()) {
      std::cout << "collision detected in pair: " << cp.first << " , " << cp.second << std::endl;
      return true;
    }
  }
  return false;
}

Eigen::VectorXd PinocchioInterface::GetRandomConfiguration() {
  const auto q = randomConfiguration(model_);
  std::cout << q.size() << std::endl;
  return q;
}

void PinocchioInterface::UpdateModel(const Eigen::VectorXd& q) {
  forwardKinematics(model_, data_, q);
  updateGeometryPlacements(model_, data_, collision_model_, collision_data_, q);
}


std::vector<pinocchio::GeomIndex> PinocchioInterface::GetContactLinkIndices() {
  std::vector<pinocchio::GeomIndex> indices;
  const auto& model = GetCollisionModel();
  for(pinocchio::GeomIndex geom_id = 0; geom_id < (pinocchio::GeomIndex)model.ngeoms; ++geom_id)
  {
    auto geo = model.geometryObjects[geom_id];
    std::cout << geo.name << std::endl;
    if(geo.name == "torso_0") {
      indices.push_back(geom_id);
    }
    if(geo.name == "rightHipPitchLink_0") {
      indices.push_back(geom_id);
    }
  }
  return indices;
}

// pinocchio::SE3 PinocchioInterface::GetContactPointTransform(const Eigen::VectorXd& q, const pinocchio::GeomIndex& index) {
//   const auto& model = GetCollisionModel();
//   const auto& data = GetCollisionData();

//   auto geo = model.geometryObjects[index];
//   auto scale = osg::Vec3(geo.meshScale[0], geo.meshScale[1], geo.meshScale[2]);

//   osg::MatrixTransform* transform = new osg::MatrixTransform();

//   auto SE3 = data.oMg[index];
//   auto t = SE3.translation();

//   osg::Vec3 center(t[0], t[1], t[2]);

//   Eigen::Quaterniond quaternion(SE3.rotation());

//   osg::Quat rotation;
//   rotation.set(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

//   auto matrix = osg::Matrix::rotate(rotation) * osg::Matrix::translate(center);
// }

//IK SOLVER:
//https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/md_doc_b-examples_i-inverse-kinematics.html
//
#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"

 // pinocchio::Model model;
 // pinocchio::buildModels::manipulator(model);
 // pinocchio::Data data(model);

Eigen::VectorXd PinocchioInterface::ComputeInverseKinematics(const Eigen::VectorXd& seed, const pinocchio::SE3& pose, const pinocchio::GeomIndex& index) {
 const double eps  = 1e-4;
 const int IT_MAX  = 1000;
 const double DT   = 1e-1;
 const double damp = 1e-4;

 Eigen::VectorXd q = pinocchio::neutral(model_);

 pinocchio::Data::Matrix6x J(6, model_.nv);
 J.setZero();

 bool success = false;
 typedef Eigen::Matrix<double, 6, 1> Vector6d;
 Vector6d err;
 Eigen::VectorXd v(model_.nv);
 for (int i=0;;i++)
 {
   pinocchio::forwardKinematics(model_, data_, q);
   const pinocchio::SE3 dMi = pose.actInv(data_.oMi[index]);
   err = pinocchio::log6(dMi).toVector();
   if(err.norm() < eps)
   {
     success = true;
     break;
   }
   if (i >= IT_MAX)
   {
     success = false;
     break;
   }
   pinocchio::computeJointJacobian(model_, data_, q, index, J);
   pinocchio::Data::Matrix6 JJt;
   JJt.noalias() = J * J.transpose();
   JJt.diagonal().array() += damp;
   v.noalias() = - J.transpose() * JJt.ldlt().solve(err);
   q = pinocchio::integrate(model_, q, v*DT);
   if(!(i%10))
     std::cout << i << ": error = " << err.transpose() << std::endl;
 }

 if(success)
 {
   std::cout << "Convergence achieved!" << std::endl;
 }
 else
 {
   std::cout << "\nWarning: the iterative algorithm has not reached convergence to the desired precision" << std::endl;
 }

 std::cout << "\nresult: " << q.transpose() << std::endl;
 std::cout << "\nfinal error: " << err.transpose() << std::endl;
 return q;
}
