#include "Visualizer.hpp"

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

#include <osgGA/GUIEventHandler>
#include <osgGA/TrackballManipulator>

#include "OsgPrimitives.hpp"

class KeyboardEventHandler : public osgGA::GUIEventHandler {
public:
  KeyboardEventHandler(Visualizer* visualizer) : visualizer_(visualizer) {
    auto& pi = visualizer_->GetPinocchioInterface();
    const auto indices = pi.GetContactLinkIndices();
    visualizer_->VisualizeIK(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,0), indices.at(0));
  };

  virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'a':
          {
            std::cout << "Pressed Key " << 'a' << std::endl;
            auto& pi = visualizer_->GetPinocchioInterface();
            const auto q = pi.GetRandomConfiguration();
            visualizer_->UpdateTransforms(q);
            if(pi.IsInCollision(q)) {
              std::cout << "Configuration " << q << " is in collision." << std::endl;
            }
          }
          break;
        case 'b':
          {
            auto& pi = visualizer_->GetPinocchioInterface();
            const auto indices = pi.GetContactLinkIndices();
            visualizer_->VisualizeIK(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1.57), indices.at(0));
          }
          break;
        case 'c':
          {
            auto& pi = visualizer_->GetPinocchioInterface();
            const auto indices = pi.GetContactLinkIndices();
            visualizer_->VisualizeIK(Eigen::Vector3d(0,0,0), Eigen::Vector3d(0,0,1.57), indices.at(1));
          }
          break;
        default:
          break;
      }
    }
    return false;
  }

private:
  Visualizer* visualizer_;
  pinocchio::GeomIndex index_{0};
};

void Visualizer::VisualizeIK(const Eigen::Vector3d& translation, const Eigen::Vector3d& rotation_rpy, const pinocchio::GeomIndex& index) {
  auto& pi = GetPinocchioInterface();

  const auto& model = pi.GetCollisionModel();
  auto geo = model.geometryObjects[index];

  Eigen::AngleAxisd rollAngle(rotation_rpy[0], Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd pitchAngle(rotation_rpy[1], Eigen::Vector3d::UnitY());
  Eigen::AngleAxisd yawAngle(rotation_rpy[2], Eigen::Vector3d::UnitZ());
  Eigen::Quaternion<double> quat = rollAngle * pitchAngle * yawAngle;
  Eigen::Matrix3d rotationMatrix = quat.matrix(); 

  const pinocchio::SE3 pose(rotationMatrix, Eigen::Vector3d(translation[0],translation[1],translation[2]));
  const auto seed = pi.GetRandomConfiguration();
  const auto q = pi.ComputeInverseKinematics(seed, pose, index);

  UpdateTransforms(q);
  std::cout << "Index[" << index <<"] : " << geo.name << std::endl;
  std::cout << q.size() << std::endl;
}


void Visualizer::UpdateTransforms(const Eigen::VectorXd& q) {
  pinocchio_interface_.UpdateModel(q);

  auto model = pinocchio_interface_.GetCollisionModel();
  auto data = pinocchio_interface_.GetCollisionData();

  for (size_t k = 0; k < root_->getNumChildren(); k++) {
    osg::Node* child = root_->getChild(k);
    osg::MatrixTransform* transform = dynamic_cast<osg::MatrixTransform*>(child);
    if(!transform) {
      continue;
    }
    auto geom_id = transform_to_link_index_[transform];
    auto geo = model.geometryObjects[geom_id];
    auto scale = osg::Vec3(geo.meshScale[0], geo.meshScale[1], geo.meshScale[2]);

    auto SE3 = data.oMg[geom_id];
    auto t = SE3.translation();

    osg::Vec3 center(t[0], t[1], t[2]);

    Eigen::Quaterniond quaternion(SE3.rotation());

    osg::Quat rotation;
    rotation.set(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

    auto matrix = osg::Matrix::rotate(rotation) * osg::Matrix::translate(center);
    //std::cout << matrix << std::endl;
    transform->setMatrix(matrix);
  }
  std::cout << "Updated " << root_->getNumChildren() << " nodes." << std::endl;
}

PinocchioInterface& Visualizer::GetPinocchioInterface() {
  return pinocchio_interface_;
}

Visualizer::Visualizer(const PinocchioInterface& pinocchio_interface) :
  pinocchio_interface_(pinocchio_interface) 
{
  root_ = new osg::Group();

  //////////////////////////////////////////////////////////////////////////////////
  //Create scenario
  //////////////////////////////////////////////////////////////////////////////////
  auto floor = 
    new osg::ShapeDrawable(
      new osg::Box(osg::Vec3(0,0,-1.2), 5.0, 5.0, 0.05)
    );
  floor->setColor(osg::Vec4(0.9f,0.9f,0.9f,1.0f));
  root_->addChild(floor);

  auto center = osg::Vec3(-0, -0, -0);
  auto xpos = osg::Vec3(center[0]+1.0, center[1], center[2]);
  auto ypos = osg::Vec3(center[0], center[1]+1.0, center[2]);
  auto zpos = osg::Vec3(center[0], center[1], center[2]+1.0);

  auto xaxis = CreateLineSegment(center, xpos, red);
  auto yaxis = CreateLineSegment(center, ypos, green);
  auto zaxis = CreateLineSegment(center, zpos, blue);

  root_->addChild(xaxis);
  root_->addChild(yaxis);
  root_->addChild(zaxis);

  osg::Light *light = new osg::Light;
  light->setPosition( osg::Vec4( 0.0f, 0.0f, 2.0f, 1.0f ) );
  light->setAmbient( osg::Vec4( 0.0f, 0.0f, 0.0f, 1.0f ) );

  osg::LightSource *light_source = new osg::LightSource;
  light_source->setLight(light);
  light_source->setLocalStateSetModes( osg::StateAttribute::ON );
  light_source->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

  root_->addChild(light_source);

  //////////////////////////////////////////////////////////////////////////////////
  //Add robot model
  //////////////////////////////////////////////////////////////////////////////////
  pinocchio_interface_.UpdateModel(pinocchio_interface_.GetRandomConfiguration());

  auto model = pinocchio_interface_.GetCollisionModel();
  auto data = pinocchio_interface_.GetCollisionData();

  for(pinocchio::GeomIndex geom_id = 0; geom_id < (pinocchio::GeomIndex)model.ngeoms; ++geom_id)
  {
    auto geo = model.geometryObjects[geom_id];
    auto scale = osg::Vec3(geo.meshScale[0], geo.meshScale[1], geo.meshScale[2]);

    osg::MatrixTransform* transform = new osg::MatrixTransform();

    auto SE3 = data.oMg[geom_id];
    auto t = SE3.translation();

    osg::Vec3 center(t[0], t[1], t[2]);

    Eigen::Quaterniond quaternion(SE3.rotation());

    osg::Quat rotation;
    rotation.set(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

    auto matrix = osg::Matrix::rotate(rotation) * osg::Matrix::translate(center);
    transform->setMatrix(matrix);

    transform_to_link_index_[transform] = geom_id;

    switch (geo.geometry->getNodeType())
    {
      case hpp::fcl::BV_AABB:
        std::cout << "AABB" << std::endl;
        break;
      case hpp::fcl::BV_OBB:
        std::cout << "OBB" << std::endl;
        break;
      case hpp::fcl::BV_RSS:
        std::cout << "RSS" << std::endl;
        break;
      case hpp::fcl::BV_kIOS:
        std::cout << "kIOS" << std::endl;
        break;
      case hpp::fcl::BV_OBBRSS:
        {
        auto mesh_path = geo.meshPath.c_str();
        std::cout << "OBBRSS : " << mesh_path << std::endl;
        osg::Node* mesh_node = osgDB::readNodeFile(mesh_path);
        if (!mesh_node) {
          std::cout << "Could not load" << std::endl;
        }
        transform->addChild(mesh_node);
        break;
        }
      case hpp::fcl::BV_KDOP16:
        std::cout << "KDOP16" << std::endl;
        break;
      case hpp::fcl::BV_KDOP18:
        std::cout << "KDOP18" << std::endl;
        break;
      case hpp::fcl::BV_KDOP24:
        std::cout << "KDOP24" << std::endl;
        break;
      case hpp::fcl::GEOM_BOX: 
        {
        std::cout << "BOX" << std::endl;
        const auto& box = static_cast<pinocchio::fcl::Box&>(*geo.geometry);
        auto lx = 2*(float) box.halfSide[0];
        auto ly = 2*(float) box.halfSide[1];
        auto lz = 2*(float) box.halfSide[2];
        auto drawable = 
          new osg::ShapeDrawable(
            new osg::Box(osg::Vec3(0,0,0), lx, ly, lz)
          );
        drawable->setColor(osg::Vec4(1.0f,1.0f,0.0f,1.0f));
        transform->addChild(drawable);
        break;
        }
      case hpp::fcl::GEOM_SPHERE: 
        std::cout << "SPHERE" << std::endl;
        break;
      case hpp::fcl::GEOM_CAPSULE: 
        std::cout << "CAPSULE" << std::endl;
        break;
      case hpp::fcl::GEOM_CONE:     
        std::cout << "CONE" << std::endl;
        break;
      case hpp::fcl::GEOM_CYLINDER: 
        std::cout << "CYLINDER" << std::endl;
        break;
      case hpp::fcl::GEOM_CONVEX:
        std::cout << "CONVEX" << std::endl;
        break;
      case hpp::fcl::GEOM_PLANE:
        std::cout << "PLANE" << std::endl;
        break;
      case hpp::fcl::GEOM_HALFSPACE:
        std::cout << "HALFSPACE" << std::endl;
        break;
      case hpp::fcl::GEOM_TRIANGLE:
        std::cout << "TRIANGLE" << std::endl;
        break;
      default:
        std::cout << "NYI" << std::endl;
        break;
    }
    root_->addChild(transform);
  }
}

int Visualizer::Run() {
  osgViewer::Viewer viewer;
  viewer.setSceneData(root_);

  KeyboardEventHandler* handler = new KeyboardEventHandler(this);
  viewer.addEventHandler(handler);
  viewer.setCameraManipulator(new osgGA::TrackballManipulator());
  return viewer.run();
}
