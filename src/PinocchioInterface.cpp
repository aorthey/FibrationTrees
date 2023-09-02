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

const auto blue = osg::Vec4(0.0f,0.0f,1.0f,1.0f);
const auto green = osg::Vec4(0.0f,1.0f,0.0f,1.0f);
const auto red = osg::Vec4(1.0f,0.0f,0.0f,1.0f);
const auto yellow = osg::Vec4(1.0f,1.0f,0.0f,1.0f);
const auto white = osg::Vec4(1.0f,1.0f,1.0f,1.0f);

PinocchioInterface::PinocchioInterface() {
}

osg::Geode* CreateLineSegment(const osg::Vec3& from, const osg::Vec3& to, const osg::Vec4& color = white) {
  osg::Geometry* axis = new osg::Geometry();
  osg::Vec3Array* v = new osg::Vec3Array;
  v->push_back(from);
  v->push_back(to);
  axis->setVertexArray(v);

  osg::Vec4Array* pColors = new osg::Vec4Array;
  pColors->push_back(color);
  axis->setColorArray( pColors );

  auto array = new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2);
  axis->addPrimitiveSet(array);

  osg::Geode *geode = new osg::Geode();
  geode->addDrawable(axis);
  //geode->setColor(osg::Vec4(0.0f,0.0f,1.0f,1.0f));

  osg::LineWidth* linewidth = new osg::LineWidth();
  linewidth->setWidth(3.0f);
  geode->getOrCreateStateSet()->setAttributeAndModes(linewidth, osg::StateAttribute::ON);

  return geode;
}

bool PinocchioInterface::LoadRobot(const std::string& urdf_filename) {
  const std::string robots_model_path = PINOCCHIO_MODELS_DIR;
  pinocchio::urdf::buildModel(urdf_filename, model_);
  std::cout << "Loaded model: " << model_.name << std::endl;
  data_ = Data(model_);

  pinocchio::urdf::buildGeom(model_, urdf_filename, pinocchio::COLLISION, collision_model_, robots_model_path);
  collision_model_.addAllCollisionPairs();
  collision_data_ = GeometryData(collision_model_);

  // for(const auto& object : collision_model_.geometryObjects) {
  //   std::cout << "Free: " << 
  //   (object.geometry->isFree() ? "Yes" : "No") << std::endl;
  // }

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
      std::cout << "collision pair: " << cp.first << " , " << cp.second << " in collision." << std::endl;
      return true;
    }
  }
  return false;
}

Eigen::VectorXd PinocchioInterface::GetRandomConfiguration() {
  return randomConfiguration(model_);
}

#include <osgGA/GUIEventHandler>

class MyKeyboardEventHandler : public osgGA::GUIEventHandler {
public:
  virtual bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter&) {
    if (ea.getEventType() == osgGA::GUIEventAdapter::KEYDOWN) {
      switch (ea.getKey()) {
        case 'a':
          std::cout << "A key pressed" << std::endl;
          break;
        case 'b':
          break;
        default:
          break;
      }
    }
    return false;
  }
};

// std::vector<osg::Matrix> ConfigToFrames(const Eigen::VectorXd& q) {

// }

int PinocchioInterface::Visualize(const Eigen::VectorXd& q) {
  forwardKinematics(model_, data_, q);

  updateGeometryPlacements(model_, data_, collision_model_, collision_data_, q);
  updateGeometryPlacements(model_, data_, visual_model_, visual_data_, q);

  //Group --> Transform --> Node
  osg::Group* root = new osg::Group();

  for(GeomIndex geom_id = 0; geom_id < (GeomIndex)visual_model_.ngeoms; ++geom_id)
  {
    auto geo = collision_model_.geometryObjects[geom_id];
    auto mesh_name = geo.name;

    auto scale = osg::Vec3(geo.meshScale[0], geo.meshScale[1], geo.meshScale[2]);

    osg::MatrixTransform* transform = new osg::MatrixTransform();

    auto SE3 = collision_data_.oMg[geom_id];
    auto t = SE3.translation();

    osg::Vec3 center(t[0], t[1], t[2]);

    Eigen::Quaterniond quaternion(SE3.rotation());

    osg::Quat rotation;
    rotation.set(quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w());

    auto matrix = osg::Matrix::rotate(rotation) * osg::Matrix::translate(center);
    transform->setMatrix(matrix);

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
        const auto& box = static_cast<fcl::Box&>(*geo.geometry);
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
    root->addChild(transform);
  }

  auto floor = 
    new osg::ShapeDrawable(
      new osg::Box(osg::Vec3(0,0,-1.2), 5.0, 5.0, 0.05)
    );
  floor->setColor(osg::Vec4(0.9f,0.9f,0.9f,1.0f));
  root->addChild(floor);

  auto center = osg::Vec3(-1, -1, -1);
  auto xpos = osg::Vec3(center[0]+1.0, center[1], center[2]);
  auto ypos = osg::Vec3(center[0], center[1]+1.5, center[2]);
  auto zpos = osg::Vec3(center[0], center[1], center[2]+2.0);

  auto xaxis = CreateLineSegment(center, xpos, red);
  auto yaxis = CreateLineSegment(center, ypos, green);
  auto zaxis = CreateLineSegment(center, zpos, blue);

  root->addChild(xaxis);
  root->addChild(yaxis);
  root->addChild(zaxis);

  osg::Light *light = new osg::Light;
  light->setPosition( osg::Vec4( 0.0f, 0.0f, 2.0f, 1.0f ) );
  light->setAmbient( osg::Vec4( 0.0f, 0.0f, 0.0f, 1.0f ) );

  osg::LightSource *light_source = new osg::LightSource;
  light_source->setLight(light);
  light_source->setLocalStateSetModes( osg::StateAttribute::ON );
  light_source->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::ON);

  root->addChild(light_source);

  // Create a window and render the scene graph.
  osgViewer::Viewer viewer;
  viewer.setSceneData(root);

  MyKeyboardEventHandler* handler = new MyKeyboardEventHandler();
  viewer.addEventHandler(handler);

  return viewer.run();
}
