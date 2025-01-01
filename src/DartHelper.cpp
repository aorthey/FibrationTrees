#include "DartHelper.hpp"

#include <dart/dynamics/TranslationalJoint.hpp>

#include "Common.hpp"
#include "Config.hpp"
#include "OmplHelper.hpp"

void PrintSkeletonInfo(const dart::dynamics::SkeletonPtr& skeleton) {
  if(skeleton == nullptr) {
    return;
  }

  std::cout << "Skeleton: " << skeleton->getName() << std::endl;
  std::cout << "  Num joints : " << skeleton->getNumJoints() << std::endl;
  std::cout << "  Num EE     : " << skeleton->getNumEndEffectors() << std::endl;

  for(size_t k = 0; k < skeleton->getNumEndEffectors(); k++) {
    const auto& ee = skeleton->getEndEffector(k);
    std::cout << "    EE " << k << " is " << ee->getName() << std::endl;
  }
  std::cout << "  Num Links  : " << skeleton->getNumBodyNodes() << std::endl;
  for(const auto& body : skeleton->getBodyNodes()) {
    std::cout <<  "    Body: " << body->getName() << std::endl;
  }

  std::cout << "  Num dofs   : " << skeleton->getNumDofs() << std::endl;
  for(const auto& dof : skeleton->getDofs()) {
    std::cout <<  "    Dof: " << dof->getName() << " (index: " << dof->getIndexInSkeleton() << ")" << std::endl;
  }

  auto config = skeleton->getConfiguration().mPositions;
  auto lb = skeleton->getPositionLowerLimits();
  auto ub = skeleton->getPositionUpperLimits();

  std::cout << "Current Config " << std::endl;
  for(size_t k = 0; k < config.size(); k++) {
    std::cout << lb[k] << " <= " << config[k] << " <= " << ub[k] << std::endl;
  }
}

Eigen::Affine3d create_rotation_matrix(double ax, double ay, double az) {
  Eigen::Affine3d rx =
      Eigen::Affine3d(Eigen::AngleAxisd(ax, Eigen::Vector3d(1, 0, 0)));
  Eigen::Affine3d ry =
      Eigen::Affine3d(Eigen::AngleAxisd(ay, Eigen::Vector3d(0, 1, 0)));
  Eigen::Affine3d rz =
      Eigen::Affine3d(Eigen::AngleAxisd(az, Eigen::Vector3d(0, 0, 1)));
  return rx * ry * rz;
}

StateXd GetRandomPosition(const dart::dynamics::SkeletonPtr& skeleton) {
  auto lb = skeleton->getPositionLowerLimits();
  auto ub = skeleton->getPositionUpperLimits();
  return MakeState(dart::math::Random::uniform(lb, ub));
}

dart::dynamics::SimpleFramePtr createSphereFrame(const State3d& position, const float radius, const State3d& color) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;

  static int counter = 0;
  auto target = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "target"+std::to_string(counter++), tf);
  dart::dynamics::ShapePtr ball(new dart::dynamics::SphereShape(radius));
  target->setShape(ball);
  target->getVisualAspect(true)->setColor(color);
  target->getVisualAspect(true)->show();
  return target;
}

dart::dynamics::SimpleFramePtr createSphereFrame(const StateXd& position, const float radius, const State3d& color) {
  return createSphereFrame(position.configuration, radius, color);
}

dart::dynamics::SimpleFramePtr createCylinderFrame(const State3d& translation, const State3d& rotationXYZ, 
    const float radius, const float height, const State4d& color) {

  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = translation;
  auto R = create_rotation_matrix(rotationXYZ[0],rotationXYZ[1],rotationXYZ[2]);
  tf.linear() = R.linear();

  static int counter = 0;
  auto cylinder_frame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "cylinder"+std::to_string(counter++), tf);
  dart::dynamics::ShapePtr cylinder(new dart::dynamics::CylinderShape(radius, height));
  cylinder_frame->setShape(cylinder);
  cylinder_frame->getVisualAspect(true)->setColor(color);
  cylinder_frame->getVisualAspect(true)->show();
  return cylinder_frame;
}

dart::dynamics::SimpleFramePtr createCylinderFrame(const StateXd& position, const State3d& rotationXYZ, const float radius, const float height, const State4d& color) {
  return createCylinderFrame(position.configuration, rotationXYZ, radius, height, color);
}

dart::dynamics::SimpleFramePtr createLineSegmentFrame(const State3d& s1, const State3d& s2, const State3d& color, float line_width) {
  static int counter = 0;
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = s1;

  auto lineFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "line_segment_"+std::to_string(counter++), tf);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(State3d::Zero(3), (s2-s1), line_width);
  lineFrame->setShape(line);
  lineFrame->createVisualAspect();
  lineFrame->getVisualAspect(true)->setColor(color);
  return lineFrame;
}

dart::dynamics::SimpleFramePtr createLineSegmentFrame(const std::vector<State3d>& vertices, const State3d& color, float line_width) {
  static int counter = 0;
  auto lineFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "line_segments_"+std::to_string(counter++));
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(line_width);
  for(const auto& vertex : vertices) {
    line->addVertex(vertex);
  }
  lineFrame->setShape(line);
  lineFrame->createVisualAspect();
  lineFrame->getVisualAspect(true)->setColor(color);
  return lineFrame;
}

dart::dynamics::SkeletonPtr createFloor(float z_position, float width) {
    dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor");
    dart::dynamics::BodyNodePtr body =
        floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;

    constexpr double kFloorHeight = 0.1;
    auto box = std::make_shared<dart::dynamics::BoxShape>(
        State3d{width, width, kFloorHeight});
    auto shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(State3d(0.9, 0.9, 0.9));

    /* Put the floor into position */
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = State3d{0.0, 0.0, z_position + (-kFloorHeight/2.0 - kFloorHeight/100.0)};
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    body->setName("floor");

    return floor;
}

dart::dynamics::SkeletonPtr createBox(const State3d& position, float length_x, float length_y, float length_z) {
    static int counter = 0;
    std::string name = "box_"+std::to_string(counter++);
    dart::dynamics::SkeletonPtr box = dart::dynamics::Skeleton::create(name);
    dart::dynamics::BodyNodePtr body =
        box->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;

    auto box_shape = std::make_shared<dart::dynamics::BoxShape>(
        State3d{length_x, length_y, length_z});
    auto shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box_shape);
    shapeNode->getVisualAspect()->setColor(State3d(0.9, 0.9, 0.9));

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = position;
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    body->setName(name);
    return box;
}

dart::dynamics::SkeletonPtr createPlanarBox(const State3d& position, float length_x, float length_y, float length_z) {
    static int counter = 0;
    std::string name = "planar_box_"+std::to_string(counter++);
    dart::dynamics::SkeletonPtr box = dart::dynamics::Skeleton::create(name);
    auto joint = box->createJointAndBodyNodePair<dart::dynamics::PlanarJoint>(nullptr);
    joint.first->setXYPlane();
    dart::dynamics::BodyNodePtr body = joint.second;

    auto box_shape = std::make_shared<dart::dynamics::BoxShape>(
        State3d{length_x, length_y, length_z});
    auto shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box_shape);
    shapeNode->getVisualAspect()->setColor(State3d(0.9, 0.9, 0.9));

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = position;
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    body->setName(name);
    return box;
}

dart::dynamics::SkeletonPtr createCylinder(const State3d& position, const State3d& rotationXYZ, float radius, float height) {
    static int counter = 0;
    std::string name = "cylinder_"+std::to_string(counter++);
    dart::dynamics::SkeletonPtr cylinder = dart::dynamics::Skeleton::create(name);

    auto joint = cylinder->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint2D>(nullptr);
    joint.first->setXYPlane();

    dart::dynamics::BodyNodePtr body = joint.second;
    dart::dynamics::ShapePtr shape = std::make_shared<dart::dynamics::CylinderShape>(radius, height);

    auto shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(shape);
    shapeNode->getVisualAspect()->setColor(State3d(0.9, 0.9, 0.9));

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = position;
    auto R = create_rotation_matrix(rotationXYZ[0],rotationXYZ[1],rotationXYZ[2]);
    tf.linear() = R.linear();
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    body->setName(name);

    return cylinder;
}

dart::dynamics::SkeletonPtr createCylinder(const State3d& position, float radius, float height) {
  auto rotation = MakeState3d({0, 0, 0});
  return createCylinder(position, rotation, radius, height);
}

dart::dynamics::SkeletonPtr createSphere(float radius) {
    static int counter = 0;
    std::string name = "sphere_"+std::to_string(counter++);
    dart::dynamics::SkeletonPtr sphere = dart::dynamics::Skeleton::create(name);

    dart::dynamics::BodyNodePtr body =
        sphere->createJointAndBodyNodePair<dart::dynamics::TranslationalJoint>(nullptr).second;
    dart::dynamics::ShapePtr shape = std::make_shared<dart::dynamics::SphereShape>(radius);

    auto shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(shape);
    shapeNode->getVisualAspect()->setColor(State3d(0.5, 0.0, 0.5));

    auto ll = MakeState(ReadConfigVariable<std::vector<double>>("sphere_robot_lower_limit"));
    auto ul = MakeState(ReadConfigVariable<std::vector<double>>("sphere_robot_upper_limit"));
    sphere->setPositionLowerLimits(ll.configuration);
    sphere->setPositionUpperLimits(ul.configuration);

    body->setName(name);

    return sphere;
}

dart::dynamics::SkeletonPtr createFromURDF(const std::string& urdf_name, const State3d& position)
{
    dart::utils::DartLoader loader;
    dart::utils::DartLoader::Options options;
    options.mDefaultRootJointType = dart::utils::DartLoader::RootJointType::FIXED;
    loader.setOptions(options);

    dart::dynamics::SkeletonPtr object
      = loader.parseSkeleton(urdf_name);

    static int count = 0;
    object->setMobile(false);

    auto body = object->getRootBodyNode();

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = position;
    body->getParentJoint()->setTransformFromParentBodyNode(tf);

    return object;
}

void addCoordinateFrameToWorld(const dart::simulation::WorldPtr& world) {
  auto frame_x = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "coordinate_frame_x");
  auto frame_y = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "coordinate_frame_y");
  auto frame_z = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "coordinate_frame_z");

  const float kCoordinateFrameLength = 0.5;
  State3d origin(-0.0, -0.0, 0.0);
  State3d ex(kCoordinateFrameLength, 0, 0);
  State3d ey(0, kCoordinateFrameLength, 0);
  State3d ez(0, 0, kCoordinateFrameLength);

  State3d color_ex(1, 0, 0);
  State3d color_ey(0, 1, 0);
  State3d color_ez(0, 0, 1);

  const float line_width = 5.0;

  auto line_x = std::make_shared<dart::dynamics::LineSegmentShape>(origin, (ex + origin), line_width);
  frame_x->setShape(line_x);
  frame_x->createVisualAspect();
  frame_x->getVisualAspect(true)->setColor(color_ex);

  auto line_y = std::make_shared<dart::dynamics::LineSegmentShape>(origin, (ey + origin), line_width);
  frame_y->setShape(line_y);
  frame_y->createVisualAspect();
  frame_y->getVisualAspect(true)->setColor(color_ey);

  auto line_z = std::make_shared<dart::dynamics::LineSegmentShape>(origin, (ez + origin), line_width);
  frame_z->setShape(line_z);
  frame_z->createVisualAspect();
  frame_z->getVisualAspect(true)->setColor(color_ez);

  world->addSimpleFrame(frame_x);
  world->addSimpleFrame(frame_y);
  world->addSimpleFrame(frame_z);
}

void changeBodyColor(const dart::dynamics::SkeletonPtr& skeleton, const Eigen::Vector4d& color) 
{
  for(const auto& body_node : skeleton->getBodyNodes()) {
    auto shapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
    for(const auto& node : shapeNodes) {
      auto properties(node->getVisualAspect()->getProperties());
      properties.mHidden = false;
      properties.mShadowed = true;
      properties.mUseDefaultColor = true;
      properties.mRGBA = color;
      node->getVisualAspect()->setProperties(properties);
    }
  }
}

void hideSkeleton(const dart::dynamics::SkeletonPtr& skeleton, bool enable) {
  for(const auto& body_node : skeleton->getBodyNodes()) {
    auto shapeNodes = body_node->getShapeNodesWith<dart::dynamics::VisualAspect>();
    for(const auto& node : shapeNodes) {
      auto properties(node->getVisualAspect()->getProperties());
      properties.mHidden = enable;
      node->getVisualAspect()->setProperties(properties);
    }
  }
}

void hide(const dart::dynamics::SkeletonPtr& skeleton) {
  hideSkeleton(skeleton, true);
}

void show(const dart::dynamics::SkeletonPtr& skeleton) {
  hideSkeleton(skeleton, false);
}
