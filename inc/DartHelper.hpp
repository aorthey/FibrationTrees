#include "dart/dart.hpp"
#include "dart/gui/osg/osg.hpp"
#include "dart/utils/urdf/urdf.hpp"
#include "dart/math/Random.hpp"

const Eigen::Vector3d kPathColor = Eigen::Vector3d(0.3, 0.8, 0.3);
const float kPathSphereSize = 0.01;
const float kPathLineWidth = 5.0;

void PrintSkeletonInfo(const dart::dynamics::SkeletonPtr& skeleton) {

  std::cout << "Skeleton: " << skeleton->getName() << std::endl;
  std::cout << "  Num joints : " << skeleton->getNumJoints() << std::endl;
  std::cout << "  Num dofs   : " << skeleton->getNumDofs() << std::endl;
  std::cout << "  Num EE     : " << skeleton->getNumEndEffectors() << std::endl;

  for(size_t k = 0; k < skeleton->getNumEndEffectors(); k++) {
    const auto& ee = skeleton->getEndEffector(k);
    std::cout << "    EE " << k << " is " << ee->getName() << std::endl;
  }
  std::cout << "  Num Links  : " << skeleton->getNumBodyNodes() << std::endl;
  for(const auto& body : skeleton->getBodyNodes()) {
    std::cout <<  "Body: " << body->getName() << std::endl;
  }

  auto config = skeleton->getConfiguration().mPositions;
  auto lb = skeleton->getPositionLowerLimits();
  auto ub = skeleton->getPositionUpperLimits();

  std::cout << "Current Config " << std::endl;
  for(size_t k = 0; k < config.size(); k++) {
    std::cout << lb[k] << " <= " << config[k] << " <= " << ub[k] << std::endl;
  }
}

Eigen::VectorXd GetRandomPosition(const dart::dynamics::SkeletonPtr& skeleton) {
  auto lb = skeleton->getPositionLowerLimits();
  auto ub = skeleton->getPositionUpperLimits();
  return dart::math::Random::uniform(lb, ub);
}

dart::dynamics::SimpleFramePtr createSphereFrame(const Eigen::Vector3d& position) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = position;

  auto target = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "target", tf);
  dart::dynamics::ShapePtr ball(new dart::dynamics::SphereShape(kPathSphereSize));
  target->setShape(ball);
  target->getVisualAspect(true)->setColor(kPathColor);
  return target;
}

dart::dynamics::SimpleFramePtr createLineSegmentFrame(const Eigen::Vector3d& s1, const Eigen::Vector3d& s2) {
  Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
  tf.translation() = s1;

  static int counter = 0;
  auto lineFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "line"+std::to_string(counter++), tf);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(Eigen::Vector3d::Zero(3), (s2-s1), kPathLineWidth);
  lineFrame->setShape(line);
  lineFrame->createVisualAspect();
  lineFrame->getVisualAspect(true)->setColor(kPathColor);
  return lineFrame;
}

dart::dynamics::SkeletonPtr createFloor() {
    dart::dynamics::SkeletonPtr floor = dart::dynamics::Skeleton::create("floor");
    dart::dynamics::BodyNodePtr body =
        floor->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;

    constexpr double floorWidth = 3.0;
    constexpr double floorHeight = 0.1;
    auto box = std::make_shared<dart::dynamics::BoxShape>(
        Eigen::Vector3d{floorWidth, floorWidth, floorHeight});
    auto shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(box);
    shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.9, 0.9, 0.9));

    /* Put the floor into position */
    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = Eigen::Vector3d{0.0, 0.0, -floorHeight/2.0 - floorHeight/100.0};
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    body->setName("floor");

    return floor;
}

dart::dynamics::SkeletonPtr createCylinder(const Eigen::Vector3d& position, float radius, float height) {
    dart::dynamics::SkeletonPtr cylinder = dart::dynamics::Skeleton::create("cylinder");

    dart::dynamics::BodyNodePtr body =
        cylinder->createJointAndBodyNodePair<dart::dynamics::WeldJoint>(nullptr).second;
    dart::dynamics::ShapePtr shape = std::make_shared<dart::dynamics::CylinderShape>(radius, height);

    auto shapeNode = body->createShapeNodeWith<dart::dynamics::VisualAspect, dart::dynamics::CollisionAspect, dart::dynamics::DynamicsAspect>(shape);
    shapeNode->getVisualAspect()->setColor(Eigen::Vector3d(0.9, 0.9, 0.9));

    Eigen::Isometry3d tf(Eigen::Isometry3d::Identity());
    tf.translation() = position;
    body->getParentJoint()->setTransformFromParentBodyNode(tf);
    body->setName("cylinder");

    return cylinder;
}

bool IsInCollision(const dart::dynamics::SkeletonPtr& rhs, const dart::dynamics::SkeletonPtr& lhs, const dart::simulation::WorldPtr& world) {
  auto collisionEngine
      = world->getConstraintSolver()->getCollisionDetector();
  auto rhsGroup = collisionEngine->createCollisionGroup(rhs.get());
  auto lhsGroup = collisionEngine->createCollisionGroup(lhs.get());

  dart::collision::CollisionOption option;
  dart::collision::CollisionResult result;
  bool collision = collisionEngine->collide(rhsGroup.get(), lhsGroup.get(), option, &result);

  for(const auto& body_node : result.getCollidingBodyNodes()) {
    std::cout << "Colliding body: " << body_node->getName() << std::endl;
  }
  return collision;
}

