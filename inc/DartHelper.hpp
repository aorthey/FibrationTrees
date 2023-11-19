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

  auto lineFrame = std::make_shared<dart::dynamics::SimpleFrame>(dart::dynamics::Frame::World(), "line", tf);
  auto line = std::make_shared<dart::dynamics::LineSegmentShape>(Eigen::Vector3d::Zero(3), (s2-s1), kPathLineWidth);
  lineFrame->setShape(line);
  lineFrame->createVisualAspect();
  lineFrame->getVisualAspect(true)->setColor(kPathColor);
  return lineFrame;
}
