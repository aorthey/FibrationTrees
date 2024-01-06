#include "robots/SphereRobot.hpp"

#include <ompl/base/spaces/RealVectorStateSpace.h>

#include "DartHelper.hpp"

dart::dynamics::SkeletonPtr SphereRobot::MakeSkeleton() {
  return createSphere(0.01);
}
