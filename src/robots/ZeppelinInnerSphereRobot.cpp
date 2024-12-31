#include "robots/ZeppelinInnerSphereRobot.hpp"

#include "DartHelper.hpp"
#include "Config.hpp"

dart::dynamics::SkeletonPtr ZeppelinInnerSphereRobot::MakeSkeleton(const YAML::Node& /*node*/) {
  auto radius = ReadConfigVariable<float>("zeppelin_inner_sphere_radius");
  return createSphere(radius);
  // return createSphere(0.05);
}
