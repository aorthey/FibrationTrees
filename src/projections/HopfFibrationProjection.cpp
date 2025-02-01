#include "projections/HopfFibrationProjection.hpp"

#include <ompl/base/spaces/SO3StateSpace.h>
#include <ompl/base/spaces/special/SphereStateSpace.h>

#include <boost/math/constants/constants.hpp>
#include "Common.hpp"

using namespace boost::math::double_constants;  // pi
using namespace ompl::base;

HopfFibrationProjection::HopfFibrationProjection(const ompl::base::SpaceInformationPtr& bundle, const ompl::base::SpaceInformationPtr& base) 
  : ompl::multilevel::FiberedProjection(bundle->getStateSpace(), base->getStateSpace()) {
  type_ = ompl::multilevel::PROJECTION_UNKNOWN;
  tmpBase = base->allocState();
  tmpBundle = bundle->allocState();
}

HopfFibrationProjection::~HopfFibrationProjection() {
  getBase()->freeState(tmpBase);
  getBundle()->freeState(tmpBundle);
}

Eigen::Vector3f HopfMap(double a, double b, double c, double d) {
  //Using quaternion representation a + b*i + c*j + d*k
  const auto x = a*a + b*b - c*c - d*d;
  const auto y = 2.0*(a*d + b*c);
  const auto z = 2.0*(b*d - a*c);
  Eigen::Vector3f v;
  v << x, y, z;
  return v;
}

void HopfFibrationProjection::project(const ompl::base::State *xBundle, ompl::base::State *xBase) const {
  const auto so3_state = xBundle->as<SO3StateSpace::StateType>();
  const auto qx = so3_state->x;
  const auto qy = so3_state->y;
  const auto qz = so3_state->z;
  const auto qw = so3_state->w;

  auto v = HopfMap(qw, qx, qy, qz);

  const auto x = v[0];
  const auto y = v[1];
  const auto z = v[2];

  //Cartesian coordinates to spherical coordinates
  const auto r = std::sqrt(x*x + y*y + z*z);
  auto theta = std::atan2(y,x);
  auto phi = std::acos(z/r);

  auto s2_state = xBase->as<SphereStateSpace::StateType>();
  s2_state->setTheta(theta);
  s2_state->setPhi(phi);
}

double HopfFibrationProjection::GetLambdaDistance(double lambda, const ompl::base::State *xBundle, ompl::base::State *xFiber) const {
  xFiber->as<SO2StateSpace::StateType>()->value = lambda;
  lift(tmpBase, xFiber, tmpBundle);
  auto d =  getBundle()->distance(xBundle, tmpBundle);
  return d;
}

void HopfFibrationProjection::projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const {
  project(xBundle, tmpBase);

  double distance = Inf;

  const double kLowerBound = -pi;
  const double kUpperBound = +pi - 1e-6;
  double lower_bound_value = kLowerBound;
  double upper_bound_value = kUpperBound;

  double best_value = 0;
  size_t Ndivision = 5;

  while(distance > Epsilon) {
    double diff = std::abs(lower_bound_value - upper_bound_value);
    for(size_t k = 0; k <= Ndivision; k++) {
      double value = lower_bound_value + (double(k)/double(Ndivision))*diff;
      auto dist_k = GetLambdaDistance(value, xBundle, xFiber);
      if(dist_k < distance) {
        distance = dist_k;
        best_value = value;
      }
    }
    lower_bound_value = std::max(kLowerBound, best_value - (1.0/double(Ndivision)) * diff);
    upper_bound_value = std::min(kUpperBound, best_value + (1.0/double(Ndivision)) * diff);
  }
  xFiber->as<SO2StateSpace::StateType>()->value = best_value;
}

void HopfFibrationProjection::lift(const ompl::base::State *xBase, 
    const ompl::base::State *xFiber, ompl::base::State *xBundle) const {
  const auto s2_state = xBase->as<SphereStateSpace::StateType>();
  const auto fiber_state = xFiber->as<SO2StateSpace::StateType>();
  if(!getFiberSpace()->satisfiesBounds(fiber_state)) {
    getFiberSpace()->printState(fiber_state);
    throw std::out_of_range("Fiber state is out of bounds.");
  }

  //Let lambda be a value in [0, 2pi] which indexes the fiber (circle) on SO3
  const auto lambda = (fiber_state->value + pi); //[0, 2*pi]
  const auto theta = s2_state->getTheta(); //[-pi,pi]
  const auto phi = s2_state->getPhi(); //[0, pi]

  auto x = std::sin(phi) * std::cos(theta);
  auto y = std::sin(phi) * std::sin(theta);
  auto z = std::cos(phi);

  auto n = std::sqrt(2.0*(1.0+x));
  auto x1 = (1+x)/n;
  auto x2 = (y)/n;
  auto x3 = (z)/n;

  auto qw = -x1 * std::sin(lambda);
  auto qx = +x1 * std::cos(lambda);
  auto qy = (x2 * std::cos(lambda) + x3*std::sin(lambda));
  auto qz = (x3 * std::cos(lambda) - x2*std::sin(lambda));

  auto so3_state = xBundle->as<SO3StateSpace::StateType>();
  so3_state->x = qx;
  so3_state->y = qy;
  so3_state->z = qz;
  so3_state->w = qw;

  if(!getBundle()->satisfiesBounds(so3_state)) {
    OMPL_ERROR("Lifting of state did not respect bounds");
    getBundle()->printState(so3_state);
    getBase()->printState(s2_state);
    throw std::out_of_range("Lifting of state did not respect bounds.");
  }
}

ompl::base::StateSpacePtr HopfFibrationProjection::computeFiberSpace() {
  unsigned int N = getDimension();
  unsigned int Y = getBaseDimension();
  if (N != 3 && Y != 2)
  {
      OMPL_ERROR("Assumed input is SO(3) -> S2, but got %d -> %d dimensions.", N, Y);
      throw std::out_of_range("Invalid dimensionality");
  }
  return std::make_shared<ompl::base::SO2StateSpace>();
}

Eigen::Vector3f HopfFibrationProjection::toVector(const ompl::base::State *xBundle) const {
  //Sterographic projection
  const auto so3_state = xBundle->as<SO3StateSpace::StateType>();
  const auto qx = so3_state->x;
  const auto qy = so3_state->y;
  const auto qz = so3_state->z;
  const auto qw = so3_state->w;

  if(std::fabs(1.0 - qw) < 1e-6) {
    OMPL_WARN("Hopf Fibration map is at singularity.");
  }
  const auto x = qx / (1.0-qw);
  const auto y = qy / (1.0-qw);
  const auto z = qz / (1.0-qw);

  Eigen::Vector3f v;
  v << x, y, z;
  return v;
}
