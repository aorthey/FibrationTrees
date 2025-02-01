#pragma once

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/State.h>
#include <ompl/multilevel/datastructures/projections/FiberedProjection.h>

class HopfFibrationProjection : public ompl::multilevel::FiberedProjection
{
 public:
  using ompl::multilevel::FiberedProjection::lift;

  HopfFibrationProjection(const ompl::base::SpaceInformationPtr& bundle, const ompl::base::SpaceInformationPtr& base);
  ~HopfFibrationProjection();

  void project(const ompl::base::State *xBundle, ompl::base::State *xBase) const override;

  void projectFiber(const ompl::base::State *xBundle, ompl::base::State *xFiber) const override;
  void lift(const ompl::base::State *xBase, const ompl::base::State *xFiber,
                    ompl::base::State *xBundle) const override;

  Eigen::Vector3f toVector(const ompl::base::State *xBundle) const;

 protected: 
  double GetLambdaDistance(double lambda, const ompl::base::State *xBundle, ompl::base::State *xFiber) const;

  ompl::base::StateSpacePtr computeFiberSpace() override;

  ompl::base::State* tmpBase;
  ompl::base::State* tmpBundle;
};

