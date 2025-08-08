#pragma once

#include "robots/MobileKukaBase.hpp"

class TimeBasedMobileKukaBase : public MobileKukaBase {
  const double kDefaultVMax = 1.0;
  const double kDefaultTMax = 20.0;
  public:
    TimeBasedMobileKukaBase() = default;
    TimeBasedMobileKukaBase(float vMax, float tMax);

    ompl::multilevel::FactoredSpaceInformationPtr MakeSpaceInformation(const RobotPtr& robot) override;
    StateXd StateToEigen(const ompl::base::State* state) const override;
    void EigenToState(const StateXd& v, ompl::base::State* state) const override;

    float GetVMax() const;
    float GetTMax() const;
    void SetVMax(float vMax);
    void SetTMax(float tMax);

  private:
    float vMax_{kDefaultVMax};
    float tMax_{kDefaultTMax};
};

