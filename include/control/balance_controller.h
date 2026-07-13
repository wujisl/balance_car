#pragma once

#include "config/vehicle_config.h"
#include "control/attitude_estimator.h"

namespace balance_car::control
{
struct BalanceTuning
{
  float targetPitchDegrees;
  float proportionalGain;
  float integralGain;
  float derivativeGain;
  float maximumMotorCommand;
};

class BalanceController
{
public:
  explicit BalanceController(const config::BalanceConfiguration &configuration);

  void reset();
  float update(const AttitudeState &attitude, float velocityPitchOffsetDegrees);
  void setProportionalGain(float gain);
  void setIntegralGain(float gain);
  void setDerivativeGain(float gain);
  void setTargetPitchDegrees(float targetPitchDegrees);
  void setMaximumMotorCommand(float maximumMotorCommand);
  BalanceTuning tuning() const;

private:
  const config::BalanceConfiguration &_configuration;
  BalanceTuning _tuning;
  float _integralDegrees = 0.0F;
};
} // namespace balance_car::control
