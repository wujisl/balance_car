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

struct BalanceState
{
  float requestedPitchDegrees = 0.0F;
  float pitchErrorDegrees = 0.0F;
  float proportionalTerm = 0.0F;
  float integralTerm = 0.0F;
  float derivativeTerm = 0.0F;
  float unclampedMotorCommand = 0.0F;
  float motorCommand = 0.0F;
  bool outputSaturated = false;
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
  BalanceTuning tuning() const;
  const BalanceState &state() const;

private:
  const config::BalanceConfiguration &_configuration;
  BalanceTuning _tuning;
  float _integralDegrees = 0.0F;
  BalanceState _state;
};
} // namespace balance_car::control
