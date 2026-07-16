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

// Latest inner-loop quantities, captured during update() for telemetry only.
struct BalanceState
{
  float requestedPitchDegrees = 0.0F;
  float pitchErrorDegrees = 0.0F;
  float proportionalTerm = 0.0F;
  float integralTerm = 0.0F;
  float derivativeTerm = 0.0F;
  float motorCommandRaw = 0.0F;
  bool saturated = false;
};

class BalanceController
{
public:
  explicit BalanceController(const config::BalanceConfiguration &configuration);

  void reset();
  float update(const AttitudeState &attitude, float velocityPitchOffsetDegrees, float deltaSeconds);
  void setProportionalGain(float gain);
  void setIntegralGain(float gain);
  void setDerivativeGain(float gain);
  void setTargetPitchDegrees(float targetPitchDegrees);
  void setMaximumMotorCommand(float maximumMotorCommand);
  BalanceTuning tuning() const;
  const BalanceState &state() const;

private:
  const config::BalanceConfiguration &_configuration;
  BalanceTuning _tuning;
  BalanceState _state;
  float _integralDegreesSeconds = 0.0F;
};
} // namespace balance_car::control
