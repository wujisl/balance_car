#include "control/balance_controller.h"

namespace balance_car::control
{
namespace
{
constexpr float kAbsoluteMaximumMotorCommand = 1.0F;
}

BalanceController::BalanceController(const config::BalanceConfiguration &configuration)
    : _configuration(configuration),
      _tuning{configuration.targetPitchDegrees, configuration.proportionalGain,
              configuration.integralGain, configuration.derivativeGain,
              configuration.maximumMotorCommand}
{
}

void BalanceController::reset()
{
  _integralDegrees = 0.0F;
}

float BalanceController::update(const AttitudeState &attitude, float velocityPitchOffsetDegrees)
{
  if (!attitude.valid)
  {
    return 0.0F;
  }

  const float requestedPitchDegrees = _tuning.targetPitchDegrees + velocityPitchOffsetDegrees;
  const float pitchErrorDegrees = attitude.pitchDegrees - requestedPitchDegrees;
  _integralDegrees += pitchErrorDegrees;
  if (_integralDegrees > _configuration.integralLimit)
  {
    _integralDegrees = _configuration.integralLimit;
  }
  else if (_integralDegrees < -_configuration.integralLimit)
  {
    _integralDegrees = -_configuration.integralLimit;
  }

  float motorCommand = _tuning.proportionalGain * pitchErrorDegrees +
                       _tuning.integralGain * _integralDegrees +
                       _tuning.derivativeGain * attitude.pitchRateDps;
  if (_configuration.motorOutputInverted)
  {
    motorCommand = -motorCommand;
  }

  if (motorCommand > _tuning.maximumMotorCommand)
  {
    return _tuning.maximumMotorCommand;
  }
  if (motorCommand < -_tuning.maximumMotorCommand)
  {
    return -_tuning.maximumMotorCommand;
  }
  return motorCommand;
}

void BalanceController::setProportionalGain(float gain)
{
  _tuning.proportionalGain = gain < 0.0F ? 0.0F : gain;
}

void BalanceController::setIntegralGain(float gain)
{
  _tuning.integralGain = gain < 0.0F ? 0.0F : gain;
}

void BalanceController::setDerivativeGain(float gain)
{
  _tuning.derivativeGain = gain < 0.0F ? 0.0F : gain;
}

void BalanceController::setTargetPitchDegrees(float targetPitchDegrees)
{
  _tuning.targetPitchDegrees = targetPitchDegrees;
}

void BalanceController::setMaximumMotorCommand(float maximumMotorCommand)
{
  if (maximumMotorCommand < 0.0F)
  {
    _tuning.maximumMotorCommand = 0.0F;
  }
  else if (maximumMotorCommand > kAbsoluteMaximumMotorCommand)
  {
    _tuning.maximumMotorCommand = kAbsoluteMaximumMotorCommand;
  }
  else
  {
    _tuning.maximumMotorCommand = maximumMotorCommand;
  }
}

BalanceTuning BalanceController::tuning() const
{
  return _tuning;
}
} // namespace balance_car::control
