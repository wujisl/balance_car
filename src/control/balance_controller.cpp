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
  _integralDegreesSeconds = 0.0F;
  _state = {};
}

float BalanceController::update(const AttitudeState &attitude, float velocityPitchOffsetDegrees,
                                float deltaSeconds)
{
  _state = {};
  _state.requestedPitchDegrees = _tuning.targetPitchDegrees + velocityPitchOffsetDegrees;
  if (!attitude.valid)
  {
    return 0.0F;
  }

  const float requestedPitchDegrees = _state.requestedPitchDegrees;
  const float pitchErrorDegrees = attitude.pitchDegrees - requestedPitchDegrees;
  const float safeDeltaSeconds = deltaSeconds > 0.0F ? deltaSeconds : 0.0F;
  float candidateIntegral = _integralDegreesSeconds + pitchErrorDegrees * safeDeltaSeconds;
  if (candidateIntegral > _configuration.integralLimit)
  {
    candidateIntegral = _configuration.integralLimit;
  }
  else if (candidateIntegral < -_configuration.integralLimit)
  {
    candidateIntegral = -_configuration.integralLimit;
  }

  _state.pitchErrorDegrees = pitchErrorDegrees;
  _state.proportionalTerm = _tuning.proportionalGain * pitchErrorDegrees;
  _state.derivativeTerm = _tuning.derivativeGain * attitude.pitchRateDps;
  const float candidateMotorCommand = _state.proportionalTerm +
                                      _tuning.integralGain * candidateIntegral +
                                      _state.derivativeTerm;
  const bool saturatedHigh = candidateMotorCommand > _tuning.maximumMotorCommand &&
                             pitchErrorDegrees > 0.0F;
  const bool saturatedLow = candidateMotorCommand < -_tuning.maximumMotorCommand &&
                            pitchErrorDegrees < 0.0F;
  // Do not integrate if the new error would push an already saturated inner
  // loop farther into saturation.  The integral remains continuous when the
  // error later points back toward the usable motor range.
  if (!saturatedHigh && !saturatedLow)
  {
    _integralDegreesSeconds = candidateIntegral;
  }
  _state.integralTerm = _tuning.integralGain * _integralDegreesSeconds;
  float motorCommand = _state.proportionalTerm + _state.integralTerm + _state.derivativeTerm;
  if (_configuration.motorOutputInverted)
  {
    motorCommand = -motorCommand;
  }
  _state.motorCommandRaw = motorCommand;
  _state.saturated = motorCommand > _tuning.maximumMotorCommand ||
                     motorCommand < -_tuning.maximumMotorCommand;

  if (motorCommand > _tuning.maximumMotorCommand) return _tuning.maximumMotorCommand;
  if (motorCommand < -_tuning.maximumMotorCommand) return -_tuning.maximumMotorCommand;
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

const BalanceState &BalanceController::state() const
{
  return _state;
}
} // namespace balance_car::control
