#include "control/balance_controller.h"

namespace balance_car::control
{
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
  _state = {};
}

float BalanceController::update(const AttitudeState &attitude, float velocityPitchOffsetDegrees)
{
  if (!attitude.valid)
  {
    _state = {};
    return 0.0F;
  }

  _state.requestedPitchDegrees = _tuning.targetPitchDegrees + velocityPitchOffsetDegrees;
  _state.pitchErrorDegrees = attitude.pitchDegrees - _state.requestedPitchDegrees;
  _integralDegrees += _state.pitchErrorDegrees;
  if (_integralDegrees > _configuration.integralLimit)
  {
    _integralDegrees = _configuration.integralLimit;
  }
  else if (_integralDegrees < -_configuration.integralLimit)
  {
    _integralDegrees = -_configuration.integralLimit;
  }

  _state.proportionalTerm = _tuning.proportionalGain * _state.pitchErrorDegrees;
  _state.integralTerm = _tuning.integralGain * _integralDegrees;
  _state.derivativeTerm = _tuning.derivativeGain * attitude.pitchRateDps;
  _state.unclampedMotorCommand = _state.proportionalTerm + _state.integralTerm + _state.derivativeTerm;
  if (_configuration.motorOutputInverted)
  {
    _state.unclampedMotorCommand = -_state.unclampedMotorCommand;
  }

  _state.outputSaturated = false;
  if (_state.unclampedMotorCommand > _tuning.maximumMotorCommand)
  {
    _state.motorCommand = _tuning.maximumMotorCommand;
    _state.outputSaturated = true;
    return _state.motorCommand;
  }
  if (_state.unclampedMotorCommand < -_tuning.maximumMotorCommand)
  {
    _state.motorCommand = -_tuning.maximumMotorCommand;
    _state.outputSaturated = true;
    return _state.motorCommand;
  }
  _state.motorCommand = _state.unclampedMotorCommand;
  return _state.motorCommand;
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

BalanceTuning BalanceController::tuning() const
{
  return _tuning;
}

const BalanceState &BalanceController::state() const
{
  return _state;
}
} // namespace balance_car::control
