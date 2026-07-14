#include "control/differential_speed_controller.h"

namespace balance_car::control
{
namespace
{
constexpr float kAbsoluteMaximumTurnMotorCommand = 1.0F;
}

DifferentialSpeedController::DifferentialSpeedController(
    const config::DifferentialSpeedConfiguration &configuration)
    : _configuration(configuration),
      _tuning{configuration.proportionalGain, configuration.integralGain,
              configuration.maximumTurnMotorCommand, configuration.outputInverted}
{
}

void DifferentialSpeedController::reset()
{
  _state = {};
  _hasMeasurement = false;
}

float DifferentialSpeedController::update(float targetDifferentialSpeedMps, float leftSpeedMps,
                                          float rightSpeedMps, float deltaSeconds,
                                          float maximumTurnMotorCommandOverride)
{
  if (deltaSeconds <= 0.0F)
  {
    return _state.turnMotorCommand;
  }

  const float measuredDifferentialSpeedMps = rightSpeedMps - leftSpeedMps;
  if (!_hasMeasurement)
  {
    _state.filteredDifferentialSpeedMps = measuredDifferentialSpeedMps;
    _hasMeasurement = true;
  }
  else
  {
    _state.filteredDifferentialSpeedMps = _configuration.measurementFilterAlpha * measuredDifferentialSpeedMps +
                                          (1.0F - _configuration.measurementFilterAlpha) *
                                              _state.filteredDifferentialSpeedMps;
  }

  _state.differentialSpeedErrorMps =
      targetDifferentialSpeedMps - _state.filteredDifferentialSpeedMps;

  // Integrate only when the candidate output does not push farther into
  // saturation. This prevents a queued turn command after a sharp disturbance.
  const float candidateIntegral = clamp(
      _state.integralMpsSeconds + _state.differentialSpeedErrorMps * deltaSeconds,
      _configuration.integralLimit);
  const float proportionalTerm = _tuning.proportionalGain * _state.differentialSpeedErrorMps;
  const float candidateOutput = proportionalTerm + _tuning.integralGain * candidateIntegral;
  const float outputLimit = maximumTurnMotorCommandOverride < 0.0F
                                ? _tuning.maximumTurnMotorCommand
                                : clamp(maximumTurnMotorCommandOverride,
                                        kAbsoluteMaximumTurnMotorCommand);
  const bool saturatedHigh = candidateOutput > outputLimit && _state.differentialSpeedErrorMps > 0.0F;
  const bool saturatedLow = candidateOutput < -outputLimit && _state.differentialSpeedErrorMps < 0.0F;
  if (!saturatedHigh && !saturatedLow)
  {
    _state.integralMpsSeconds = candidateIntegral;
  }

  float turnMotorCommand = proportionalTerm + _tuning.integralGain * _state.integralMpsSeconds;
  if (_tuning.outputInverted)
  {
    turnMotorCommand = -turnMotorCommand;
  }
  _state.turnMotorCommandRaw = turnMotorCommand;
  _state.turnMotorCommand = clamp(turnMotorCommand, outputLimit);
  return _state.turnMotorCommand;
}

void DifferentialSpeedController::setProportionalGain(float gain)
{
  _tuning.proportionalGain = gain < 0.0F ? 0.0F : gain;
}

void DifferentialSpeedController::setIntegralGain(float gain)
{
  _tuning.integralGain = gain < 0.0F ? 0.0F : gain;
}

void DifferentialSpeedController::setMaximumTurnMotorCommand(float maximumTurnMotorCommand)
{
  _tuning.maximumTurnMotorCommand = clamp(maximumTurnMotorCommand, kAbsoluteMaximumTurnMotorCommand);
}

void DifferentialSpeedController::setOutputInverted(bool inverted)
{
  _tuning.outputInverted = inverted;
}

DifferentialSpeedTuning DifferentialSpeedController::tuning() const
{
  return _tuning;
}

const DifferentialSpeedState &DifferentialSpeedController::state() const
{
  return _state;
}

float DifferentialSpeedController::clamp(float value, float limit)
{
  if (limit <= 0.0F)
  {
    return 0.0F;
  }
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}
} // namespace balance_car::control
