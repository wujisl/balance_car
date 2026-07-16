#include "control/velocity_controller.h"

namespace balance_car::control
{
namespace
{
constexpr float kAbsoluteMaximumPitchOffsetDegrees = 15.0F;
}

VelocityController::VelocityController(const config::VelocityConfiguration &configuration)
    : _configuration(configuration),
      _tuning{configuration.proportionalGain, configuration.integralGain,
              configuration.maximumPitchOffsetDegrees, configuration.outputInverted}
{
}

void VelocityController::reset()
{
  _state = {};
  _hasMeasurement = false;
}

float VelocityController::update(float targetSpeedMps, float measuredSpeedMps, float deltaSeconds)
{
  if (deltaSeconds <= 0.0F)
  {
    return _state.pitchOffsetDegrees;
  }

  if (!_hasMeasurement)
  {
    _state.filteredSpeedMps = measuredSpeedMps;
    _hasMeasurement = true;
  }
  else
  {
    _state.filteredSpeedMps = _configuration.measurementFilterAlpha * measuredSpeedMps +
                              (1.0F - _configuration.measurementFilterAlpha) * _state.filteredSpeedMps;
  }

  _state.speedErrorMps = targetSpeedMps - _state.filteredSpeedMps;
  float candidateIntegral = _state.integralMpsSeconds + _state.speedErrorMps * deltaSeconds;
  if (candidateIntegral > _configuration.integralLimit)
  {
    candidateIntegral = _configuration.integralLimit;
  }
  else if (candidateIntegral < -_configuration.integralLimit)
  {
    candidateIntegral = -_configuration.integralLimit;
  }

  const float proportionalTerm = _tuning.proportionalGain * _state.speedErrorMps;
  const float candidateOutput = proportionalTerm + _tuning.integralGain * candidateIntegral;
  const bool saturatedHigh = candidateOutput > _tuning.maximumPitchOffsetDegrees &&
                             _state.speedErrorMps > 0.0F;
  const bool saturatedLow = candidateOutput < -_tuning.maximumPitchOffsetDegrees &&
                            _state.speedErrorMps < 0.0F;
  if (!saturatedHigh && !saturatedLow)
  {
    _state.integralMpsSeconds = candidateIntegral;
  }

  const float logicalPitchOffsetDegrees = proportionalTerm +
                                          _tuning.integralGain * _state.integralMpsSeconds;
  float pitchOffsetDegrees = logicalPitchOffsetDegrees;
  if (_tuning.outputInverted)
  {
    pitchOffsetDegrees = -pitchOffsetDegrees;
  }
  _state.saturated = logicalPitchOffsetDegrees > _tuning.maximumPitchOffsetDegrees ||
                     logicalPitchOffsetDegrees < -_tuning.maximumPitchOffsetDegrees;
  if (pitchOffsetDegrees > _tuning.maximumPitchOffsetDegrees)
  {
    pitchOffsetDegrees = _tuning.maximumPitchOffsetDegrees;
  }
  else if (pitchOffsetDegrees < -_tuning.maximumPitchOffsetDegrees)
  {
    pitchOffsetDegrees = -_tuning.maximumPitchOffsetDegrees;
  }

  _state.pitchOffsetDegrees = pitchOffsetDegrees;
  return pitchOffsetDegrees;
}

void VelocityController::setProportionalGain(float gain)
{
  _tuning.proportionalGain = gain < 0.0F ? 0.0F : gain;
}

void VelocityController::setIntegralGain(float gain)
{
  _tuning.integralGain = gain < 0.0F ? 0.0F : gain;
}

void VelocityController::setMaximumPitchOffsetDegrees(float maximumPitchOffsetDegrees)
{
  if (maximumPitchOffsetDegrees < 0.0F)
  {
    _tuning.maximumPitchOffsetDegrees = 0.0F;
  }
  else if (maximumPitchOffsetDegrees > kAbsoluteMaximumPitchOffsetDegrees)
  {
    _tuning.maximumPitchOffsetDegrees = kAbsoluteMaximumPitchOffsetDegrees;
  }
  else
  {
    _tuning.maximumPitchOffsetDegrees = maximumPitchOffsetDegrees;
  }
}

void VelocityController::setOutputInverted(bool inverted)
{
  _tuning.outputInverted = inverted;
}

VelocityTuning VelocityController::tuning() const
{
  return _tuning;
}

const VelocityState &VelocityController::state() const
{
  return _state;
}
} // namespace balance_car::control
