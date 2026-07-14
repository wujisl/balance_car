#include "control/attitude_estimator.h"

#include <math.h>

namespace balance_car::control
{
AttitudeEstimator::AttitudeEstimator(const config::AttitudeConfiguration &configuration)
    : _configuration(configuration)
{
}

void AttitudeEstimator::reset()
{
  _state = {};
}

AttitudeState AttitudeEstimator::update(const drivers::ImuSample &sample, bool useAccelerometerCorrection)
{
  if (!sample.valid)
  {
    AttitudeState invalidState = _state;
    invalidState.valid = false;
    return invalidState;
  }

  float accelerometerPitchDegrees = 0.0F;
  float pitchRateDps = 0.0F;
  switch (_configuration.pitchAxis)
  {
  case config::AttitudeConfiguration::PitchAxis::X:
    accelerometerPitchDegrees = atan2f(sample.accelYG, sample.accelZG) * RAD_TO_DEG;
    pitchRateDps = sample.gyroXDps;
    break;
  case config::AttitudeConfiguration::PitchAxis::Y:
    accelerometerPitchDegrees = atan2f(-sample.accelXG, sample.accelZG) * RAD_TO_DEG;
    pitchRateDps = sample.gyroYDps;
    break;
  }
  accelerometerPitchDegrees += _configuration.accelerometerAngleOffsetDegrees;
  if (_configuration.pitchAngleInverted)
  {
    accelerometerPitchDegrees = -accelerometerPitchDegrees;
  }

  if (_configuration.pitchGyroInverted)
  {
    pitchRateDps = -pitchRateDps;
  }

  const uint32_t elapsedMs = _state.valid ? sample.timestampMs - _state.timestampMs : 0;
  const float deltaSeconds = static_cast<float>(elapsedMs) / 1000.0F;
  if (!_state.valid || elapsedMs == 0 || elapsedMs > 100)
  {
    _state.pitchDegrees = accelerometerPitchDegrees;
  }
  else
  {
    const float predictedPitchDegrees = _state.pitchDegrees + pitchRateDps * deltaSeconds;
    if (!useAccelerometerCorrection)
    {
      _state.pitchDegrees = predictedPitchDegrees;
    }
    else
    {
      const float filterAlpha = _configuration.complementaryFilterTimeConstantSeconds /
                                (_configuration.complementaryFilterTimeConstantSeconds + deltaSeconds);
      _state.pitchDegrees = filterAlpha * predictedPitchDegrees +
                            (1.0F - filterAlpha) * accelerometerPitchDegrees;
    }
  }

  _state.pitchRateDps = pitchRateDps;
  _state.accelerometerPitchDegrees = accelerometerPitchDegrees;
  _state.timestampMs = sample.timestampMs;
  _state.valid = true;
  return _state;
}

const AttitudeState &AttitudeEstimator::state() const
{
  return _state;
}
} // namespace balance_car::control
