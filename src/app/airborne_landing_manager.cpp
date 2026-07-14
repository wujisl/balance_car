#include "app/airborne_landing_manager.h"

#include <math.h>

namespace balance_car::app
{
AirborneLandingManager::AirborneLandingManager(
    const config::AirborneLandingConfiguration &configuration)
    : _configuration(configuration)
{
}

void AirborneLandingManager::reset()
{
  _state = AirborneLandingState::Grounded;
  _lowAccelerationStartedAtMs = 0;
  _airborneStartedAtMs = 0;
  _landingDetectedAtMs = 0;
  _landingStableStartedAtMs = 0;
  _recoveryStartedAtMs = 0;
}

AirborneLandingEvent AirborneLandingManager::update(const drivers::ImuSample &sample, uint32_t nowMs)
{
  if (!_configuration.enabled || !sample.valid)
  {
    return AirborneLandingEvent::None;
  }

  const float accelerationG = accelerationMagnitudeG(sample);
  switch (_state)
  {
  case AirborneLandingState::Grounded:
    if (accelerationG < _configuration.airborneAccelerationThresholdG)
    {
      if (_lowAccelerationStartedAtMs == 0)
      {
        _lowAccelerationStartedAtMs = nowMs;
      }
      if (nowMs - _lowAccelerationStartedAtMs >= _configuration.airborneConfirmationMs)
      {
        _state = AirborneLandingState::Airborne;
        _airborneStartedAtMs = nowMs;
        _lowAccelerationStartedAtMs = 0;
        return AirborneLandingEvent::EnteredAirborne;
      }
    }
    else
    {
      _lowAccelerationStartedAtMs = 0;
    }
    break;

  case AirborneLandingState::Airborne:
    if (nowMs - _airborneStartedAtMs > _configuration.maximumAirborneMs)
    {
      _state = AirborneLandingState::Fault;
      return AirborneLandingEvent::Fault;
    }
    if (accelerationG >= _configuration.landingAccelerationMinimumG)
    {
      _state = AirborneLandingState::LandingSettling;
      _landingDetectedAtMs = nowMs;
      _landingStableStartedAtMs = accelerationInLandingBand(sample) ? nowMs : 0;
    }
    break;

  case AirborneLandingState::LandingSettling:
    if (nowMs - _landingDetectedAtMs > _configuration.landingRecoveryTimeoutMs)
    {
      _state = AirborneLandingState::Fault;
      return AirborneLandingEvent::Fault;
    }
    if (!accelerationInLandingBand(sample))
    {
      _landingStableStartedAtMs = 0;
      break;
    }
    if (_landingStableStartedAtMs == 0)
    {
      _landingStableStartedAtMs = nowMs;
      break;
    }
    if (nowMs - _landingStableStartedAtMs >= _configuration.landingSettleMs)
    {
      _state = AirborneLandingState::Recovering;
      _recoveryStartedAtMs = nowMs;
      return AirborneLandingEvent::ResetAttitude;
    }
    break;

  case AirborneLandingState::Recovering:
    if (nowMs - _recoveryStartedAtMs >= _configuration.motorRecoveryRampMs)
    {
      _state = AirborneLandingState::Grounded;
      return AirborneLandingEvent::RecoveryComplete;
    }
    break;

  case AirborneLandingState::Fault:
    break;
  }
  return AirborneLandingEvent::None;
}

bool AirborneLandingManager::isEnabled() const
{
  return _configuration.enabled;
}

bool AirborneLandingManager::useAccelerometerCorrection() const
{
  return _state == AirborneLandingState::Grounded || _state == AirborneLandingState::Recovering;
}

bool AirborneLandingManager::enforcePitchLimit() const
{
  return _state == AirborneLandingState::Grounded || _state == AirborneLandingState::Recovering;
}

bool AirborneLandingManager::holdMotorOutput() const
{
  return _state == AirborneLandingState::Airborne || _state == AirborneLandingState::LandingSettling ||
         _state == AirborneLandingState::Fault;
}

bool AirborneLandingManager::allowMotionControl() const
{
  return _state == AirborneLandingState::Grounded;
}

float AirborneLandingManager::motorOutputScale(uint32_t nowMs) const
{
  if (_state == AirborneLandingState::Grounded)
  {
    return 1.0F;
  }
  if (_state != AirborneLandingState::Recovering || _configuration.motorRecoveryRampMs == 0)
  {
    return 0.0F;
  }
  const float scale = static_cast<float>(nowMs - _recoveryStartedAtMs) /
                      static_cast<float>(_configuration.motorRecoveryRampMs);
  return scale > 1.0F ? 1.0F : scale;
}

AirborneLandingState AirborneLandingManager::state() const
{
  return _state;
}

const char *AirborneLandingManager::stateName(AirborneLandingState state)
{
  switch (state)
  {
  case AirborneLandingState::Grounded:
    return "GROUNDED";
  case AirborneLandingState::Airborne:
    return "AIRBORNE";
  case AirborneLandingState::LandingSettling:
    return "LANDING_SETTLING";
  case AirborneLandingState::Recovering:
    return "RECOVERING";
  case AirborneLandingState::Fault:
    return "FAULT";
  }
  return "UNKNOWN";
}

bool AirborneLandingManager::accelerationInLandingBand(const drivers::ImuSample &sample) const
{
  const float accelerationG = accelerationMagnitudeG(sample);
  return accelerationG >= _configuration.landingAccelerationMinimumG &&
         accelerationG <= _configuration.landingAccelerationMaximumG;
}

float AirborneLandingManager::accelerationMagnitudeG(const drivers::ImuSample &sample)
{
  return sqrtf(sample.accelXG * sample.accelXG + sample.accelYG * sample.accelYG +
               sample.accelZG * sample.accelZG);
}
} // namespace balance_car::app
