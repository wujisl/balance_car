#include "app/airborne_landing_manager.h"

#include <math.h>

namespace balance_car::app
{
AirborneLandingManager::AirborneLandingManager(
    const config::AirborneLandingConfiguration &configuration)
    : _tuning()
{
  _tuning.enabled = configuration.enabled;
  _tuning.airborneAccelerationThresholdG = configuration.airborneAccelerationThresholdG;
  _tuning.airborneConfirmationMs = configuration.airborneConfirmationMs;
  _tuning.maximumAirborneMs = configuration.maximumAirborneMs;
  _tuning.landingAccelerationMinimumG = configuration.landingAccelerationMinimumG;
  _tuning.landingAccelerationMaximumG = configuration.landingAccelerationMaximumG;
  _tuning.landingSettleMs = configuration.landingSettleMs;
  _tuning.landingRecoveryTimeoutMs = configuration.landingRecoveryTimeoutMs;
  _tuning.motorRecoveryRampMs = configuration.motorRecoveryRampMs;
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
  if (!_tuning.enabled || !sample.valid)
  {
    return AirborneLandingEvent::None;
  }

  const float accelerationG = accelerationMagnitudeG(sample);
  switch (_state)
  {
  case AirborneLandingState::Grounded:
    if (accelerationG < _tuning.airborneAccelerationThresholdG)
    {
      if (_lowAccelerationStartedAtMs == 0)
      {
        _lowAccelerationStartedAtMs = nowMs;
      }
      if (nowMs - _lowAccelerationStartedAtMs >= _tuning.airborneConfirmationMs)
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
    if (nowMs - _airborneStartedAtMs > _tuning.maximumAirborneMs)
    {
      _state = AirborneLandingState::Fault;
      return AirborneLandingEvent::Fault;
    }
    if (accelerationG >= fminf(_tuning.landingAccelerationMinimumG,
                                _tuning.landingAccelerationMaximumG))
    {
      _state = AirborneLandingState::LandingSettling;
      _landingDetectedAtMs = nowMs;
      _landingStableStartedAtMs = accelerationInLandingBand(sample) ? nowMs : 0;
    }
    break;

  case AirborneLandingState::LandingSettling:
    if (nowMs - _landingDetectedAtMs > _tuning.landingRecoveryTimeoutMs)
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
    if (nowMs - _landingStableStartedAtMs >= _tuning.landingSettleMs)
    {
      _state = AirborneLandingState::Recovering;
      _recoveryStartedAtMs = nowMs;
      return AirborneLandingEvent::ResetAttitude;
    }
    break;

  case AirborneLandingState::Recovering:
    if (nowMs - _recoveryStartedAtMs >= _tuning.motorRecoveryRampMs)
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
  return _tuning.enabled;
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
  if (_state != AirborneLandingState::Recovering || _tuning.motorRecoveryRampMs == 0)
  {
    return 0.0F;
  }
  const float scale = static_cast<float>(nowMs - _recoveryStartedAtMs) /
                      static_cast<float>(_tuning.motorRecoveryRampMs);
  return scale > 1.0F ? 1.0F : scale;
}

AirborneLandingState AirborneLandingManager::state() const
{
  return _state;
}

AirborneLandingTuning AirborneLandingManager::tuning() const
{
  return _tuning;
}

void AirborneLandingManager::setEnabled(bool enabled)
{
  _tuning.enabled = enabled;
  reset();
}

void AirborneLandingManager::setAirborneAccelerationThresholdG(float thresholdG)
{
  _tuning.airborneAccelerationThresholdG = constrain(thresholdG, 0.05F, 0.95F);
}

void AirborneLandingManager::setAirborneConfirmationMs(uint16_t durationMs)
{
  _tuning.airborneConfirmationMs = durationMs > 100 ? 100 : durationMs;
}

void AirborneLandingManager::setMaximumAirborneMs(uint16_t durationMs)
{
  _tuning.maximumAirborneMs = constrain(durationMs, static_cast<uint16_t>(100), static_cast<uint16_t>(2000));
}

void AirborneLandingManager::setLandingAccelerationMinimumG(float accelerationG)
{
  _tuning.landingAccelerationMinimumG = constrain(accelerationG, 0.10F, 3.50F);
}

void AirborneLandingManager::setLandingAccelerationMaximumG(float accelerationG)
{
  _tuning.landingAccelerationMaximumG = constrain(accelerationG, 0.20F, 4.00F);
}

void AirborneLandingManager::setLandingSettleMs(uint16_t durationMs)
{
  _tuning.landingSettleMs = constrain(durationMs, static_cast<uint16_t>(10), static_cast<uint16_t>(500));
}

void AirborneLandingManager::setLandingRecoveryTimeoutMs(uint16_t durationMs)
{
  _tuning.landingRecoveryTimeoutMs = constrain(durationMs, static_cast<uint16_t>(100), static_cast<uint16_t>(3000));
}

void AirborneLandingManager::setMotorRecoveryRampMs(uint16_t durationMs)
{
  _tuning.motorRecoveryRampMs = durationMs > 1500 ? 1500 : durationMs;
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
  const float minimumG = fminf(_tuning.landingAccelerationMinimumG,
                               _tuning.landingAccelerationMaximumG);
  const float maximumG = fmaxf(_tuning.landingAccelerationMinimumG,
                               _tuning.landingAccelerationMaximumG);
  return accelerationG >= minimumG && accelerationG <= maximumG;
}

float AirborneLandingManager::accelerationMagnitudeG(const drivers::ImuSample &sample)
{
  return sqrtf(sample.accelXG * sample.accelXG + sample.accelYG * sample.accelYG +
               sample.accelZG * sample.accelZG);
}
} // namespace balance_car::app
