#pragma once

#include "config/vehicle_config.h"
#include "drivers/imu_driver.h"

#include <Arduino.h>

namespace balance_car::app
{
enum class AirborneLandingState : uint8_t
{
  Grounded,
  Airborne,
  LandingSettling,
  Recovering,
  Fault,
};

enum class AirborneLandingEvent : uint8_t
{
  None,
  EnteredAirborne,
  ResetAttitude,
  RecoveryComplete,
  Fault,
};

// Protects the control loops when the wheels have no usable ground contact.
// It deliberately does not claim to make a high drop safe: it prevents the
// ground-control loops from acting on free-fall accelerometer data and only
// resumes output after a stationary-gravity observation.
class AirborneLandingManager
{
public:
  explicit AirborneLandingManager(const config::AirborneLandingConfiguration &configuration);

  void reset();
  AirborneLandingEvent update(const drivers::ImuSample &sample, uint32_t nowMs);
  bool isEnabled() const;
  bool useAccelerometerCorrection() const;
  bool enforcePitchLimit() const;
  bool holdMotorOutput() const;
  bool allowMotionControl() const;
  float motorOutputScale(uint32_t nowMs) const;
  AirborneLandingState state() const;
  static const char *stateName(AirborneLandingState state);

private:
  bool accelerationInLandingBand(const drivers::ImuSample &sample) const;
  static float accelerationMagnitudeG(const drivers::ImuSample &sample);

  const config::AirborneLandingConfiguration &_configuration;
  AirborneLandingState _state = AirborneLandingState::Grounded;
  uint32_t _lowAccelerationStartedAtMs = 0;
  uint32_t _airborneStartedAtMs = 0;
  uint32_t _landingDetectedAtMs = 0;
  uint32_t _landingStableStartedAtMs = 0;
  uint32_t _recoveryStartedAtMs = 0;
};
} // namespace balance_car::app
