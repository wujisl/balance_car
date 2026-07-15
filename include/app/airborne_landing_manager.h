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

// Runtime copy of the landing-protection settings. It intentionally lives in
// the manager rather than mutating the compile-time vehicle configuration, so
// Wi-Fi test adjustments are discarded by a reboot.
struct AirborneLandingTuning
{
  bool enabled = false;
  float airborneAccelerationThresholdG = 0.35F;
  uint16_t airborneConfirmationMs = 20;
  uint16_t maximumAirborneMs = 500;
  float landingAccelerationMinimumG = 0.75F;
  float landingAccelerationMaximumG = 1.25F;
  uint16_t landingSettleMs = 60;
  uint16_t landingRecoveryTimeoutMs = 700;
  uint16_t motorRecoveryRampMs = 250;
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
  AirborneLandingTuning tuning() const;
  void setEnabled(bool enabled);
  void setAirborneAccelerationThresholdG(float thresholdG);
  void setAirborneConfirmationMs(uint16_t durationMs);
  void setMaximumAirborneMs(uint16_t durationMs);
  void setLandingAccelerationMinimumG(float accelerationG);
  void setLandingAccelerationMaximumG(float accelerationG);
  void setLandingSettleMs(uint16_t durationMs);
  void setLandingRecoveryTimeoutMs(uint16_t durationMs);
  void setMotorRecoveryRampMs(uint16_t durationMs);
  static const char *stateName(AirborneLandingState state);
  static float accelerationMagnitudeG(const drivers::ImuSample &sample);

private:
  bool accelerationInLandingBand(const drivers::ImuSample &sample) const;

  AirborneLandingTuning _tuning;
  AirborneLandingState _state = AirborneLandingState::Grounded;
  uint32_t _lowAccelerationStartedAtMs = 0;
  uint32_t _airborneStartedAtMs = 0;
  uint32_t _landingDetectedAtMs = 0;
  uint32_t _landingStableStartedAtMs = 0;
  uint32_t _recoveryStartedAtMs = 0;
};
} // namespace balance_car::app
