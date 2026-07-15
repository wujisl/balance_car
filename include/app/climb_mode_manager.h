#pragma once

#include "config/vehicle_config.h"

#include <Arduino.h>

namespace balance_car::app
{
enum class ClimbModeState : uint8_t
{
  Normal,
  Engaging,
  Active,
  Disengaging,
};

struct ClimbModeTuning
{
  float forwardPitchFeedforwardDegrees = 0.0F;
  float speedIntegralGain = 0.0F;
  float maximumPitchOffsetDegrees = 0.0F;
  float maximumMotorCommand = 0.0F;
  float maximumTargetSpeedMps = 0.0F;
  float maximumDifferentialSpeedMps = 0.0F;
  float maximumTurnMotorCommand = 0.0F;
  bool outputInverted = false;
};

struct ClimbModeOutput
{
  bool requested = false;
  bool active = false;
  float feedforwardPitchDegrees = 0.0F;
  float integralPitchDegrees = 0.0F;
  float pitchOffsetDegrees = 0.0F;
  float maximumMotorCommand = 0.0F;
  float maximumTurnMotorCommand = 0.0F;
};

// Applies the known-grade compensation profile only after an explicit request.
// Deliberately does not try to infer a road slope from the IMU: that decision
// belongs to the mission/entry detector that will be added later.
class ClimbModeManager
{
public:
  explicit ClimbModeManager(const config::ClimbModeConfiguration &configuration);

  void reset();
  void setEnabled(bool enabled);
  bool isRequested() const;
  ClimbModeState state() const;
  ClimbModeOutput update(float targetSpeedMps, float measuredSpeedMps,
                         float normalVelocityPitchOffsetDegrees, float deltaSeconds);

  float limitTargetSpeedMps(float requestedSpeedMps) const;
  float limitDifferentialSpeedMps(float requestedDifferentialSpeedMps) const;

  void setForwardPitchFeedforwardDegrees(float degrees);
  void setSpeedIntegralGain(float gain);
  void setMaximumPitchOffsetDegrees(float degrees);
  void setMaximumMotorCommand(float command);
  void setMaximumTargetSpeedMps(float speedMps);
  void setMaximumDifferentialSpeedMps(float speedMps);
  void setMaximumTurnMotorCommand(float command);
  void setOutputInverted(bool inverted);
  ClimbModeTuning tuning() const;

  static const char *stateName(ClimbModeState state);

private:
  static float clamp(float value, float limit);
  static float approach(float current, float target, float maximumDelta);
  void updateState();

  const config::ClimbModeConfiguration &_configuration;
  ClimbModeTuning _tuning;
  ClimbModeState _state = ClimbModeState::Normal;
  bool _requested = false;
  bool _forwardAssist = false;
  float _feedforwardPitchDegrees = 0.0F;
  float _integralPitchDegrees = 0.0F;
};
} // namespace balance_car::app
