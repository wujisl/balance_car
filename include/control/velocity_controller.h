#pragma once

#include "config/vehicle_config.h"

namespace balance_car::control
{
struct VelocityTuning
{
  float proportionalGain;
  float integralGain;
  float maximumPitchOffsetDegrees;
  bool outputInverted;
};

struct VelocityState
{
  float filteredSpeedMps = 0.0F;
  float speedErrorMps = 0.0F;
  float integralMpsSeconds = 0.0F;
  float pitchOffsetDegrees = 0.0F;
};

class VelocityController
{
public:
  explicit VelocityController(const config::VelocityConfiguration &configuration);

  void reset();
  float update(float targetSpeedMps, float measuredSpeedMps, float deltaSeconds);
  void setProportionalGain(float gain);
  void setIntegralGain(float gain);
  void setMaximumPitchOffsetDegrees(float maximumPitchOffsetDegrees);
  void setOutputInverted(bool inverted);
  VelocityTuning tuning() const;
  const VelocityState &state() const;

private:
  const config::VelocityConfiguration &_configuration;
  VelocityTuning _tuning;
  VelocityState _state;
  bool _hasMeasurement = false;
};
} // namespace balance_car::control
