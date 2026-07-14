#pragma once

#include "config/vehicle_config.h"
#include "drivers/imu_driver.h"

namespace balance_car::control
{
struct AttitudeState
{
  float pitchDegrees = 0.0F;
  float pitchRateDps = 0.0F;
  float accelerometerPitchDegrees = 0.0F;
  uint32_t timestampMs = 0;
  bool valid = false;
};

class AttitudeEstimator
{
public:
  explicit AttitudeEstimator(const config::AttitudeConfiguration &configuration);

  void reset();
  AttitudeState update(const drivers::ImuSample &sample, bool useAccelerometerCorrection = true);
  const AttitudeState &state() const;

private:
  const config::AttitudeConfiguration &_configuration;
  AttitudeState _state;
};
} // namespace balance_car::control
