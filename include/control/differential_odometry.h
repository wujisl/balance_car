#pragma once

#include "config/vehicle_config.h"
#include "drivers/encoder_driver.h"

namespace balance_car::control
{
struct DifferentialOdometryState
{
  // Heading is relative to the most recent balance arm. Positive values mean
  // the right wheel has travelled farther than the left wheel.
  float headingDegrees = 0.0F;
  float yawRateDegreesPerSecond = 0.0F;
};

class DifferentialOdometry
{
public:
  explicit DifferentialOdometry(const config::OdometryConfiguration &configuration);

  void reset();
  void update(const drivers::WheelSpeed &wheelSpeed, float deltaSeconds);
  const DifferentialOdometryState &state() const;

private:
  static float wrapRadians(float angleRadians);

  const config::OdometryConfiguration &_configuration;
  DifferentialOdometryState _state;
  bool _hasYawRateMeasurement = false;
};
} // namespace balance_car::control
