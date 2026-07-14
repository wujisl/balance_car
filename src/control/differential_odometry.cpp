#include "control/differential_odometry.h"

#include <math.h>

namespace balance_car::control
{
namespace
{
constexpr float kRadiansToDegrees = 180.0F / PI;
}

DifferentialOdometry::DifferentialOdometry(const config::OdometryConfiguration &configuration)
    : _configuration(configuration)
{
}

void DifferentialOdometry::reset()
{
  _state = {};
  _hasYawRateMeasurement = false;
}

void DifferentialOdometry::update(const drivers::WheelSpeed &wheelSpeed, float deltaSeconds)
{
  if (deltaSeconds <= 0.0F || _configuration.wheelTrackMeters <= 0.0F)
  {
    return;
  }

  // These distances originate from the raw encoder tick deltas in the same
  // control interval, so the heading integral does not depend on Wi-Fi packet
  // timing or the velocity-controller filter.
  const float leftDistanceMeters = wheelSpeed.leftMetersPerSecond * deltaSeconds;
  const float rightDistanceMeters = wheelSpeed.rightMetersPerSecond * deltaSeconds;
  const float headingDeltaRadians =
      (rightDistanceMeters - leftDistanceMeters) / _configuration.wheelTrackMeters;
  const float headingRadians = wrapRadians(_state.headingDegrees / kRadiansToDegrees + headingDeltaRadians);
  _state.headingDegrees = headingRadians * kRadiansToDegrees;

  const float measuredYawRateDegreesPerSecond =
      headingDeltaRadians * kRadiansToDegrees / deltaSeconds;
  if (!_hasYawRateMeasurement)
  {
    _state.yawRateDegreesPerSecond = measuredYawRateDegreesPerSecond;
    _hasYawRateMeasurement = true;
    return;
  }

  const float alpha = _configuration.yawRateFilterAlpha < 0.0F
                          ? 0.0F
                          : (_configuration.yawRateFilterAlpha > 1.0F ? 1.0F
                                                                         : _configuration.yawRateFilterAlpha);
  _state.yawRateDegreesPerSecond = alpha * measuredYawRateDegreesPerSecond +
                                   (1.0F - alpha) * _state.yawRateDegreesPerSecond;
}

const DifferentialOdometryState &DifferentialOdometry::state() const
{
  return _state;
}

float DifferentialOdometry::wrapRadians(float angleRadians)
{
  while (angleRadians > PI)
  {
    angleRadians -= 2.0F * PI;
  }
  while (angleRadians <= -PI)
  {
    angleRadians += 2.0F * PI;
  }
  return angleRadians;
}
} // namespace balance_car::control
