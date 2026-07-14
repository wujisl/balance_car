#pragma once

#include "config/vehicle_config.h"

namespace balance_car::control
{
class MotionCommand
{
public:
  explicit MotionCommand(const config::MotionConfiguration &configuration);

  void setTargetSpeedMps(float targetSpeedMps);
  void adjustTargetSpeedMps(float deltaMps);
  void setTurnCommand(float turnCommand);
  void adjustTurnCommand(float deltaCommand);
  void setTargetDifferentialSpeedMps(float targetDifferentialSpeedMps);
  void adjustTargetDifferentialSpeedMps(float deltaMps);
  void clear();
  float targetSpeedMps() const;
  float turnCommand() const;
  float targetDifferentialSpeedMps() const;

private:
  float clampSpeed(float value) const;
  float clampTurn(float value) const;

  const config::MotionConfiguration &_configuration;
  float _targetSpeedMps = 0.0F;
  float _turnCommand = 0.0F;
};
} // namespace balance_car::control
