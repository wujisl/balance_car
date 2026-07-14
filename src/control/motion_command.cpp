#include "control/motion_command.h"

namespace balance_car::control
{
MotionCommand::MotionCommand(const config::MotionConfiguration &configuration) : _configuration(configuration)
{
}

void MotionCommand::setTargetSpeedMps(float targetSpeedMps)
{
  _targetSpeedMps = clampSpeed(targetSpeedMps);
}

void MotionCommand::adjustTargetSpeedMps(float deltaMps)
{
  setTargetSpeedMps(_targetSpeedMps + deltaMps);
}

void MotionCommand::setTurnCommand(float turnCommand)
{
  _turnCommand = clampTurn(turnCommand);
}

void MotionCommand::adjustTurnCommand(float deltaCommand)
{
  setTurnCommand(_turnCommand + deltaCommand);
}

void MotionCommand::setTargetDifferentialSpeedMps(float targetDifferentialSpeedMps)
{
  setTurnCommand(targetDifferentialSpeedMps);
}

void MotionCommand::adjustTargetDifferentialSpeedMps(float deltaMps)
{
  adjustTurnCommand(deltaMps);
}

void MotionCommand::clear()
{
  _targetSpeedMps = 0.0F;
  _turnCommand = 0.0F;
}

float MotionCommand::targetSpeedMps() const
{
  return _targetSpeedMps;
}

float MotionCommand::turnCommand() const
{
  return _turnCommand;
}

float MotionCommand::targetDifferentialSpeedMps() const
{
  return _turnCommand;
}

float MotionCommand::clampSpeed(float value) const
{
  if (value > _configuration.maximumTargetSpeedMps)
  {
    return _configuration.maximumTargetSpeedMps;
  }
  if (value < -_configuration.maximumTargetSpeedMps)
  {
    return -_configuration.maximumTargetSpeedMps;
  }
  return value;
}

float MotionCommand::clampTurn(float value) const
{
  if (value > _configuration.maximumTurnCommand)
  {
    return _configuration.maximumTurnCommand;
  }
  if (value < -_configuration.maximumTurnCommand)
  {
    return -_configuration.maximumTurnCommand;
  }
  return value;
}
} // namespace balance_car::control
