#include "control/motor_mixer.h"

namespace balance_car::control
{
namespace
{
float clamp(float value, float limit)
{
  if (limit <= 0.0F)
  {
    return 0.0F;
  }
  if (value > limit)
  {
    return limit;
  }
  if (value < -limit)
  {
    return -limit;
  }
  return value;
}
} // namespace

MixedMotorCommand MotorMixer::mix(float balanceCommand, float turnCommand, float maximumMotorCommand)
{
  MixedMotorCommand mixedCommand;
  const float outputLimit = clamp(maximumMotorCommand, 1.0F);
  mixedCommand.appliedBalanceCommand = clamp(balanceCommand, outputLimit);
  mixedCommand.requestedTurnCommand = turnCommand;

  // Keep the common-mode balance output intact. The remaining headroom is
  // symmetric for a differential command and guarantees both motor outputs
  // remain inside the requested final command limit.
  const float availableTurnMagnitude = outputLimit -
                                       (mixedCommand.appliedBalanceCommand < 0.0F
                                            ? -mixedCommand.appliedBalanceCommand
                                            : mixedCommand.appliedBalanceCommand);
  mixedCommand.appliedTurnCommand = clamp(turnCommand, availableTurnMagnitude);
  mixedCommand.left = mixedCommand.appliedBalanceCommand - mixedCommand.appliedTurnCommand;
  mixedCommand.right = mixedCommand.appliedBalanceCommand + mixedCommand.appliedTurnCommand;
  return mixedCommand;
}
} // namespace balance_car::control
