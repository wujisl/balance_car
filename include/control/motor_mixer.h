#pragma once

namespace balance_car::control
{
struct MixedMotorCommand
{
  float left = 0.0F;
  float right = 0.0F;
  float appliedBalanceCommand = 0.0F;
  float requestedTurnCommand = 0.0F;
  float appliedTurnCommand = 0.0F;
};

class MotorMixer
{
public:
  // The balance command has priority. Steering only consumes the output
  // headroom that remains after applying it, so one saturated wheel cannot
  // silently attenuate the balance correction on both wheels.
  static MixedMotorCommand mix(float balanceCommand, float turnCommand,
                               float maximumMotorCommand = 1.0F);
};
} // namespace balance_car::control
