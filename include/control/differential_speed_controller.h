#pragma once

#include "config/vehicle_config.h"

namespace balance_car::control
{
struct DifferentialSpeedTuning
{
  float proportionalGain;
  float integralGain;
  float maximumTurnMotorCommand;
  bool outputInverted;
};

struct DifferentialSpeedState
{
  float filteredDifferentialSpeedMps = 0.0F;
  float differentialSpeedErrorMps = 0.0F;
  float integralMpsSeconds = 0.0F;
  float turnMotorCommandRaw = 0.0F;
  float turnMotorCommand = 0.0F;
};

// Generates a steering motor command from the right-minus-left wheel-speed
// error. This closes the differential-speed loop while the balance controller
// remains responsible for common-mode wheel motion.
class DifferentialSpeedController
{
public:
  explicit DifferentialSpeedController(const config::DifferentialSpeedConfiguration &configuration);

  void reset();
  float update(float targetDifferentialSpeedMps, float leftSpeedMps, float rightSpeedMps,
               float deltaSeconds, float maximumTurnMotorCommandOverride = -1.0F);
  void setProportionalGain(float gain);
  void setIntegralGain(float gain);
  void setMaximumTurnMotorCommand(float maximumTurnMotorCommand);
  void setOutputInverted(bool inverted);
  DifferentialSpeedTuning tuning() const;
  const DifferentialSpeedState &state() const;

private:
  static float clamp(float value, float limit);

  const config::DifferentialSpeedConfiguration &_configuration;
  DifferentialSpeedTuning _tuning;
  DifferentialSpeedState _state;
  bool _hasMeasurement = false;
};
} // namespace balance_car::control
