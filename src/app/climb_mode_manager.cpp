#include "app/climb_mode_manager.h"

#include <math.h>

namespace balance_car::app
{
namespace
{
constexpr float kAbsoluteMaximumPitchOffsetDegrees = 15.0F;
constexpr float kAbsoluteMaximumMotorCommand = 1.0F;
constexpr float kStateEpsilonDegrees = 0.01F;
}

ClimbModeManager::ClimbModeManager(const config::ClimbModeConfiguration &configuration)
    : _configuration(configuration)
{
  _tuning.forwardPitchFeedforwardDegrees = configuration.forwardPitchFeedforwardDegrees;
  _tuning.speedIntegralGain = configuration.speedIntegralGain;
  _tuning.maximumPitchOffsetDegrees = configuration.maximumPitchOffsetDegrees;
  _tuning.maximumMotorCommand = configuration.maximumMotorCommand;
  _tuning.maximumTargetSpeedMps = configuration.maximumTargetSpeedMps;
  _tuning.maximumDifferentialSpeedMps = configuration.maximumDifferentialSpeedMps;
  _tuning.maximumTurnMotorCommand = configuration.maximumTurnMotorCommand;
  _tuning.outputInverted = configuration.outputInverted;
}

void ClimbModeManager::reset()
{
  _state = ClimbModeState::Normal;
  _requested = false;
  _forwardAssist = false;
  _feedforwardPitchDegrees = 0.0F;
  _integralPitchDegrees = 0.0F;
}

void ClimbModeManager::setEnabled(bool enabled)
{
  _requested = enabled;
  if (!enabled)
  {
    _forwardAssist = false;
  }
  updateState();
}

bool ClimbModeManager::isRequested() const
{
  return _requested;
}

ClimbModeState ClimbModeManager::state() const
{
  return _state;
}

ClimbModeOutput ClimbModeManager::update(float targetSpeedMps, float measuredSpeedMps,
                                         float normalVelocityPitchOffsetDegrees, float deltaSeconds)
{
  if (deltaSeconds > 0.0F)
  {
    // Reverse motion must never retain an uphill-forward bias.  A requested
    // mode remains armed for a later forward command, but its output ramps out.
    _forwardAssist = _requested && targetSpeedMps >= 0.0F;
    const float desiredFeedforward = _forwardAssist ? _tuning.forwardPitchFeedforwardDegrees : 0.0F;
    const float maximumFeedforwardDelta =
        _configuration.pitchFeedforwardSlewRateDegreesPerSecond * deltaSeconds;
    _feedforwardPitchDegrees =
        approach(_feedforwardPitchDegrees, desiredFeedforward, maximumFeedforwardDelta);

    if (_forwardAssist)
    {
      const float speedErrorMps = targetSpeedMps - measuredSpeedMps;
      float integralDelta = _tuning.speedIntegralGain * speedErrorMps * deltaSeconds;
      if (_tuning.outputInverted)
      {
        integralDelta = -integralDelta;
      }

      const float candidateIntegral =
          clamp(_integralPitchDegrees + integralDelta, _configuration.maximumIntegralPitchDegrees);
      const float candidatePitchOffset = normalVelocityPitchOffsetDegrees + _feedforwardPitchDegrees +
                                         candidateIntegral;
      const bool pushesFurtherIntoPositiveLimit =
          candidatePitchOffset > _tuning.maximumPitchOffsetDegrees && integralDelta > 0.0F;
      const bool pushesFurtherIntoNegativeLimit =
          candidatePitchOffset < -_tuning.maximumPitchOffsetDegrees && integralDelta < 0.0F;
      if (!pushesFurtherIntoPositiveLimit && !pushesFurtherIntoNegativeLimit)
      {
        _integralPitchDegrees = candidateIntegral;
      }
    }
    else
    {
      _integralPitchDegrees = approach(
          _integralPitchDegrees, 0.0F,
          _configuration.integralUnwindRateDegreesPerSecond * deltaSeconds);
    }
  }

  const float pitchOffsetDegrees =
      clamp(normalVelocityPitchOffsetDegrees + _feedforwardPitchDegrees + _integralPitchDegrees,
            _tuning.maximumPitchOffsetDegrees);
  updateState();
  const bool active = _state != ClimbModeState::Normal;
  ClimbModeOutput output;
  output.requested = _requested;
  output.active = active;
  output.feedforwardPitchDegrees = _feedforwardPitchDegrees;
  output.integralPitchDegrees = _integralPitchDegrees;
  output.pitchOffsetDegrees = pitchOffsetDegrees;
  output.maximumMotorCommand = active ? _tuning.maximumMotorCommand : 0.0F;
  output.maximumTurnMotorCommand = active ? _tuning.maximumTurnMotorCommand : 0.0F;
  return output;
}

float ClimbModeManager::limitTargetSpeedMps(float requestedSpeedMps) const
{
  if (!_requested || requestedSpeedMps <= _tuning.maximumTargetSpeedMps)
  {
    return requestedSpeedMps;
  }
  return _tuning.maximumTargetSpeedMps;
}

float ClimbModeManager::limitDifferentialSpeedMps(float requestedDifferentialSpeedMps) const
{
  if (!_requested)
  {
    return requestedDifferentialSpeedMps;
  }
  return clamp(requestedDifferentialSpeedMps, _tuning.maximumDifferentialSpeedMps);
}

void ClimbModeManager::setForwardPitchFeedforwardDegrees(float degrees)
{
  _tuning.forwardPitchFeedforwardDegrees = clamp(degrees, kAbsoluteMaximumPitchOffsetDegrees);
}

void ClimbModeManager::setSpeedIntegralGain(float gain)
{
  _tuning.speedIntegralGain = gain < 0.0F ? 0.0F : gain;
}

void ClimbModeManager::setMaximumPitchOffsetDegrees(float degrees)
{
  _tuning.maximumPitchOffsetDegrees =
      clamp(degrees < 0.0F ? 0.0F : degrees, kAbsoluteMaximumPitchOffsetDegrees);
}

void ClimbModeManager::setMaximumMotorCommand(float command)
{
  _tuning.maximumMotorCommand = clamp(command < 0.0F ? 0.0F : command, kAbsoluteMaximumMotorCommand);
}

void ClimbModeManager::setMaximumTargetSpeedMps(float speedMps)
{
  _tuning.maximumTargetSpeedMps = speedMps < 0.0F ? 0.0F : speedMps;
}

void ClimbModeManager::setMaximumDifferentialSpeedMps(float speedMps)
{
  _tuning.maximumDifferentialSpeedMps = speedMps < 0.0F ? 0.0F : speedMps;
}

void ClimbModeManager::setMaximumTurnMotorCommand(float command)
{
  _tuning.maximumTurnMotorCommand =
      clamp(command < 0.0F ? 0.0F : command, kAbsoluteMaximumMotorCommand);
}

void ClimbModeManager::setOutputInverted(bool inverted)
{
  _tuning.outputInverted = inverted;
}

ClimbModeTuning ClimbModeManager::tuning() const
{
  return _tuning;
}

const char *ClimbModeManager::stateName(ClimbModeState state)
{
  switch (state)
  {
  case ClimbModeState::Normal:
    return "NORMAL";
  case ClimbModeState::Engaging:
    return "ENGAGING";
  case ClimbModeState::Active:
    return "ACTIVE";
  case ClimbModeState::Disengaging:
    return "DISENGAGING";
  }
  return "UNKNOWN";
}

float ClimbModeManager::clamp(float value, float limit)
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

float ClimbModeManager::approach(float current, float target, float maximumDelta)
{
  if (maximumDelta <= 0.0F)
  {
    return current;
  }
  if (current < target)
  {
    const float next = current + maximumDelta;
    return next > target ? target : next;
  }
  if (current > target)
  {
    const float next = current - maximumDelta;
    return next < target ? target : next;
  }
  return current;
}

void ClimbModeManager::updateState()
{
  if (_forwardAssist)
  {
    _state = fabsf(_feedforwardPitchDegrees - _tuning.forwardPitchFeedforwardDegrees) <=
                     kStateEpsilonDegrees
                 ? ClimbModeState::Active
                 : ClimbModeState::Engaging;
    return;
  }

  _state = (fabsf(_feedforwardPitchDegrees) <= kStateEpsilonDegrees &&
            fabsf(_integralPitchDegrees) <= kStateEpsilonDegrees)
               ? ClimbModeState::Normal
               : ClimbModeState::Disengaging;
}
} // namespace balance_car::app
