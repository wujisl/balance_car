#include "app/offline_arm_control.h"

namespace balance_car::app
{
OfflineArmControl::OfflineArmControl(uint8_t buttonPin, uint16_t armHoldMs)
    : _buttonPin(buttonPin), _armHoldMs(armHoldMs)
{
}

void OfflineArmControl::begin()
{
  pinMode(_buttonPin, INPUT_PULLUP);
  _buttonWasPressed = isButtonPressed();
  _awaitingInitialRelease = _buttonWasPressed;
}

OfflineArmEvent OfflineArmControl::update(uint32_t nowMs, SafetyState safetyState)
{
  const bool buttonPressed = isButtonPressed();
  if (_awaitingInitialRelease)
  {
    if (!buttonPressed)
    {
      _awaitingInitialRelease = false;
      _buttonWasPressed = false;
    }
    return OfflineArmEvent::None;
  }

  if (safetyState == SafetyState::Balancing)
  {
    const bool stopRequested = buttonPressed && !_buttonWasPressed;
    _buttonWasPressed = buttonPressed;
    return stopRequested ? OfflineArmEvent::StopBalance : OfflineArmEvent::None;
  }

  if (safetyState != SafetyState::Standby)
  {
    _buttonPressedAtMs = 0;
    _startRequested = false;
    _buttonWasPressed = buttonPressed;
    _wasInStandby = false;
    return OfflineArmEvent::None;
  }

  // A BOOT press that began while the system was still starting is ignored;
  // require a fresh long press after the relaxed startup check completes.
  if (!_wasInStandby)
  {
    _wasInStandby = true;
    _buttonPressedAtMs = 0;
    _startRequested = false;
    _buttonWasPressed = buttonPressed;
    return OfflineArmEvent::None;
  }

  if (!buttonPressed)
  {
    _buttonPressedAtMs = 0;
    _startRequested = false;
    _buttonWasPressed = false;
    return OfflineArmEvent::None;
  }

  if (!_buttonWasPressed)
  {
    _buttonPressedAtMs = nowMs;
    _startRequested = false;
  }
  _buttonWasPressed = true;

  if (!_startRequested && nowMs - _buttonPressedAtMs >= _armHoldMs)
  {
    _startRequested = true;
    return OfflineArmEvent::StartBalance;
  }

  return OfflineArmEvent::None;
}

bool OfflineArmControl::isButtonPressed() const
{
  return digitalRead(_buttonPin) == LOW;
}
} // namespace balance_car::app
