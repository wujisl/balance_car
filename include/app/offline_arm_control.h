#pragma once

#include "app/safety_manager.h"

#include <Arduino.h>

namespace balance_car::app
{
enum class OfflineArmEvent
{
  None,
  StartBalance,
  StopBalance,
};

class OfflineArmControl
{
public:
  OfflineArmControl(uint8_t buttonPin, uint16_t armHoldMs);

  void begin();
  OfflineArmEvent update(uint32_t nowMs, SafetyState safetyState);

private:
  bool isButtonPressed() const;

  uint8_t _buttonPin;
  uint16_t _armHoldMs;
  uint32_t _buttonPressedAtMs = 0;
  bool _buttonWasPressed = false;
  bool _awaitingInitialRelease = false;
  bool _startRequested = false;
  bool _wasInStandby = false;
};
} // namespace balance_car::app
