#pragma once

#include <Arduino.h>
#include <Wire.h>
#include "config/board_pins.h"

namespace balance_car::drivers
{
struct VisionChassisState
{
  uint16_t sequence = 0;
  bool balancing = false;
  bool trackingEnabled = false;
  bool wheelSpeedValid = false;
  int16_t leftSpeedMmps = 0;
  int16_t rightSpeedMmps = 0;
  int16_t forwardTargetMmps = 0;
  uint32_t timestampMs = 0;
};

struct VisionSample
{
  uint16_t sequence = 0;
  uint16_t chassisSequence = 0;
  bool trackValid = false;
  bool held = false;
  bool calibrated = false;
  bool qualityOk = false;
  bool found = false;
  int16_t lateralErrorMm = 0;
  int16_t headingErrorCdeg = 0;
  int16_t curvatureMilliperM = 0;
  int16_t deltaSpeedTargetMmps = 0;
  uint8_t quality = 0;
  uint8_t validRows = 0;
  uint8_t missedFrames = 0;
  uint16_t lookaheadMm = 0;
  uint8_t thresholdUsed = 0;
  int16_t targetX = -1;
};

class VisionI2cClient
{
public:
  VisionI2cClient(TwoWire &wire, const config::VisionI2cPins &pins);
  bool begin();
  bool exchange(const VisionChassisState &state, VisionSample &sample);
  bool isHealthy() const;
  bool isFresh() const;

private:
  static constexpr uint8_t kAddress = 0x42;
  static constexpr uint32_t kFrequencyHz = 400000;
  TwoWire &_wire;
  const config::VisionI2cPins &_pins;
  bool _healthy = false;
  bool _fresh = false;
  uint16_t _lastVisionSequence = 0;
  uint32_t _lastVisionAdvanceMs = 0;
};
} // namespace balance_car::drivers
