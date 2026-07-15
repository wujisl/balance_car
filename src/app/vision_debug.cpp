#include "app/vision_debug.h"

namespace balance_car::app
{
namespace
{
constexpr uint32_t kPrintPeriodMs = 500;
}

VisionDebug::VisionDebug(drivers::VisionI2cClient &client) : _client(client)
{
}

void VisionDebug::begin()
{
  Serial.println(_client.begin() ? "[I2C-INIT] vision master ready"
                                 : "[I2C-INIT] vision master init failed");
}

void VisionDebug::update(uint32_t nowMs)
{
  if (nowMs - _lastPrintMs < kPrintPeriodMs)
  {
    return;
  }

  _lastPrintMs = nowMs;
  // I2C v2 is exchanged by main.cpp together with chassis state; this legacy
  // helper deliberately does not issue a second read transaction.
  Serial.printf("[I2C] i2c=%s seq=%u valid=%u held=%u cal=%u dv=%d q=%u rows=%u miss=%u thr=%u\n",
                _client.isHealthy() ? "OK" : "ERR", _latest.sequence,
                _latest.trackValid ? 1U : 0U, _latest.held ? 1U : 0U,
                _latest.calibrated ? 1U : 0U, _latest.deltaSpeedTargetMmps,
                _latest.quality, _latest.validRows, _latest.missedFrames, _latest.thresholdUsed);
}
} // namespace balance_car::app
