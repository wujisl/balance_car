#pragma once

#include <Arduino.h>

namespace balance_car::config
{
  struct WifiDebugConfiguration
  {
    const char *ssid;
    const char *password;
    uint8_t channel;
    uint16_t telemetryPort;
    uint16_t commandPort;
    uint16_t telemetryPeriodMs;
  };

  // This AP is only for bench tuning. Change these values here if another
  // vehicle is powered nearby.
  constexpr WifiDebugConfiguration kWifiDebugConfiguration = {
      .ssid = "BALANCECAR_B10",
      .password = "balance123",
      .channel = 1,
      .telemetryPort = 9000,
      .commandPort = 9001,
      .telemetryPeriodMs = 20,
  };
} // namespace balance_car::config
