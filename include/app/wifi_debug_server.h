#pragma once

#include "config/wifi_debug_config.h"

#include <Arduino.h>
#include <IPAddress.h>
#include <WiFiUdp.h>

namespace balance_car::app
{
enum class WifiCommandKind : uint8_t
{
  Tuning,
  Arm,
  Stop,
  Reset,
  Drive,
  Turn,
  Track,
};

struct WifiTuningCommand
{
  uint32_t requestSequence = 0;
  WifiCommandKind kind = WifiCommandKind::Tuning;
  char domain[16] = {};
  char parameter[16] = {};
  float value = 0.0F;
};

struct WifiTelemetry
{
  uint32_t timestampMs = 0;
  uint8_t safetyState = 0;
  uint8_t faultCode = 0;
  bool imuValid = false;
  float pitchDegrees = 0.0F;
  float pitchRateDps = 0.0F;
  float accelerometerPitchDegrees = 0.0F;
  float accelXG = 0.0F;
  float accelYG = 0.0F;
  float accelZG = 0.0F;
  float gyroXDps = 0.0F;
  float gyroYDps = 0.0F;
  float gyroZDps = 0.0F;
  float targetSpeedMps = 0.0F;
  float filteredSpeedMps = 0.0F;
  float speedErrorMps = 0.0F;
  float speedPitchOffsetDegrees = 0.0F;
  // Target right-minus-left wheel speed, in m/s.
  float turnCommand = 0.0F;
  float filteredDifferentialSpeedMps = 0.0F;
  float differentialSpeedErrorMps = 0.0F;
  float turnMotorCommand = 0.0F;
  float appliedTurnMotorCommand = 0.0F;
  float leftMotorCommand = 0.0F;
  float rightMotorCommand = 0.0F;
  float leftWheelSpeedMps = 0.0F;
  float rightWheelSpeedMps = 0.0F;
  float requestedPitchDegrees = 0.0F;
  float balancePitchErrorDegrees = 0.0F;
  float balanceProportionalTerm = 0.0F;
  float balanceIntegralTerm = 0.0F;
  float balanceDerivativeTerm = 0.0F;
  float balanceMotorRaw = 0.0F;
  float balanceKp = 0.0F;
  float balanceKi = 0.0F;
  float balanceKd = 0.0F;
  float balanceTrimDegrees = 0.0F;
  float speedKp = 0.0F;
  float speedKi = 0.0F;
  bool speedInverted = false;
  float turnKp = 0.0F;
  float turnKi = 0.0F;
  float maximumTurnMotorCommand = 0.0F;
  bool turnInverted = false;
  float maximumMotorCommand = 0.0F;
  float maximumPitchOffsetDegrees = 0.0F;
  float headingDegrees = 0.0F;
  float yawRateDegreesPerSecond = 0.0F;
  bool visionTrackingEnabled = false;
  bool visionSampleFresh = false;
  bool visionCommandAccepted = false;
  float visionDeltaSpeedMps = 0.0F;
  uint16_t visionTargetUpdatePeriodMs = 0;
  bool visionTargetFilterEnabled = false;
  uint16_t visionTargetMaximumStepMmps = 0;
};

// Wi-Fi transport only: it has no knowledge of motors, arming, or control laws.
class WifiDebugServer
{
public:
  explicit WifiDebugServer(const config::WifiDebugConfiguration &configuration);

  void begin();
  void service();
  void writeConsoleByte(uint8_t byte);
  void writeConsoleBytes(const uint8_t *data, size_t length);
  bool takeTuningCommand(WifiTuningCommand &command);
  void sendCommandResult(uint32_t requestSequence, bool accepted, const char *reason);
  void publish(const WifiTelemetry &telemetry, uint32_t nowMs);

private:
  static constexpr size_t kConsoleLineCapacity = 192;
  static constexpr size_t kConsoleHistoryDepth = 32;

  bool parseCommand(char *packet, size_t length);
  void finishConsoleLine();
  void sendConsoleLine(const char *line);
  void sendConsoleHistory();
  void sendReply(uint32_t requestSequence, const char *status, const char *reason);

  const config::WifiDebugConfiguration &_configuration;
  WiFiUDP _udp;
  WifiTuningCommand _pendingCommand;
  IPAddress _replyIp;
  IPAddress _telemetryIp;
  uint16_t _replyPort = 0;
  uint16_t _telemetryPort = 0;
  uint32_t _telemetrySequence = 0;
  uint32_t _consoleSequence = 0;
  uint32_t _lastTelemetryMs = 0;
  uint32_t _lastTelemetryDiagnosticsMs = 0;
  uint32_t _lastSubscriptionDiagnosticsMs = 0;
  uint32_t _subscriptionCount = 0;
  uint16_t _telemetryPacketsSinceDiagnostics = 0;
  uint16_t _telemetryFailuresSinceDiagnostics = 0;
  char _consoleLine[kConsoleLineCapacity] = {};
  char _consoleHistory[kConsoleHistoryDepth][kConsoleLineCapacity] = {};
  size_t _consoleLineLength = 0;
  uint8_t _consoleHistoryCount = 0;
  uint8_t _consoleHistoryNext = 0;
  bool _hasPendingCommand = false;
  bool _started = false;
};
} // namespace balance_car::app
