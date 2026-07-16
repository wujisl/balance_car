#include "app/wifi_debug_server.h"

#include <WiFi.h>

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace balance_car::app
{
namespace
{
constexpr size_t kCommandBufferCapacity = 128;
constexpr size_t kTelemetryBufferCapacity = 640;

bool isAllowedParameter(const char *domain, const char *parameter)
{
  return (strcmp(domain, "balance") == 0 &&
          (strcmp(parameter, "kp") == 0 || strcmp(parameter, "ki") == 0 ||
           strcmp(parameter, "kd") == 0 || strcmp(parameter, "trim") == 0 ||
           strcmp(parameter, "max_motor") == 0)) ||
          (strcmp(domain, "speed") == 0 &&
           (strcmp(parameter, "kp") == 0 || strcmp(parameter, "ki") == 0 ||
            strcmp(parameter, "max_pitch") == 0)) ||
          (strcmp(domain, "turn") == 0 &&
           (strcmp(parameter, "kp") == 0 || strcmp(parameter, "ki") == 0 ||
            strcmp(parameter, "max") == 0)) ||
          (strcmp(domain, "vision") == 0 &&
           (strcmp(parameter, "period_ms") == 0 || strcmp(parameter, "filter") == 0 ||
            strcmp(parameter, "max_step_mmps") == 0 ||
            strcmp(parameter, "curve_hold") == 0 ||
            strcmp(parameter, "curve_hold_mmps") == 0));
}

void toLowercase(char *text)
{
  for (; *text != '\0'; ++text)
  {
    *text = static_cast<char>(tolower(static_cast<unsigned char>(*text)));
  }
}
} // namespace

WifiDebugServer::WifiDebugServer(const config::WifiDebugConfiguration &configuration)
    : _configuration(configuration)
{
}

void WifiDebugServer::begin()
{
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  if (!WiFi.softAP(_configuration.ssid, _configuration.password, _configuration.channel))
  {
    Serial.println("[WIFI] AP_START=FAILED");
    return;
  }

  if (!_udp.begin(_configuration.commandPort))
  {
    Serial.println("[WIFI] UDP_BIND=FAILED");
    return;
  }

  _started = true;
  Serial.print("[WIFI] AP=READY SSID=");
  Serial.print(_configuration.ssid);
  Serial.print(" IP=");
  Serial.print(WiFi.softAPIP());
  Serial.print(" TELEMETRY_PORT=");
  Serial.print(_configuration.telemetryPort);
  Serial.print(" COMMAND_PORT=");
  Serial.println(_configuration.commandPort);
  char startupLine[kConsoleLineCapacity] = {};
  snprintf(startupLine, sizeof(startupLine), "[WIFI] AP=READY SSID=%s IP=%u.%u.%u.%u TELEMETRY_PORT=%u COMMAND_PORT=%u",
           _configuration.ssid, WiFi.softAPIP()[0], WiFi.softAPIP()[1], WiFi.softAPIP()[2], WiFi.softAPIP()[3],
           _configuration.telemetryPort, _configuration.commandPort);
  writeConsoleBytes(reinterpret_cast<const uint8_t *>(startupLine), strlen(startupLine));
  writeConsoleByte('\n');
  writeConsoleByte('\n');
}

void WifiDebugServer::service()
{
  if (!_started)
  {
    return;
  }

  const int packetSize = _udp.parsePacket();
  if (packetSize <= 0)
  {
    return;
  }

  const IPAddress senderIp = _udp.remoteIP();
  const uint16_t senderPort = _udp.remotePort();
  if (packetSize >= static_cast<int>(kCommandBufferCapacity))
  {
    while (_udp.available() > 0)
    {
      _udp.read();
    }
    _replyIp = senderIp;
    _replyPort = senderPort;
    sendReply(0, "ERR", "TOO_LONG");
    return;
  }

  char packet[kCommandBufferCapacity] = {};
  const int readSize = _udp.read(reinterpret_cast<uint8_t *>(packet), packetSize);
  if (readSize > 0)
  {
    _replyIp = senderIp;
    _replyPort = senderPort;
    parseCommand(packet, static_cast<size_t>(readSize));
  }
}

void WifiDebugServer::writeConsoleByte(uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }
  if (byte == '\n')
  {
    finishConsoleLine();
    return;
  }
  if (_consoleLineLength < kConsoleLineCapacity - 1)
  {
    _consoleLine[_consoleLineLength++] = static_cast<char>(byte);
  }
}

void WifiDebugServer::writeConsoleBytes(const uint8_t *data, size_t length)
{
  if (data == nullptr)
  {
    return;
  }
  for (size_t index = 0; index < length; ++index)
  {
    writeConsoleByte(data[index]);
  }
}

bool WifiDebugServer::takeTuningCommand(WifiTuningCommand &command)
{
  if (!_hasPendingCommand)
  {
    return false;
  }
  command = _pendingCommand;
  _hasPendingCommand = false;
  return true;
}

void WifiDebugServer::sendCommandResult(uint32_t requestSequence, bool accepted, const char *reason)
{
  sendReply(requestSequence, accepted ? "OK" : "ERR", reason);
}

void WifiDebugServer::publish(const WifiTelemetry &telemetry, uint32_t nowMs)
{
  if (!_started || nowMs - _lastTelemetryMs < _configuration.telemetryPeriodMs)
  {
    return;
  }

  _lastTelemetryMs = nowMs;
  char packet[kTelemetryBufferCapacity] = {};
  const uint32_t telemetrySequence = _telemetrySequence++;
  const int length = snprintf(
      packet, sizeof(packet),
      "T,10,%lu,%lu,%u,%u,%u,"
      "%.3f,%.3f,%.3f,"       // pitch, pitch rate, accelerometer pitch
      "%.3f,%.3f,%.3f,"       // accelerometer X/Y/Z
      "%.3f,%.3f,%.3f,"       // gyro X/Y/Z
      "%.3f,%.3f,%.3f,"       // target, filtered, error speed
      "%.3f,"                  // speed pitch offset
      "%.3f,%.3f,%.3f,%.3f,"  // target/filtered/error differential speed, turn command
      "%.3f,"                  // turn command after mixer headroom limit
      "%.3f,%.3f,"             // left/right motor command
      "%.5f,%.5f,%.5f,"       // balance Kp/Ki/Kd
      "%.3f,"                   // balance trim
      "%.5f,%.5f,"             // speed Kp/Ki
      "%.3f,%.3f,"              // maximum motor, maximum pitch offset
      "%.3f,%.3f,"              // left/right wheel speed
      "%.3f,%.3f,"              // requested pitch, balance pitch error
      "%.5f,%.5f,%.5f,"         // balance P/I/D terms
      "%.5f,%u,"               // raw balance motor command, speed output inverted
      "%.5f,%.5f,%.3f,%u,"    // turn Kp/Ki/max command, output inverted
      "%.3f,%.3f,"             // relative heading, filtered yaw rate
      "%u,%u,%u,%.3f,%u,%u,%u," // vision enabled, fresh sample, accepted, camera delta-v, update period/filter/max-step
      "%u,%u,%u,%u,%u,%u,%u\n", // saturation flags, encoder/IMU validity, actual loop periods
      static_cast<unsigned long>(telemetrySequence),
      static_cast<unsigned long>(telemetry.timestampMs),
      static_cast<unsigned int>(telemetry.safetyState),
      static_cast<unsigned int>(telemetry.faultCode),
      telemetry.imuValid ? 1U : 0U,
      telemetry.pitchDegrees, telemetry.pitchRateDps, telemetry.accelerometerPitchDegrees,
      telemetry.accelXG, telemetry.accelYG, telemetry.accelZG,
      telemetry.gyroXDps, telemetry.gyroYDps, telemetry.gyroZDps,
      telemetry.targetSpeedMps, telemetry.filteredSpeedMps, telemetry.speedErrorMps,
      telemetry.speedPitchOffsetDegrees,
      telemetry.turnCommand, telemetry.filteredDifferentialSpeedMps,
      telemetry.differentialSpeedErrorMps, telemetry.turnMotorCommand,
      telemetry.appliedTurnMotorCommand,
      telemetry.leftMotorCommand, telemetry.rightMotorCommand,
      telemetry.balanceKp, telemetry.balanceKi, telemetry.balanceKd,
      telemetry.balanceTrimDegrees, telemetry.speedKp, telemetry.speedKi,
      telemetry.maximumMotorCommand, telemetry.maximumPitchOffsetDegrees,
      telemetry.leftWheelSpeedMps, telemetry.rightWheelSpeedMps,
      telemetry.requestedPitchDegrees, telemetry.balancePitchErrorDegrees,
      telemetry.balanceProportionalTerm, telemetry.balanceIntegralTerm, telemetry.balanceDerivativeTerm,
      telemetry.balanceMotorRaw, telemetry.speedInverted ? 1U : 0U,
      telemetry.turnKp, telemetry.turnKi, telemetry.maximumTurnMotorCommand,
      telemetry.turnInverted ? 1U : 0U,
      telemetry.headingDegrees, telemetry.yawRateDegreesPerSecond,
      telemetry.visionTrackingEnabled ? 1U : 0U,
      telemetry.visionSampleFresh ? 1U : 0U,
      telemetry.visionCommandAccepted ? 1U : 0U,
      telemetry.visionDeltaSpeedMps,
      static_cast<unsigned int>(telemetry.visionTargetUpdatePeriodMs),
      telemetry.visionTargetFilterEnabled ? 1U : 0U,
      static_cast<unsigned int>(telemetry.visionTargetMaximumStepMmps),
      telemetry.balanceInnerSaturated ? 1U : 0U,
      telemetry.velocityLoopSaturated ? 1U : 0U,
      telemetry.turnLoopSaturated ? 1U : 0U,
      telemetry.encoderValid ? 1U : 0U,
      telemetry.imuCalibrated ? 1U : 0U,
      static_cast<unsigned int>(telemetry.balanceControlPeriodMs),
      static_cast<unsigned int>(telemetry.velocityControlPeriodMs));
  if (length <= 0 || length >= static_cast<int>(sizeof(packet)))
  {
    return;
  }

  const IPAddress targetIp = _telemetryPort != 0 ? _telemetryIp : WiFi.softAPBroadcastIP();
  const uint16_t targetPort = _telemetryPort != 0 ? _telemetryPort : _configuration.telemetryPort;
  _udp.beginPacket(targetIp, targetPort);
  _udp.write(reinterpret_cast<const uint8_t *>(packet), static_cast<size_t>(length));
  const bool sent = _udp.endPacket() != 0;
  if (sent)
  {
    ++_telemetryPacketsSinceDiagnostics;
  }
  else
  {
    ++_telemetryFailuresSinceDiagnostics;
  }

  // D,1 is deliberately a separate, compact diagnostics packet.  T,10 must
  // remain byte-for-byte compatible with the existing tuning hosts, whereas
  // characterization needs signed cumulative ticks and the 40 ms tick delta.
  char diagnosticsPacket[240] = {};
  const int diagnosticsLength = snprintf(
      diagnosticsPacket, sizeof(diagnosticsPacket),
      "D,1,%lu,%lu,%ld,%ld,%ld,%ld,%.3f,%.3f\n",
      static_cast<unsigned long>(telemetrySequence),
      static_cast<unsigned long>(telemetry.timestampMs),
      static_cast<long>(telemetry.leftEncoderTicks),
      static_cast<long>(telemetry.rightEncoderTicks),
      static_cast<long>(telemetry.leftEncoderTickDelta),
      static_cast<long>(telemetry.rightEncoderTickDelta),
      telemetry.leftEncoderTicksPerSecond, telemetry.rightEncoderTicksPerSecond);
  if (diagnosticsLength > 0 && diagnosticsLength < static_cast<int>(sizeof(diagnosticsPacket)))
  {
    _udp.beginPacket(targetIp, targetPort);
    _udp.write(reinterpret_cast<const uint8_t *>(diagnosticsPacket), static_cast<size_t>(diagnosticsLength));
    _udp.endPacket();
  }
  if (nowMs - _lastTelemetryDiagnosticsMs >= 1000)
  {
    _lastTelemetryDiagnosticsMs = nowMs;
    Serial.print("[WIFI] TELEMETRY_TX_OK=");
    Serial.print(_telemetryPacketsSinceDiagnostics);
    Serial.print(" FAIL=");
    Serial.print(_telemetryFailuresSinceDiagnostics);
    Serial.print(" SEQ=");
    Serial.print(_telemetrySequence - 1);
    Serial.print(" BYTES=");
    Serial.print(length);
    Serial.print(" TARGET=");
    Serial.print(targetIp);
    Serial.print(':');
    Serial.print(targetPort);
    Serial.print(" LAST_SEND=");
    Serial.print(sent ? "OK" : "FAILED");
    Serial.print(" SUBSCRIBED=");
    Serial.print(_telemetryPort != 0 ? 1 : 0);
    Serial.print(" STATIONS=");
    Serial.println(WiFi.softAPgetStationNum());
    _telemetryPacketsSinceDiagnostics = 0;
    _telemetryFailuresSinceDiagnostics = 0;
  }
}

bool WifiDebugServer::parseCommand(char *packet, size_t length)
{
  packet[length] = '\0';
  while (length > 0 && isspace(static_cast<unsigned char>(packet[length - 1])))
  {
    packet[--length] = '\0';
  }
  // Accept legacy host packets that accidentally ended with the two literal
  // characters "\\n" instead of a newline.
  if (length >= 2 && packet[length - 2] == '\\' && packet[length - 1] == 'n')
  {
    length -= 2;
    packet[length] = '\0';
  }
  char *context = nullptr;
  char *prefix = strtok_r(packet, ",", &context);
  char *sequenceText = strtok_r(nullptr, ",", &context);
  if (prefix != nullptr && strcmp(prefix, "H") == 0)
  {
    if (sequenceText != nullptr)
    {
      sendReply(0, "ERR", "FORMAT");
      return false;
    }
    const bool changed = _telemetryPort != _replyPort || _telemetryIp != _replyIp;
    _telemetryIp = _replyIp;
    _telemetryPort = _replyPort;
    ++_subscriptionCount;
    if (changed)
    {
      Serial.print("[WIFI] TELEMETRY_SUBSCRIBER=");
      Serial.print(_telemetryIp);
      Serial.print(':');
      Serial.println(_telemetryPort);
      sendConsoleHistory();
    }
    const uint32_t nowMs = millis();
    if (changed || nowMs - _lastSubscriptionDiagnosticsMs >= 2000)
    {
      _lastSubscriptionDiagnosticsMs = nowMs;
    }
    return false;
  }

  if (prefix == nullptr || sequenceText == nullptr)
  {
    sendReply(0, "ERR", "FORMAT");
    return false;
  }

  char *sequenceEnd = nullptr;
  const unsigned long sequence = strtoul(sequenceText, &sequenceEnd, 10);
  if (*sequenceText == '\0' || *sequenceEnd != '\0')
  {
    sendReply(0, "ERR", "INVALID_SEQUENCE");
    return false;
  }

  if (_hasPendingCommand)
  {
    sendReply(static_cast<uint32_t>(sequence), "ERR", "BUSY");
    return false;
  }

  if (strcmp(prefix, "C") == 0)
  {
    char *action = strtok_r(nullptr, ",", &context);
    if (action == nullptr)
    {
      sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
      return false;
    }
    toLowercase(action);
    _pendingCommand = {};
    _pendingCommand.requestSequence = static_cast<uint32_t>(sequence);
    if (strcmp(action, "arm") == 0)
    {
      if (strtok_r(nullptr, ",", &context) != nullptr)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::Arm;
    }
    else if (strcmp(action, "stop") == 0)
    {
      if (strtok_r(nullptr, ",", &context) != nullptr)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::Stop;
    }
    else if (strcmp(action, "reset") == 0)
    {
      if (strtok_r(nullptr, ",", &context) != nullptr)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::Reset;
    }
    else if (strcmp(action, "drive") == 0)
    {
      char *speedText = strtok_r(nullptr, ",", &context);
      if (speedText == nullptr || strtok_r(nullptr, ",", &context) != nullptr)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
        return false;
      }
      char *speedEnd = nullptr;
      const float speedMps = strtof(speedText, &speedEnd);
      if (*speedText == '\0' || *speedEnd != '\0' || !isfinite(speedMps))
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "INVALID_SPEED");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::Drive;
      _pendingCommand.value = speedMps;
    }
    else if (strcmp(action, "turn") == 0)
    {
      char *turnText = strtok_r(nullptr, ",", &context);
      if (turnText == nullptr || strtok_r(nullptr, ",", &context) != nullptr)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
        return false;
      }
      char *turnEnd = nullptr;
      const float turn = strtof(turnText, &turnEnd);
      if (*turnText == '\0' || *turnEnd != '\0' || !isfinite(turn) || fabsf(turn) > 0.20F)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "TURN_RANGE");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::Turn;
      _pendingCommand.value = turn;
    }
    else if (strcmp(action, "track") == 0)
    {
      char *enabledText = strtok_r(nullptr, ",", &context);
      if (enabledText == nullptr || strtok_r(nullptr, ",", &context) != nullptr ||
          (strcmp(enabledText, "0") != 0 && strcmp(enabledText, "1") != 0))
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "TRACK_VALUE");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::Track;
      _pendingCommand.value = strcmp(enabledText, "1") == 0 ? 1.0F : 0.0F;
    }
    else if (strcmp(action, "motor") == 0)
    {
      char *leftText = strtok_r(nullptr, ",", &context);
      char *rightText = strtok_r(nullptr, ",", &context);
      if (leftText == nullptr || rightText == nullptr || strtok_r(nullptr, ",", &context) != nullptr)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
        return false;
      }
      char *leftEnd = nullptr;
      char *rightEnd = nullptr;
      const float leftPower = strtof(leftText, &leftEnd);
      const float rightPower = strtof(rightText, &rightEnd);
      // This parser-side bound is repeated by SafetyManager.  Keeping both
      // makes malformed or future call sites unable to bypass the limit.
      if (*leftText == '\0' || *rightText == '\0' || *leftEnd != '\0' || *rightEnd != '\0' ||
          !isfinite(leftPower) || !isfinite(rightPower) || fabsf(leftPower) > 0.35F || fabsf(rightPower) > 0.35F)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "MOTOR_RANGE");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::MotorTest;
      _pendingCommand.value = leftPower;
      _pendingCommand.value2 = rightPower;
    }
    else if (strcmp(action, "imu_cal") == 0)
    {
      if (strtok_r(nullptr, ",", &context) != nullptr)
      {
        sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
        return false;
      }
      _pendingCommand.kind = WifiCommandKind::CalibrateImu;
    }
    else
    {
      sendReply(static_cast<uint32_t>(sequence), "ERR", "INVALID_ACTION");
      return false;
    }
    _hasPendingCommand = true;
    return true;
  }

  char *domain = strtok_r(nullptr, ",", &context);
  char *parameter = strtok_r(nullptr, ",", &context);
  char *valueText = strtok_r(nullptr, ",", &context);
  if (strcmp(prefix, "P") != 0 || domain == nullptr || parameter == nullptr || valueText == nullptr ||
      strtok_r(nullptr, ",", &context) != nullptr)
  {
    sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
    return false;
  }

  char *valueEnd = nullptr;
  const float value = strtof(valueText, &valueEnd);
  toLowercase(domain);
  toLowercase(parameter);
  if (*valueText == '\0' || *valueEnd != '\0' || !isfinite(value) ||
      !isAllowedParameter(domain, parameter))
  {
    sendReply(static_cast<uint32_t>(sequence), "ERR", "INVALID_PARAMETER");
    return false;
  }

  _pendingCommand = {};
  _pendingCommand.requestSequence = static_cast<uint32_t>(sequence);
  _pendingCommand.kind = WifiCommandKind::Tuning;
  strncpy(_pendingCommand.domain, domain, sizeof(_pendingCommand.domain) - 1);
  strncpy(_pendingCommand.parameter, parameter, sizeof(_pendingCommand.parameter) - 1);
  _pendingCommand.value = value;
  _hasPendingCommand = true;
  return true;
}

void WifiDebugServer::finishConsoleLine()
{
  if (_consoleLineLength == 0)
  {
    return;
  }
  _consoleLine[_consoleLineLength] = '\0';
  // The host's serial-mirror pane is reserved for compact tracking I2C
  // diagnostics.  IMU and Wi-Fi diagnostics continue on hardware Serial;
  // IMU data itself remains available through normal UDP telemetry/CSV.
  if (strncmp(_consoleLine, "[I2C]", 5) != 0)
  {
    _consoleLineLength = 0;
    _consoleLine[0] = '\0';
    return;
  }
  strncpy(_consoleHistory[_consoleHistoryNext], _consoleLine, kConsoleLineCapacity - 1);
  _consoleHistory[_consoleHistoryNext][kConsoleLineCapacity - 1] = '\0';
  _consoleHistoryNext = static_cast<uint8_t>((_consoleHistoryNext + 1) % kConsoleHistoryDepth);
  if (_consoleHistoryCount < kConsoleHistoryDepth)
  {
    ++_consoleHistoryCount;
  }
  sendConsoleLine(_consoleLine);
  _consoleLineLength = 0;
  _consoleLine[0] = '\0';
}

void WifiDebugServer::sendConsoleLine(const char *line)
{
  if (!_started || _telemetryPort == 0 || line == nullptr || line[0] == '\0')
  {
    return;
  }
  char packet[kConsoleLineCapacity + 32] = {};
  const int length = snprintf(packet, sizeof(packet), "L,%lu,%s\n",
                              static_cast<unsigned long>(_consoleSequence++), line);
  if (length <= 0 || length >= static_cast<int>(sizeof(packet)))
  {
    return;
  }
  _udp.beginPacket(_telemetryIp, _telemetryPort);
  _udp.write(reinterpret_cast<const uint8_t *>(packet), static_cast<size_t>(length));
  _udp.endPacket();
}

void WifiDebugServer::sendConsoleHistory()
{
  const uint8_t first = _consoleHistoryCount == kConsoleHistoryDepth ? _consoleHistoryNext : 0;
  for (uint8_t index = 0; index < _consoleHistoryCount; ++index)
  {
    const uint8_t slot = static_cast<uint8_t>((first + index) % kConsoleHistoryDepth);
    sendConsoleLine(_consoleHistory[slot]);
  }
}

void WifiDebugServer::sendReply(uint32_t requestSequence, const char *status, const char *reason)
{
  if (!_started || _replyPort == 0)
  {
    return;
  }
  char reply[64] = {};
  const int length = snprintf(reply, sizeof(reply), "A,%lu,%s,%s\n",
                              static_cast<unsigned long>(requestSequence), status, reason);
  if (length <= 0 || length >= static_cast<int>(sizeof(reply)))
  {
    return;
  }
  _udp.beginPacket(_replyIp, _replyPort);
  _udp.write(reinterpret_cast<const uint8_t *>(reply), static_cast<size_t>(length));
  _udp.endPacket();
}
} // namespace balance_car::app
