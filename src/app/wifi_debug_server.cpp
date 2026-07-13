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
constexpr size_t kTelemetryBufferCapacity = 512;

bool isAllowedParameter(const char *domain, const char *parameter)
{
  return (strcmp(domain, "balance") == 0 &&
          (strcmp(parameter, "kp") == 0 || strcmp(parameter, "ki") == 0 ||
           strcmp(parameter, "kd") == 0 || strcmp(parameter, "trim") == 0 ||
           strcmp(parameter, "max_motor") == 0)) ||
         (strcmp(domain, "speed") == 0 &&
          (strcmp(parameter, "kp") == 0 || strcmp(parameter, "ki") == 0 ||
           strcmp(parameter, "max_pitch") == 0));
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
  const int length = snprintf(
      packet, sizeof(packet),
      "T,1,%lu,%lu,%u,%u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.5f,%.5f,%.5f,%.3f,%.5f,%.5f,%.3f,%.3f\n",
      static_cast<unsigned long>(_telemetrySequence++),
      static_cast<unsigned long>(telemetry.timestampMs),
      static_cast<unsigned int>(telemetry.safetyState),
      static_cast<unsigned int>(telemetry.faultCode),
      telemetry.imuValid ? 1U : 0U,
      telemetry.pitchDegrees, telemetry.pitchRateDps, telemetry.accelerometerPitchDegrees,
      telemetry.accelXG, telemetry.accelYG, telemetry.accelZG,
      telemetry.gyroXDps, telemetry.gyroYDps, telemetry.gyroZDps,
      telemetry.targetSpeedMps, telemetry.filteredSpeedMps, telemetry.speedErrorMps,
      telemetry.speedPitchOffsetDegrees, telemetry.turnCommand,
      telemetry.leftMotorCommand, telemetry.rightMotorCommand,
      telemetry.balanceKp, telemetry.balanceKi, telemetry.balanceKd,
      telemetry.balanceTrimDegrees, telemetry.speedKp, telemetry.speedKi,
      telemetry.maximumMotorCommand, telemetry.maximumPitchOffsetDegrees);
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
  if (nowMs - _lastTelemetryDiagnosticsMs >= 1000)
  {
    _lastTelemetryDiagnosticsMs = nowMs;
    Serial.print("[WIFI] TELEMETRY_TX=");
    Serial.print(_telemetryPacketsSinceDiagnostics);
    Serial.print(" TARGET=");
    Serial.print(targetIp);
    Serial.print(':');
    Serial.print(targetPort);
    Serial.print(" LAST_SEND=");
    Serial.println(sent ? "OK" : "FAILED");
    char logLine[kConsoleLineCapacity] = {};
    snprintf(logLine, sizeof(logLine), "[WIFI] TELEMETRY_TX=%u TARGET=%u.%u.%u.%u:%u LAST_SEND=%s",
             _telemetryPacketsSinceDiagnostics,
             targetIp[0], targetIp[1], targetIp[2], targetIp[3], targetPort,
             sent ? "OK" : "FAILED");
    sendConsoleLine(logLine);
    _telemetryPacketsSinceDiagnostics = 0;
  }
}

bool WifiDebugServer::parseCommand(char *packet, size_t length)
{
  packet[length] = '\0';
  while (length > 0 && isspace(static_cast<unsigned char>(packet[length - 1])))
  {
    packet[--length] = '\0';
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
    if (changed)
    {
      Serial.print("[WIFI] TELEMETRY_SUBSCRIBER=");
      Serial.print(_telemetryIp);
      Serial.print(':');
      Serial.println(_telemetryPort);
      char logLine[kConsoleLineCapacity] = {};
      snprintf(logLine, sizeof(logLine), "[WIFI] TELEMETRY_SUBSCRIBER=%u.%u.%u.%u:%u",
               _telemetryIp[0], _telemetryIp[1], _telemetryIp[2], _telemetryIp[3], _telemetryPort);
      sendConsoleLine(logLine);
    }
    sendConsoleHistory();
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
    if (action == nullptr || strtok_r(nullptr, ",", &context) != nullptr)
    {
      sendReply(static_cast<uint32_t>(sequence), "ERR", "FORMAT");
      return false;
    }
    toLowercase(action);
    _pendingCommand = {};
    _pendingCommand.requestSequence = static_cast<uint32_t>(sequence);
    if (strcmp(action, "arm") == 0)
      _pendingCommand.kind = WifiCommandKind::Arm;
    else if (strcmp(action, "stop") == 0)
      _pendingCommand.kind = WifiCommandKind::Stop;
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
