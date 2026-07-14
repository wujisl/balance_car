#include "app/safety_manager.h"

#include <math.h>

namespace balance_car::app
{
SafetyManager::SafetyManager(drivers::MotorDriver &motorDriver, const config::SafetyConfiguration &configuration)
    : _motorDriver(motorDriver), _configuration(configuration)
{
}

void SafetyManager::begin()
{
  _motorDriver.setEnabled(false);
  _state = SafetyState::SelfTesting;
  _faultCode = FaultCode::None;
  _selfTestPassed = false;
}

void SafetyManager::completeSelfTest(const SelfTestReport &report)
{
  // Startup uses the relaxed criterion defined by SelfTest: IMU initialized
  // and one readable sample.  Motor/encoder diagnostics stay informational.
  _selfTestPassed = report.imuReady && report.imuSampleValid;
  if (!report.passed)
  {
    reportFault(FaultCode::SelfTestFailed);
    return;
  }

  _motorDriver.setEnabled(false);
  _state = SafetyState::Standby;
  _faultCode = FaultCode::None;
}

bool SafetyManager::requestBalance(float pitchDegrees, bool attitudeValid, bool imuHealthy)
{
  if (!_selfTestPassed || _state != SafetyState::Standby || !attitudeValid || !imuHealthy ||
      fabsf(pitchDegrees) > _configuration.balanceStartAngleLimitDegrees)
  {
    return false;
  }

  _motorDriver.setEnabled(true);
  _motorDriver.stop();
  _state = SafetyState::Balancing;
  return true;
}

void SafetyManager::monitorBalance(float pitchDegrees, bool attitudeValid, bool imuHealthy,
                                   bool enforcePitchLimit)
{
  if (_state != SafetyState::Balancing)
  {
    return;
  }

  if (!attitudeValid || !imuHealthy)
  {
    reportFault(FaultCode::ImuUnhealthy);
    return;
  }

  if (enforcePitchLimit && fabsf(pitchDegrees) >= _configuration.balanceFaultAngleDegrees)
  {
    reportFault(FaultCode::PitchLimitExceeded);
  }
}

bool SafetyManager::requestManualMotorTest(float leftPower, float rightPower, uint32_t nowMs)
{
  if (_state != SafetyState::Standby && _state != SafetyState::ManualTest)
  {
    return false;
  }

  _motorDriver.setEnabled(true);
  _motorDriver.setNormalized(leftPower, rightPower);
  _manualTestExpiresAtMs = nowMs + _configuration.manualTestDurationMs;
  _state = SafetyState::ManualTest;
  return true;
}

void SafetyManager::disarm()
{
  _motorDriver.setEnabled(false);
  _manualTestExpiresAtMs = 0;
  if (_state != SafetyState::Fault)
  {
    _state = _selfTestPassed ? SafetyState::Standby : SafetyState::SelfTesting;
  }
}

void SafetyManager::reportFault(FaultCode faultCode)
{
  _motorDriver.setEnabled(false);
  _manualTestExpiresAtMs = 0;
  _faultCode = faultCode;
  _state = SafetyState::Fault;
}

void SafetyManager::update(uint32_t nowMs)
{
  if (_state == SafetyState::ManualTest && static_cast<int32_t>(nowMs - _manualTestExpiresAtMs) >= 0)
  {
    disarm();
  }
}

SafetyState SafetyManager::state() const
{
  return _state;
}

FaultCode SafetyManager::faultCode() const
{
  return _faultCode;
}

bool SafetyManager::isBalancing() const
{
  return _state == SafetyState::Balancing;
}

const char *SafetyManager::stateName(SafetyState state)
{
  switch (state)
  {
  case SafetyState::Boot:
    return "BOOT";
  case SafetyState::SelfTesting:
    return "SELF_TESTING";
  case SafetyState::Standby:
    return "STANDBY";
  case SafetyState::ManualTest:
    return "MANUAL_TEST";
  case SafetyState::Balancing:
    return "BALANCING";
  case SafetyState::Fault:
    return "FAULT";
  }
  return "UNKNOWN";
}

const char *SafetyManager::faultName(FaultCode faultCode)
{
  switch (faultCode)
  {
  case FaultCode::None:
    return "NONE";
  case FaultCode::SelfTestFailed:
    return "SELF_TEST_FAILED";
  case FaultCode::ImuUnhealthy:
    return "IMU_UNHEALTHY";
  case FaultCode::PitchLimitExceeded:
    return "PITCH_LIMIT_EXCEEDED";
  case FaultCode::AirborneLandingFailed:
    return "AIRBORNE_LANDING_FAILED";
  }
  return "UNKNOWN";
}
} // namespace balance_car::app
