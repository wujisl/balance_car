#pragma once

#include "app/self_test.h"
#include "config/vehicle_config.h"
#include "drivers/motor_driver.h"

#include <Arduino.h>

namespace balance_car::app
{
enum class SafetyState
{
  Boot,
  SelfTesting,
  Standby,
  ManualTest,
  Balancing,
  Fault,
};

enum class FaultCode
{
  None,
  SelfTestFailed,
  ImuUnhealthy,
  PitchLimitExceeded,
  AirborneLandingFailed,
};

class SafetyManager
{
public:
  SafetyManager(drivers::MotorDriver &motorDriver, const config::SafetyConfiguration &configuration);

  void begin();
  void completeSelfTest(const SelfTestReport &report);
  bool requestManualMotorTest(float leftPower, float rightPower, uint32_t nowMs);
  bool requestBalance(float pitchDegrees, bool attitudeValid, bool imuHealthy);
  void monitorBalance(float pitchDegrees, bool attitudeValid, bool imuHealthy,
                      bool enforcePitchLimit = true);
  void disarm();
  void reportFault(FaultCode faultCode);
  void update(uint32_t nowMs);
  SafetyState state() const;
  FaultCode faultCode() const;
  bool isBalancing() const;
  static const char *stateName(SafetyState state);
  static const char *faultName(FaultCode faultCode);

private:
  drivers::MotorDriver &_motorDriver;
  const config::SafetyConfiguration &_configuration;
  SafetyState _state = SafetyState::Boot;
  FaultCode _faultCode = FaultCode::None;
  uint32_t _manualTestExpiresAtMs = 0;
  bool _selfTestPassed = false;
};
} // namespace balance_car::app
