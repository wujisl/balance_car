#include <Arduino.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "app/flight_logger.h"
#include "app/offline_arm_control.h"
#include "app/safety_manager.h"
#include "app/self_test.h"
#include "config/board_pins.h"
#include "config/vehicle_config.h"
#include "control/attitude_estimator.h"
#include "control/balance_controller.h"
#include "control/motion_command.h"
#include "control/motor_mixer.h"
#include "control/velocity_controller.h"
#include "drivers/encoder_driver.h"
#include "drivers/imu_driver.h"
#include "drivers/motor_driver.h"
#include "hal/i2c_bus.h"

namespace
{
constexpr uint32_t kTelemetryPeriodMs = 500;
constexpr size_t kSerialCommandCapacity = 96;

balance_car::hal::I2cBus i2cBus(Wire);
balance_car::drivers::MotorDriver motorDriver(
    balance_car::config::kMotorPins, balance_car::config::kMotorConfiguration);
balance_car::drivers::EncoderDriver encoderDriver(
    balance_car::config::kEncoderPins, balance_car::config::kEncoderConfiguration);
balance_car::drivers::ImuDriver imuDriver(
    i2cBus, balance_car::config::kImuPins, balance_car::config::kImuConfiguration);
balance_car::control::AttitudeEstimator attitudeEstimator(
    balance_car::config::kAttitudeConfiguration);
balance_car::control::BalanceController balanceController(
    balance_car::config::kBalanceConfiguration);
balance_car::control::VelocityController velocityController(
    balance_car::config::kVelocityConfiguration);
balance_car::control::MotionCommand motionCommand(
    balance_car::config::kMotionConfiguration);
balance_car::app::SelfTest selfTest(motorDriver, encoderDriver, imuDriver);
balance_car::app::SafetyManager safetyManager(
    motorDriver, balance_car::config::kSafetyConfiguration);
balance_car::app::OfflineArmControl offlineArmControl(
    balance_car::config::kBalanceArmButtonPin, balance_car::config::kSafetyConfiguration.offlineArmHoldMs);
balance_car::app::FlightLogger flightLogger;

balance_car::drivers::ImuSample latestImuSample = {};
balance_car::drivers::WheelSpeed latestWheelSpeed = {};
balance_car::control::AttitudeState latestAttitude = {};
balance_car::control::MixedMotorCommand latestMixedMotorCommand = {};
float latestBalanceMotorCommand = 0.0F;
float latestVelocityPitchOffsetDegrees = 0.0F;
uint32_t lastControlUpdateMs = 0;
uint32_t lastVelocityUpdateMs = 0;
uint32_t lastTelemetryMs = 0;
char serialCommandBuffer[kSerialCommandCapacity] = {};
size_t serialCommandLength = 0;
bool autoBalanceStartPending = false;

void printHelp()
{
  Serial.println("[CMD] h/help s/status b/arm x/stop 1..6=motor-test");
  Serial.println("[CMD] w/z speed+/- c=speed-zero a/d turn-/+ f=turn-zero");
  Serial.println("[CMD] set balance kp|ki|kd|trim <value>");
  Serial.println("[CMD] set speed kp|ki|target|invert <value>");
  Serial.println("[CMD] set motion speed|turn <value>");
  Serial.println("[CMD] log status|dump|clear (dump and clear require the motors to be stopped)");
  Serial.println("[CMD] After reset and self-test, balance starts automatically when the IMU sample is valid; press BOOT while balancing to stop.");
}

void printStatus()
{
  const balance_car::control::BalanceTuning balanceTuning = balanceController.tuning();
  const balance_car::control::VelocityTuning velocityTuning = velocityController.tuning();
  const balance_car::control::VelocityState velocityState = velocityController.state();
  Serial.print("[STATUS] STATE=");
  Serial.print(balance_car::app::SafetyManager::stateName(safetyManager.state()));
  Serial.print(" FAULT=");
  Serial.print(balance_car::app::SafetyManager::faultName(safetyManager.faultCode()));
  Serial.print(" PITCH=");
  Serial.print(latestAttitude.pitchDegrees, 2);
  Serial.print(" TARGET_SPEED=");
  Serial.print(motionCommand.targetSpeedMps(), 3);
  Serial.print(" FILTERED_SPEED=");
  Serial.print(velocityState.filteredSpeedMps, 3);
  Serial.print(" TURN=");
  Serial.println(motionCommand.turnCommand(), 3);
  Serial.print("[TUNING] BALANCE_KP=");
  Serial.print(balanceTuning.proportionalGain, 4);
  Serial.print(" KI=");
  Serial.print(balanceTuning.integralGain, 4);
  Serial.print(" KD=");
  Serial.print(balanceTuning.derivativeGain, 4);
  Serial.print(" TRIM=");
  Serial.print(balanceTuning.targetPitchDegrees, 3);
  Serial.print(" SPEED_KP=");
  Serial.print(velocityTuning.proportionalGain, 3);
  Serial.print(" KI=");
  Serial.print(velocityTuning.integralGain, 3);
  Serial.print(" INVERT=");
  Serial.println(velocityTuning.outputInverted ? 1 : 0);
}

void printMotionCommand()
{
  Serial.print("[MOTION] TARGET_SPEED_MPS=");
  Serial.print(motionCommand.targetSpeedMps(), 3);
  Serial.print(" TURN_COMMAND=");
  Serial.println(motionCommand.turnCommand(), 3);
}

void appendFlightLogSample()
{
  const balance_car::control::BalanceState &balanceState = balanceController.state();
  balance_car::app::FlightLogSample sample = {};
  sample.timestampMs = latestImuSample.timestampMs;
  sample.pitchDegrees = latestAttitude.pitchDegrees;
  sample.accelerometerPitchDegrees = latestAttitude.accelerometerPitchDegrees;
  sample.pitchRateDps = latestAttitude.pitchRateDps;
  sample.requestedPitchDegrees = balanceState.requestedPitchDegrees;
  sample.pitchErrorDegrees = balanceState.pitchErrorDegrees;
  sample.proportionalTerm = balanceState.proportionalTerm;
  sample.integralTerm = balanceState.integralTerm;
  sample.derivativeTerm = balanceState.derivativeTerm;
  sample.unclampedBalanceCommand = balanceState.unclampedMotorCommand;
  sample.balanceCommand = balanceState.motorCommand;
  sample.leftMotorCommand = latestMixedMotorCommand.left;
  sample.rightMotorCommand = latestMixedMotorCommand.right;
  sample.leftSpeedMps = latestWheelSpeed.leftMetersPerSecond;
  sample.rightSpeedMps = latestWheelSpeed.rightMetersPerSecond;
  sample.safetyState = static_cast<uint8_t>(safetyManager.state());
  sample.faultCode = static_cast<uint8_t>(safetyManager.faultCode());
  sample.outputSaturated = balanceState.outputSaturated ? 1U : 0U;
  flightLogger.append(sample);
}

void requestMotorTest(float leftPower, float rightPower)
{
  if (safetyManager.requestManualMotorTest(leftPower, rightPower, millis()))
  {
    Serial.println("[DIAG] MOTOR_TEST=ACTIVE");
    return;
  }
  Serial.println("[DIAG] MOTOR_TEST=REJECTED");
}

void requestBalance()
{
  if (safetyManager.requestBalance(latestAttitude.pitchDegrees, latestAttitude.valid, imuDriver.isHealthy()))
  {
    balanceController.reset();
    velocityController.reset();
    motionCommand.clear();
    latestBalanceMotorCommand = 0.0F;
    latestVelocityPitchOffsetDegrees = 0.0F;
    flightLogger.startSession();
    Serial.println("[BALANCE] STATE=ACTIVE");
    return;
  }
  Serial.println("[BALANCE] STATE=REJECTED");
}

void stopMotorOutput()
{
  safetyManager.disarm();
  const bool logSaved = flightLogger.saveSession();
  balanceController.reset();
  velocityController.reset();
  motionCommand.clear();
  latestBalanceMotorCommand = 0.0F;
  latestVelocityPitchOffsetDegrees = 0.0F;
  latestMixedMotorCommand = {};
  if (!logSaved)
  {
    Serial.println("[LOG] SAVE=FAILED");
  }
}

void applyTuningCommand(const char *domain, const char *parameter, float value)
{
  if (strcmp(domain, "balance") == 0)
  {
    if (strcmp(parameter, "kp") == 0)
    {
      balanceController.setProportionalGain(value);
    }
    else if (strcmp(parameter, "ki") == 0)
    {
      balanceController.setIntegralGain(value);
    }
    else if (strcmp(parameter, "kd") == 0)
    {
      balanceController.setDerivativeGain(value);
    }
    else if (strcmp(parameter, "trim") == 0)
    {
      balanceController.setTargetPitchDegrees(value);
    }
    else
    {
      Serial.println("[CMD] UNKNOWN_BALANCE_PARAMETER");
      return;
    }
  }
  else if (strcmp(domain, "speed") == 0)
  {
    if (strcmp(parameter, "kp") == 0)
    {
      velocityController.setProportionalGain(value);
    }
    else if (strcmp(parameter, "ki") == 0)
    {
      velocityController.setIntegralGain(value);
    }
    else if (strcmp(parameter, "target") == 0)
    {
      motionCommand.setTargetSpeedMps(value);
    }
    else if (strcmp(parameter, "invert") == 0)
    {
      velocityController.setOutputInverted(value >= 0.5F);
    }
    else
    {
      Serial.println("[CMD] UNKNOWN_SPEED_PARAMETER");
      return;
    }
  }
  else if (strcmp(domain, "motion") == 0)
  {
    if (strcmp(parameter, "speed") == 0)
    {
      motionCommand.setTargetSpeedMps(value);
    }
    else if (strcmp(parameter, "turn") == 0)
    {
      motionCommand.setTurnCommand(value);
    }
    else
    {
      Serial.println("[CMD] UNKNOWN_MOTION_PARAMETER");
      return;
    }
  }
  else
  {
    Serial.println("[CMD] UNKNOWN_TUNING_DOMAIN");
    return;
  }

  Serial.println("[CMD] SET=OK");
  printStatus();
}

void processSingleCharacterCommand(char command)
{
  switch (command)
  {
  case 'h':
    printHelp();
    break;
  case 's':
    printStatus();
    break;
  case 'b':
    requestBalance();
    break;
  case 'x':
  case '0':
    stopMotorOutput();
    Serial.println("[DIAG] MOTOR_OUTPUT=STOPPED");
    break;
  case '1':
    requestMotorTest(balance_car::config::kSafetyConfiguration.manualTestPower, 0.0F);
    break;
  case '2':
    requestMotorTest(-balance_car::config::kSafetyConfiguration.manualTestPower, 0.0F);
    break;
  case '3':
    requestMotorTest(0.0F, balance_car::config::kSafetyConfiguration.manualTestPower);
    break;
  case '4':
    requestMotorTest(0.0F, -balance_car::config::kSafetyConfiguration.manualTestPower);
    break;
  case '5':
    requestMotorTest(balance_car::config::kSafetyConfiguration.manualTestPower,
                     balance_car::config::kSafetyConfiguration.manualTestPower);
    break;
  case '6':
    requestMotorTest(-balance_car::config::kSafetyConfiguration.manualTestPower,
                     -balance_car::config::kSafetyConfiguration.manualTestPower);
    break;
  case 'w':
    motionCommand.adjustTargetSpeedMps(balance_car::config::kMotionConfiguration.targetSpeedStepMps);
    printMotionCommand();
    break;
  case 'z':
    motionCommand.adjustTargetSpeedMps(-balance_car::config::kMotionConfiguration.targetSpeedStepMps);
    printMotionCommand();
    break;
  case 'c':
    motionCommand.setTargetSpeedMps(0.0F);
    printMotionCommand();
    break;
  case 'a':
    motionCommand.adjustTurnCommand(-balance_car::config::kMotionConfiguration.turnCommandStep);
    printMotionCommand();
    break;
  case 'd':
    motionCommand.adjustTurnCommand(balance_car::config::kMotionConfiguration.turnCommandStep);
    printMotionCommand();
    break;
  case 'f':
    motionCommand.setTurnCommand(0.0F);
    printMotionCommand();
    break;
  default:
    Serial.println("[CMD] UNKNOWN");
    break;
  }
}

void processLogCommand(const char *action)
{
  if (strcmp(action, "status") == 0)
  {
    flightLogger.printStatus(Serial);
    return;
  }
  if (safetyManager.isBalancing())
  {
    Serial.println("[LOG] REJECTED_WHILE_BALANCING");
    return;
  }
  if (strcmp(action, "dump") == 0)
  {
    flightLogger.dumpCsv(Serial);
    return;
  }
  if (strcmp(action, "clear") == 0)
  {
    Serial.println(flightLogger.clearSavedLog() ? "[LOG] CLEAR=OK" : "[LOG] CLEAR=FAILED");
    return;
  }
  Serial.println("[LOG] UNKNOWN_ACTION");
}

void processCommandLine(char *commandLine)
{
  if (commandLine[0] == '\0')
  {
    return;
  }

  if (commandLine[1] == '\0')
  {
    processSingleCharacterCommand(commandLine[0]);
    return;
  }

  if (strcmp(commandLine, "help") == 0)
  {
    printHelp();
    return;
  }
  if (strcmp(commandLine, "status") == 0 || strcmp(commandLine, "show") == 0)
  {
    printStatus();
    return;
  }
  if (strcmp(commandLine, "arm") == 0)
  {
    requestBalance();
    return;
  }
  if (strcmp(commandLine, "stop") == 0)
  {
    stopMotorOutput();
    Serial.println("[DIAG] MOTOR_OUTPUT=STOPPED");
    return;
  }

  char logAction[16] = {};
  if (sscanf(commandLine, "log %15s", logAction) == 1)
  {
    processLogCommand(logAction);
    return;
  }

  char domain[16] = {};
  char parameter[16] = {};
  float value = 0.0F;
  if (sscanf(commandLine, "set %15s %15s %f", domain, parameter, &value) == 3)
  {
    applyTuningCommand(domain, parameter, value);
    return;
  }

  Serial.println("[CMD] INVALID_FORMAT");
}

void processSerialInput()
{
  while (Serial.available() > 0)
  {
    const char input = static_cast<char>(Serial.read());
    if (input == '\r' || input == '\n')
    {
      if (serialCommandLength > 0)
      {
        serialCommandBuffer[serialCommandLength] = '\0';
        processCommandLine(serialCommandBuffer);
        serialCommandLength = 0;
      }
      continue;
    }
    if (isprint(static_cast<unsigned char>(input)) && serialCommandLength < kSerialCommandCapacity - 1)
    {
      serialCommandBuffer[serialCommandLength++] = input;
    }
  }
}

void processOfflineArmControl(uint32_t nowMs)
{
  const balance_car::app::OfflineArmEvent event = offlineArmControl.update(nowMs, safetyManager.state());
  if (event == balance_car::app::OfflineArmEvent::StopBalance)
  {
    stopMotorOutput();
    Serial.println("[OFFLINE] MOTOR_OUTPUT=STOPPED");
  }
}

void updateVelocityControl(uint32_t nowMs)
{
  if (nowMs - lastVelocityUpdateMs < balance_car::config::kVelocityConfiguration.controlPeriodMs)
  {
    return;
  }

  const uint32_t elapsedMs = nowMs - lastVelocityUpdateMs;
  lastVelocityUpdateMs = nowMs;
  latestWheelSpeed = encoderDriver.sample(static_cast<float>(elapsedMs) / 1000.0F);
  if (!safetyManager.isBalancing())
  {
    velocityController.reset();
    latestVelocityPitchOffsetDegrees = 0.0F;
    return;
  }

  const float measuredSpeedMps = 0.5F *
                                 (latestWheelSpeed.leftMetersPerSecond + latestWheelSpeed.rightMetersPerSecond);
  latestVelocityPitchOffsetDegrees = velocityController.update(
      motionCommand.targetSpeedMps(), measuredSpeedMps, static_cast<float>(elapsedMs) / 1000.0F);
}

void updateBalanceControl(uint32_t nowMs)
{
  if (nowMs - lastControlUpdateMs < balance_car::config::kBalanceConfiguration.controlPeriodMs)
  {
    return;
  }

  lastControlUpdateMs = nowMs;
  latestImuSample = imuDriver.read();
  latestAttitude = attitudeEstimator.update(latestImuSample);
  if (autoBalanceStartPending)
  {
    // Start once after reset, only after a valid attitude sample is available.
    // A rejected request (for example, the car is already lying down) requires
    // another reset after placing the car upright.
    autoBalanceStartPending = false;
    requestBalance();
  }
  const bool wasBalancing = safetyManager.isBalancing();
  safetyManager.monitorBalance(latestAttitude.pitchDegrees, latestAttitude.valid, imuDriver.isHealthy());

  if (!safetyManager.isBalancing())
  {
    if (wasBalancing)
    {
      appendFlightLogSample();
      if (!flightLogger.saveSession())
      {
        Serial.println("[LOG] SAVE=FAILED");
      }
    }
    latestBalanceMotorCommand = 0.0F;
    latestMixedMotorCommand = {};
    return;
  }

  latestBalanceMotorCommand = balanceController.update(latestAttitude, latestVelocityPitchOffsetDegrees);
  latestMixedMotorCommand = balance_car::control::MotorMixer::mix(
      latestBalanceMotorCommand, motionCommand.turnCommand());
  motorDriver.setNormalized(latestMixedMotorCommand.left, latestMixedMotorCommand.right);
  appendFlightLogSample();
}

void printTelemetry(uint32_t nowMs)
{
  if (nowMs - lastTelemetryMs < kTelemetryPeriodMs)
  {
    return;
  }

  lastTelemetryMs = nowMs;
  const balance_car::control::VelocityState velocityState = velocityController.state();
  Serial.print("[TELEMETRY] pitch_deg=");
  Serial.print(latestAttitude.pitchDegrees, 2);
  Serial.print(" accel_pitch_deg=");
  Serial.print(latestAttitude.accelerometerPitchDegrees, 2);
  Serial.print(" pitch_rate_dps=");
  Serial.print(latestAttitude.pitchRateDps, 2);
  Serial.print(" accel_g=");
  Serial.print(latestImuSample.accelXG, 3);
  Serial.print(',');
  Serial.print(latestImuSample.accelYG, 3);
  Serial.print(',');
  Serial.print(latestImuSample.accelZG, 3);
  Serial.print(" gyro_dps=");
  Serial.print(latestImuSample.gyroXDps, 2);
  Serial.print(',');
  Serial.print(latestImuSample.gyroYDps, 2);
  Serial.print(',');
  Serial.print(latestImuSample.gyroZDps, 2);
  Serial.print(" target_speed_mps=");
  Serial.print(motionCommand.targetSpeedMps(), 3);
  Serial.print(" measured_speed_mps=");
  Serial.print(velocityState.filteredSpeedMps, 3);
  Serial.print(" speed_error_mps=");
  Serial.print(velocityState.speedErrorMps, 3);
  Serial.print(" speed_pitch_offset_deg=");
  Serial.print(latestVelocityPitchOffsetDegrees, 3);
  Serial.print(" turn=");
  Serial.print(motionCommand.turnCommand(), 3);
  Serial.print(" motor_lr=");
  Serial.print(latestMixedMotorCommand.left, 3);
  Serial.print(',');
  Serial.print(latestMixedMotorCommand.right, 3);
  Serial.print(" ticks=");
  Serial.print(latestWheelSpeed.leftTicks);
  Serial.print(',');
  Serial.println(latestWheelSpeed.rightTicks);
}
} // namespace

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("===== Balance Car Cascaded Control =====");
  offlineArmControl.begin();
  Serial.println(flightLogger.begin() ? "[LOG] STORAGE=READY" : "[LOG] STORAGE=UNAVAILABLE");
  safetyManager.begin();
  const balance_car::app::SelfTestReport report = selfTest.run();
  balance_car::app::SelfTest::printReport(Serial, report);
  safetyManager.completeSelfTest(report);
  autoBalanceStartPending = report.passed;
  printStatus();
  printHelp();
}

void loop()
{
  const uint32_t nowMs = millis();
  processSerialInput();
  safetyManager.update(nowMs);
  processOfflineArmControl(nowMs);
  updateVelocityControl(nowMs);
  updateBalanceControl(nowMs);
  printTelemetry(nowMs);
  delay(1);
}
