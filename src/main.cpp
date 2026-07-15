#include <Arduino.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "app/airborne_landing_manager.h"
#include "app/climb_mode_manager.h"
#include "app/offline_arm_control.h"
#include "app/safety_manager.h"
#include "app/self_test.h"
#include "app/wifi_debug_server.h"
#include "config/board_pins.h"
#include "config/vehicle_config.h"
#include "control/attitude_estimator.h"
#include "control/balance_controller.h"
#include "control/differential_speed_controller.h"
#include "control/differential_odometry.h"
#include "control/motion_command.h"
#include "control/motor_mixer.h"
#include "control/velocity_controller.h"
#include "drivers/encoder_driver.h"
#include "drivers/imu_driver.h"
#include "drivers/motor_driver.h"
#include "drivers/vision_i2c_client.h"
#include "hal/i2c_bus.h"

namespace
{
  constexpr uint32_t kTelemetryPeriodMs = 500;
  constexpr size_t kSerialCommandCapacity = 96;
  constexpr float kWifiDriveSpeedSlewRateMpsPerSecond = 0.10F;

  uint16_t millisecondsFromCommandValue(float value)
  {
    if (value <= 0.0F)
    {
      return 0;
    }
    if (value >= 65535.0F)
    {
      return 65535;
    }
    return static_cast<uint16_t>(value + 0.5F);
  }

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
  balance_car::control::DifferentialSpeedController differentialSpeedController(
      balance_car::config::kDifferentialSpeedConfiguration);
  balance_car::control::DifferentialOdometry differentialOdometry(
      balance_car::config::kOdometryConfiguration);
  balance_car::control::MotionCommand motionCommand(
      balance_car::config::kMotionConfiguration);
  balance_car::app::SelfTest selfTest(motorDriver, encoderDriver, imuDriver);
  balance_car::app::SafetyManager safetyManager(
      motorDriver, balance_car::config::kSafetyConfiguration);
  balance_car::app::AirborneLandingManager airborneLandingManager(
      balance_car::config::kAirborneLandingConfiguration);
  balance_car::app::ClimbModeManager climbModeManager(
      balance_car::config::kClimbModeConfiguration);
  balance_car::app::OfflineArmControl offlineArmControl(
      balance_car::config::kBalanceArmButtonPin, balance_car::config::kSafetyConfiguration.offlineArmHoldMs);
  balance_car::app::WifiDebugServer wifiDebugServer(balance_car::config::kWifiDebugConfiguration);

  class DebugConsole final : public Stream
  {
  public:
    DebugConsole(HardwareSerial &serial, balance_car::app::WifiDebugServer &wifi)
        : _serial(serial), _wifi(wifi)
    {
    }

    void begin(unsigned long baudRate)
    {
      _serial.begin(baudRate);
    }

    int available() override { return _serial.available(); }
    int read() override { return _serial.read(); }
    int peek() override { return _serial.peek(); }
    void flush() override { _serial.flush(); }

    size_t write(uint8_t byte) override
    {
      const size_t written = _serial.write(byte);
      _wifi.writeConsoleByte(byte);
      return written;
    }

    size_t write(const uint8_t *data, size_t length) override
    {
      const size_t written = _serial.write(data, length);
      _wifi.writeConsoleBytes(data, length);
      return written;
    }

  private:
    HardwareSerial &_serial;
    balance_car::app::WifiDebugServer &_wifi;
  };

  DebugConsole debugConsole(Serial, wifiDebugServer);

#define Serial debugConsole

  balance_car::drivers::ImuSample latestImuSample = {};
  balance_car::drivers::WheelSpeed latestWheelSpeed = {};
  balance_car::control::AttitudeState latestAttitude = {};
  balance_car::control::MixedMotorCommand latestMixedMotorCommand = {};
  float latestBalanceMotorCommand = 0.0F;
  float latestVelocityPitchOffsetDegrees = 0.0F;
  float latestTurnMotorCommand = 0.0F;
  balance_car::app::ClimbModeOutput latestClimbModeOutput = {};
  float latestEffectiveTargetSpeedMps = 0.0F;
  float latestEffectiveDifferentialSpeedMps = 0.0F;
  float wifiDriveRequestedSpeedMps = 0.0F;
  bool wifiDriveSlewActive = false;
  uint32_t lastControlUpdateMs = 0;
  uint32_t lastVelocityUpdateMs = 0;
  uint32_t lastTelemetryMs = 0;
  balance_car::app::SafetyState lastReportedSafetyState = balance_car::app::SafetyState::Boot;
  char serialCommandBuffer[kSerialCommandCapacity] = {};
  size_t serialCommandLength = 0;

  void printHelp()
  {
    Serial.println("[CMD] h/help s/status b/arm x/stop 1..6=motor-test");
    Serial.println("[CMD] w/z speed+/- c=speed-zero a/d diff-speed-/+ f=diff-speed-zero");
    Serial.println("[CMD] set balance kp|ki|kd|trim <value>");
    Serial.println("[CMD] set speed kp|ki|target|invert <value>");
    Serial.println("[CMD] set turn kp|ki|max|invert <value>");
    Serial.println("[CMD] set motion speed|turn <value>");
    Serial.println("[CMD] climb on|off (manual mode; no automatic slope detection)");
    Serial.println("[CMD] set climb ff|ki|max_pitch|max_motor|max_speed|max_turn|invert <value>");
    Serial.println("[CMD] Hold BOOT for 1.5 seconds in STANDBY to arm offline balance; press BOOT while balancing to stop.");
  }

  void printStatus()
  {
    const balance_car::control::BalanceTuning balanceTuning = balanceController.tuning();
    const balance_car::control::VelocityTuning velocityTuning = velocityController.tuning();
    const balance_car::control::VelocityState velocityState = velocityController.state();
    const balance_car::control::DifferentialSpeedTuning turnTuning = differentialSpeedController.tuning();
    const balance_car::control::DifferentialSpeedState turnState = differentialSpeedController.state();
    Serial.print("[STATUS] STATE=");
    Serial.print(balance_car::app::SafetyManager::stateName(safetyManager.state()));
    Serial.print(" FAULT=");
    Serial.print(balance_car::app::SafetyManager::faultName(safetyManager.faultCode()));
    Serial.print(" PITCH=");
    Serial.print(latestAttitude.pitchDegrees, 2);
    Serial.print(" TARGET_SPEED=");
    Serial.print(latestEffectiveTargetSpeedMps, 3);
    Serial.print(" FILTERED_SPEED=");
    Serial.print(velocityState.filteredSpeedMps, 3);
    Serial.print(" TARGET_DIFF_SPEED=");
    Serial.print(latestEffectiveDifferentialSpeedMps, 3);
    Serial.print(" FILTERED_DIFF_SPEED=");
    Serial.print(turnState.filteredDifferentialSpeedMps, 3);
    Serial.print(" TURN_OUTPUT=");
    Serial.println(latestTurnMotorCommand, 3);
    Serial.print("[LANDING] ENABLED=");
    Serial.print(airborneLandingManager.isEnabled() ? 1 : 0);
    Serial.print(" STATE=");
    Serial.println(balance_car::app::AirborneLandingManager::stateName(airborneLandingManager.state()));
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
    Serial.print(velocityTuning.outputInverted ? 1 : 0);
    Serial.print(" MAX_MOTOR=");
    Serial.print(balanceTuning.maximumMotorCommand, 3);
    Serial.print(" MAX_PITCH_OFFSET=");
    Serial.print(velocityTuning.maximumPitchOffsetDegrees, 3);
    Serial.print(" TURN_KP=");
    Serial.print(turnTuning.proportionalGain, 3);
    Serial.print(" TURN_KI=");
    Serial.print(turnTuning.integralGain, 3);
    Serial.print(" MAX_TURN_OUTPUT=");
    Serial.println(turnTuning.maximumTurnMotorCommand, 3);
    const balance_car::app::ClimbModeTuning climbTuning = climbModeManager.tuning();
    Serial.print("[CLIMB] REQUESTED=");
    Serial.print(climbModeManager.isRequested() ? 1 : 0);
    Serial.print(" STATE=");
    Serial.print(balance_car::app::ClimbModeManager::stateName(climbModeManager.state()));
    Serial.print(" FF=");
    Serial.print(latestClimbModeOutput.feedforwardPitchDegrees, 3);
    Serial.print(" I=");
    Serial.print(latestClimbModeOutput.integralPitchDegrees, 3);
    Serial.print(" OFFSET=");
    Serial.print(latestClimbModeOutput.pitchOffsetDegrees, 3);
    Serial.print(" MAX_MOTOR=");
    Serial.print(climbTuning.maximumMotorCommand, 3);
    Serial.print(" MAX_SPEED=");
    Serial.println(climbTuning.maximumTargetSpeedMps, 3);
  }

  void printMotionCommand()
  {
    Serial.print("[MOTION] TARGET_SPEED_MPS=");
    Serial.print(motionCommand.targetSpeedMps(), 3);
    Serial.print(" TARGET_DIFF_SPEED_MPS=");
    Serial.println(motionCommand.targetDifferentialSpeedMps(), 3);
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

  bool requestBalance()
  {
    if (safetyManager.requestBalance(latestAttitude.pitchDegrees, latestAttitude.valid, imuDriver.isHealthy()))
    {
      balanceController.reset();
      velocityController.reset();
      differentialSpeedController.reset();
      differentialOdometry.reset();
      airborneLandingManager.reset();
      climbModeManager.reset();
      motionCommand.clear();
      motionCommand.setTargetSpeedMps(balance_car::config::kMotionConfiguration.initialTargetSpeedMps);
      wifiDriveRequestedSpeedMps = 0.0F;
      wifiDriveSlewActive = false;
      latestBalanceMotorCommand = 0.0F;
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
      latestClimbModeOutput = {};
      latestEffectiveTargetSpeedMps = 0.0F;
      latestEffectiveDifferentialSpeedMps = 0.0F;
      Serial.print("[BALANCE] STATE=ACTIVE INITIAL_TARGET_SPEED_MPS=");
      Serial.println(motionCommand.targetSpeedMps(), 3);
      return true;
    }
    Serial.println("[BALANCE] STATE=REJECTED");
    return false;
  }

  void stopMotorOutput()
  {
    safetyManager.disarm();
    balanceController.reset();
    velocityController.reset();
    differentialSpeedController.reset();
    differentialOdometry.reset();
    airborneLandingManager.reset();
    climbModeManager.reset();
    motionCommand.clear();
    wifiDriveRequestedSpeedMps = 0.0F;
    wifiDriveSlewActive = false;
    latestBalanceMotorCommand = 0.0F;
    latestVelocityPitchOffsetDegrees = 0.0F;
    latestTurnMotorCommand = 0.0F;
    latestClimbModeOutput = {};
    latestEffectiveTargetSpeedMps = 0.0F;
    latestEffectiveDifferentialSpeedMps = 0.0F;
    latestMixedMotorCommand = {};
  }

  void cancelWifiDriveSlew()
  {
    wifiDriveRequestedSpeedMps = 0.0F;
    wifiDriveSlewActive = false;
  }

  bool requestClimbMode(bool enabled)
  {
    if (enabled && !safetyManager.isBalancing())
    {
      return false;
    }

    climbModeManager.setEnabled(enabled);
    Serial.print("[CLIMB] REQUESTED=");
    Serial.println(enabled ? "ON" : "OFF");
    return true;
  }

  bool requestAirborneLandingMode(bool enabled)
  {
    // Configuration must be changed before arming. Changing thresholds while
    // airborne or recovering would make one landing use mixed criteria.
    if (safetyManager.state() != balance_car::app::SafetyState::Standby)
    {
      return false;
    }

    airborneLandingManager.setEnabled(enabled);
    Serial.print("[LANDING] PROTECTION=");
    Serial.println(enabled ? "ON" : "OFF");
    return true;
  }

  void handleAirborneLandingEvent(balance_car::app::AirborneLandingEvent event)
  {
    using balance_car::app::AirborneLandingEvent;
    switch (event)
    {
    case AirborneLandingEvent::EnteredAirborne:
      // Ground-speed and differential-speed feedback are invalid without tire
      // contact. Drop motion requests before the landing recovery sequence.
      motionCommand.clear();
      cancelWifiDriveSlew();
      balanceController.reset();
      velocityController.reset();
      differentialSpeedController.reset();
      climbModeManager.reset();
      latestBalanceMotorCommand = 0.0F;
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
      latestMixedMotorCommand = {};
      Serial.println("[LANDING] STATE=AIRBORNE; motion commands cleared, motor output held");
      break;

    case AirborneLandingEvent::ResetAttitude:
      // The manager has observed normal gravity for the configured settling
      // interval. Re-anchor pitch before gradually restoring balance output.
      attitudeEstimator.reset();
      balanceController.reset();
      velocityController.reset();
      differentialSpeedController.reset();
      differentialOdometry.reset();
      climbModeManager.reset();
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
      latestClimbModeOutput = {};
      Serial.println("[LANDING] STATE=RECOVERING; attitude reset, motor output ramping");
      break;

    case AirborneLandingEvent::RecoveryComplete:
      Serial.println("[LANDING] STATE=GROUNDED; normal control resumed");
      break;

    case AirborneLandingEvent::Fault:
      safetyManager.reportFault(balance_car::app::FaultCode::AirborneLandingFailed);
      latestBalanceMotorCommand = 0.0F;
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
      latestMixedMotorCommand = {};
      Serial.println("[LANDING] STATE=FAULT; landing recovery timed out");
      break;

    case AirborneLandingEvent::None:
      break;
    }
  }

  bool requestWifiDriveSpeed(float speedMps)
  {
    if (!safetyManager.isBalancing())
    {
      return false;
    }
    if (speedMps < -balance_car::config::kMotionConfiguration.maximumTargetSpeedMps ||
        speedMps > balance_car::config::kMotionConfiguration.maximumTargetSpeedMps)
    {
      return false;
    }

    wifiDriveRequestedSpeedMps = speedMps;
    wifiDriveSlewActive = true;
    Serial.print("[WIFI] DRIVE_REQUEST_MPS=");
    Serial.println(speedMps, 3);
    return true;
  }

  void updateWifiDriveTarget(float deltaSeconds)
  {
    if (!wifiDriveSlewActive || deltaSeconds <= 0.0F)
    {
      return;
    }

    const float currentSpeedMps = motionCommand.targetSpeedMps();
    const float maximumDeltaMps = kWifiDriveSpeedSlewRateMpsPerSecond * deltaSeconds;
    float nextSpeedMps = currentSpeedMps;
    if (currentSpeedMps < wifiDriveRequestedSpeedMps)
    {
      nextSpeedMps += maximumDeltaMps;
      if (nextSpeedMps > wifiDriveRequestedSpeedMps)
      {
        nextSpeedMps = wifiDriveRequestedSpeedMps;
      }
    }
    else if (currentSpeedMps > wifiDriveRequestedSpeedMps)
    {
      nextSpeedMps -= maximumDeltaMps;
      if (nextSpeedMps < wifiDriveRequestedSpeedMps)
      {
        nextSpeedMps = wifiDriveRequestedSpeedMps;
      }
    }
    motionCommand.setTargetSpeedMps(nextSpeedMps);
    if (wifiDriveRequestedSpeedMps == 0.0F && nextSpeedMps == 0.0F)
    {
      wifiDriveSlewActive = false;
    }
  }

  void reportSafetyFaultTransition()
  {
    const balance_car::app::SafetyState currentState = safetyManager.state();
    if (currentState == balance_car::app::SafetyState::Fault &&
        lastReportedSafetyState != balance_car::app::SafetyState::Fault)
    {
      Serial.print("[SAFETY] FALL_OR_RUNTIME_FAULT: ");
      Serial.print(balance_car::app::SafetyManager::faultName(safetyManager.faultCode()));
      Serial.println("; motors disabled, Wi-Fi telemetry and logs continue.");
    }
    lastReportedSafetyState = currentState;
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
        cancelWifiDriveSlew();
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
    else if (strcmp(domain, "turn") == 0)
    {
      if (strcmp(parameter, "kp") == 0)
      {
        differentialSpeedController.setProportionalGain(value);
      }
      else if (strcmp(parameter, "ki") == 0)
      {
        differentialSpeedController.setIntegralGain(value);
      }
      else if (strcmp(parameter, "max") == 0)
      {
        differentialSpeedController.setMaximumTurnMotorCommand(value);
      }
      else if (strcmp(parameter, "invert") == 0)
      {
        differentialSpeedController.setOutputInverted(value >= 0.5F);
      }
      else
      {
        Serial.println("[CMD] UNKNOWN_TURN_PARAMETER");
        return;
      }
    }
    else if (strcmp(domain, "climb") == 0)
    {
      if (strcmp(parameter, "ff") == 0)
      {
        climbModeManager.setForwardPitchFeedforwardDegrees(value);
      }
      else if (strcmp(parameter, "ki") == 0)
      {
        climbModeManager.setSpeedIntegralGain(value);
      }
      else if (strcmp(parameter, "max_pitch") == 0)
      {
        climbModeManager.setMaximumPitchOffsetDegrees(value);
      }
      else if (strcmp(parameter, "max_motor") == 0)
      {
        climbModeManager.setMaximumMotorCommand(value);
      }
      else if (strcmp(parameter, "max_speed") == 0)
      {
        climbModeManager.setMaximumTargetSpeedMps(value);
      }
      else if (strcmp(parameter, "max_turn") == 0)
      {
        climbModeManager.setMaximumTurnMotorCommand(value);
      }
      else if (strcmp(parameter, "invert") == 0)
      {
        climbModeManager.setOutputInverted(value >= 0.5F);
      }
      else
      {
        Serial.println("[CMD] UNKNOWN_CLIMB_PARAMETER");
        return;
      }
    }
    else if (strcmp(domain, "motion") == 0)
    {
      if (strcmp(parameter, "speed") == 0)
      {
        cancelWifiDriveSlew();
        motionCommand.setTargetSpeedMps(value);
      }
      else if (strcmp(parameter, "turn") == 0)
      {
        motionCommand.setTargetDifferentialSpeedMps(value);
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
      cancelWifiDriveSlew();
      motionCommand.adjustTargetSpeedMps(balance_car::config::kMotionConfiguration.targetSpeedStepMps);
      printMotionCommand();
      break;
    case 'z':
      cancelWifiDriveSlew();
      motionCommand.adjustTargetSpeedMps(-balance_car::config::kMotionConfiguration.targetSpeedStepMps);
      printMotionCommand();
      break;
    case 'c':
      cancelWifiDriveSlew();
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
    if (strcmp(commandLine, "climb on") == 0)
    {
      if (!requestClimbMode(true))
      {
        Serial.println("[CLIMB] REJECTED=NOT_BALANCING");
      }
      return;
    }
    if (strcmp(commandLine, "climb off") == 0)
    {
      requestClimbMode(false);
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
    if (event == balance_car::app::OfflineArmEvent::StartBalance)
    {
      requestBalance();
    }
    else if (event == balance_car::app::OfflineArmEvent::StopBalance)
    {
      stopMotorOutput();
      Serial.println("[OFFLINE] MOTOR_OUTPUT=STOPPED");
    }
  }

  bool applyWifiTuningCommand(const balance_car::app::WifiTuningCommand &command)
  {
    if (strcmp(command.domain, "balance") == 0)
    {
      if (strcmp(command.parameter, "kp") == 0)
        balanceController.setProportionalGain(command.value);
      else if (strcmp(command.parameter, "ki") == 0)
        balanceController.setIntegralGain(command.value);
      else if (strcmp(command.parameter, "kd") == 0)
        balanceController.setDerivativeGain(command.value);
      else if (strcmp(command.parameter, "trim") == 0)
        balanceController.setTargetPitchDegrees(command.value);
      else if (strcmp(command.parameter, "max_motor") == 0)
        balanceController.setMaximumMotorCommand(command.value);
      else
        return false;
    }
    else if (strcmp(command.domain, "speed") == 0)
    {
      if (strcmp(command.parameter, "kp") == 0)
        velocityController.setProportionalGain(command.value);
      else if (strcmp(command.parameter, "ki") == 0)
        velocityController.setIntegralGain(command.value);
      else if (strcmp(command.parameter, "max_pitch") == 0)
        velocityController.setMaximumPitchOffsetDegrees(command.value);
      else
        return false;
    }
    else if (strcmp(command.domain, "turn") == 0)
    {
      if (strcmp(command.parameter, "kp") == 0)
        differentialSpeedController.setProportionalGain(command.value);
      else if (strcmp(command.parameter, "ki") == 0)
        differentialSpeedController.setIntegralGain(command.value);
      else if (strcmp(command.parameter, "max") == 0)
        differentialSpeedController.setMaximumTurnMotorCommand(command.value);
      else
        return false;
    }
    else if (strcmp(command.domain, "climb") == 0)
    {
      if (strcmp(command.parameter, "ff") == 0)
        climbModeManager.setForwardPitchFeedforwardDegrees(command.value);
      else if (strcmp(command.parameter, "ki") == 0)
        climbModeManager.setSpeedIntegralGain(command.value);
      else if (strcmp(command.parameter, "max_pitch") == 0)
        climbModeManager.setMaximumPitchOffsetDegrees(command.value);
      else if (strcmp(command.parameter, "max_motor") == 0)
        climbModeManager.setMaximumMotorCommand(command.value);
      else if (strcmp(command.parameter, "max_speed") == 0)
        climbModeManager.setMaximumTargetSpeedMps(command.value);
      else if (strcmp(command.parameter, "max_turn") == 0)
        climbModeManager.setMaximumTurnMotorCommand(command.value);
      else if (strcmp(command.parameter, "invert") == 0)
        climbModeManager.setOutputInverted(command.value >= 0.5F);
      else
        return false;
    }
    else if (strcmp(command.domain, "landing") == 0)
    {
      // Landing criteria are intentionally immutable during BALANCING. The
      // operator can tune while disarmed, then arm for one controlled trial.
      if (safetyManager.state() != balance_car::app::SafetyState::Standby)
      {
        return false;
      }
      if (strcmp(command.parameter, "airborne_g") == 0)
        airborneLandingManager.setAirborneAccelerationThresholdG(command.value);
      else if (strcmp(command.parameter, "confirm_ms") == 0)
        airborneLandingManager.setAirborneConfirmationMs(millisecondsFromCommandValue(command.value));
      else if (strcmp(command.parameter, "max_airborne_ms") == 0)
        airborneLandingManager.setMaximumAirborneMs(millisecondsFromCommandValue(command.value));
      else if (strcmp(command.parameter, "landing_min_g") == 0)
        airborneLandingManager.setLandingAccelerationMinimumG(command.value);
      else if (strcmp(command.parameter, "landing_max_g") == 0)
        airborneLandingManager.setLandingAccelerationMaximumG(command.value);
      else if (strcmp(command.parameter, "settle_ms") == 0)
        airborneLandingManager.setLandingSettleMs(millisecondsFromCommandValue(command.value));
      else if (strcmp(command.parameter, "timeout_ms") == 0)
        airborneLandingManager.setLandingRecoveryTimeoutMs(millisecondsFromCommandValue(command.value));
      else if (strcmp(command.parameter, "ramp_ms") == 0)
        airborneLandingManager.setMotorRecoveryRampMs(millisecondsFromCommandValue(command.value));
      else
        return false;
    }
    else
    {
      return false;
    }

    Serial.printf("[WIFI] SET %s %s=%.5f\n", command.domain, command.parameter, command.value);
    return true;
  }

  float wifiTuningValue(const balance_car::app::WifiTuningCommand &command)
  {
    const balance_car::control::BalanceTuning balanceTuning = balanceController.tuning();
    const balance_car::control::VelocityTuning velocityTuning = velocityController.tuning();
    const balance_car::control::DifferentialSpeedTuning turnTuning = differentialSpeedController.tuning();
    if (strcmp(command.domain, "balance") == 0)
    {
      if (strcmp(command.parameter, "kp") == 0) return balanceTuning.proportionalGain;
      if (strcmp(command.parameter, "ki") == 0) return balanceTuning.integralGain;
      if (strcmp(command.parameter, "kd") == 0) return balanceTuning.derivativeGain;
      if (strcmp(command.parameter, "trim") == 0) return balanceTuning.targetPitchDegrees;
      if (strcmp(command.parameter, "max_motor") == 0) return balanceTuning.maximumMotorCommand;
    }
    if (strcmp(command.domain, "speed") == 0)
    {
      if (strcmp(command.parameter, "kp") == 0) return velocityTuning.proportionalGain;
      if (strcmp(command.parameter, "ki") == 0) return velocityTuning.integralGain;
      if (strcmp(command.parameter, "max_pitch") == 0) return velocityTuning.maximumPitchOffsetDegrees;
    }
    if (strcmp(command.domain, "turn") == 0)
    {
      if (strcmp(command.parameter, "kp") == 0) return turnTuning.proportionalGain;
      if (strcmp(command.parameter, "ki") == 0) return turnTuning.integralGain;
      if (strcmp(command.parameter, "max") == 0) return turnTuning.maximumTurnMotorCommand;
    }
    const balance_car::app::ClimbModeTuning climbTuning = climbModeManager.tuning();
    if (strcmp(command.domain, "climb") == 0)
    {
      if (strcmp(command.parameter, "ff") == 0) return climbTuning.forwardPitchFeedforwardDegrees;
      if (strcmp(command.parameter, "ki") == 0) return climbTuning.speedIntegralGain;
      if (strcmp(command.parameter, "max_pitch") == 0) return climbTuning.maximumPitchOffsetDegrees;
      if (strcmp(command.parameter, "max_motor") == 0) return climbTuning.maximumMotorCommand;
      if (strcmp(command.parameter, "max_speed") == 0) return climbTuning.maximumTargetSpeedMps;
      if (strcmp(command.parameter, "max_turn") == 0) return climbTuning.maximumTurnMotorCommand;
      if (strcmp(command.parameter, "invert") == 0) return climbTuning.outputInverted ? 1.0F : 0.0F;
    }
    const balance_car::app::AirborneLandingTuning landingTuning = airborneLandingManager.tuning();
    if (strcmp(command.domain, "landing") == 0)
    {
      if (strcmp(command.parameter, "airborne_g") == 0) return landingTuning.airborneAccelerationThresholdG;
      if (strcmp(command.parameter, "confirm_ms") == 0) return landingTuning.airborneConfirmationMs;
      if (strcmp(command.parameter, "max_airborne_ms") == 0) return landingTuning.maximumAirborneMs;
      if (strcmp(command.parameter, "landing_min_g") == 0) return landingTuning.landingAccelerationMinimumG;
      if (strcmp(command.parameter, "landing_max_g") == 0) return landingTuning.landingAccelerationMaximumG;
      if (strcmp(command.parameter, "settle_ms") == 0) return landingTuning.landingSettleMs;
      if (strcmp(command.parameter, "timeout_ms") == 0) return landingTuning.landingRecoveryTimeoutMs;
      if (strcmp(command.parameter, "ramp_ms") == 0) return landingTuning.motorRecoveryRampMs;
    }
    return 0.0F;
  }

  void processWifiTuning()
  {
    wifiDebugServer.service();
    balance_car::app::WifiTuningCommand command;
    while (wifiDebugServer.takeTuningCommand(command))
    {
      if (command.kind == balance_car::app::WifiCommandKind::Arm)
      {
        const bool accepted = requestBalance();
        wifiDebugServer.sendCommandResult(command.requestSequence, accepted,
                                          accepted ? "ARMED" : "ARM_REJECTED");
      }
      else if (command.kind == balance_car::app::WifiCommandKind::Stop)
      {
        stopMotorOutput();
        Serial.println("[WIFI] MOTOR_OUTPUT=STOPPED");
        wifiDebugServer.sendCommandResult(command.requestSequence, true, "STOPPED");
      }
      else if (command.kind == balance_car::app::WifiCommandKind::Reset)
      {
        // Match a physical RESET press: stop the current process and restart
        // the MCU, including all safety/control state and Wi-Fi services.
        stopMotorOutput();
        wifiDebugServer.sendCommandResult(command.requestSequence, true, "RESTARTING");
        Serial.println("[WIFI] RESET=RESTARTING");
        delay(80);
        ESP.restart();
      }
      else if (command.kind == balance_car::app::WifiCommandKind::Drive)
      {
        const bool accepted = requestWifiDriveSpeed(command.value);
        wifiDebugServer.sendCommandResult(command.requestSequence, accepted,
                                          accepted ? "DRIVE_ACCEPTED" :
                                                     (safetyManager.isBalancing() ? "SPEED_RANGE" : "NOT_BALANCING"));
      }
      else if (command.kind == balance_car::app::WifiCommandKind::Turn)
      {
        if (safetyManager.isBalancing())
        {
          motionCommand.setTargetDifferentialSpeedMps(command.value);
          wifiDebugServer.sendCommandResult(command.requestSequence, true, "TURN_ACCEPTED");
        }
        else
        {
          wifiDebugServer.sendCommandResult(command.requestSequence, false, "NOT_BALANCING");
        }
      }
      else if (command.kind == balance_car::app::WifiCommandKind::ClimbMode)
      {
        const bool accepted = requestClimbMode(command.value >= 0.5F);
        wifiDebugServer.sendCommandResult(command.requestSequence, accepted,
                                          accepted ? (command.value >= 0.5F ? "CLIMB_ON" : "CLIMB_OFF")
                                                   : "NOT_BALANCING");
      }
      else if (command.kind == balance_car::app::WifiCommandKind::AirborneLandingMode)
      {
        const bool enabled = command.value >= 0.5F;
        const bool accepted = requestAirborneLandingMode(enabled);
        wifiDebugServer.sendCommandResult(command.requestSequence, accepted,
                                          accepted ? (enabled ? "LANDING_ON" : "LANDING_OFF")
                                                   : "STANDBY_REQUIRED");
      }
      else
      {
        const bool accepted = applyWifiTuningCommand(command);
        char reason[96] = {};
        if (accepted)
        {
          // The ACK is the authoritative transaction result. Echo the exact
          // post-clamp controller value so the host need not infer success
          // from an unrelated, potentially delayed telemetry packet.
          snprintf(reason, sizeof(reason), "APPLIED,%s,%s,%.5f",
                   command.domain, command.parameter, wifiTuningValue(command));
        }
        wifiDebugServer.sendCommandResult(command.requestSequence, accepted,
                                          accepted ? reason : "REJECTED");
      }
    }
  }

  void publishWifiTelemetry(uint32_t nowMs)
  {
    const balance_car::control::VelocityState velocityState = velocityController.state();
    const balance_car::control::DifferentialSpeedState turnState = differentialSpeedController.state();
    const balance_car::control::BalanceTuning balanceTuning = balanceController.tuning();
    const balance_car::control::BalanceState balanceState = balanceController.state();
    const balance_car::control::VelocityTuning velocityTuning = velocityController.tuning();
    const balance_car::control::DifferentialSpeedTuning turnTuning = differentialSpeedController.tuning();
    balance_car::app::WifiTelemetry telemetry = {};
    telemetry.timestampMs = nowMs;
    telemetry.safetyState = static_cast<uint8_t>(safetyManager.state());
    telemetry.faultCode = static_cast<uint8_t>(safetyManager.faultCode());
    telemetry.imuValid = latestImuSample.valid && latestAttitude.valid;
    telemetry.pitchDegrees = latestAttitude.pitchDegrees;
    telemetry.pitchRateDps = latestAttitude.pitchRateDps;
    telemetry.accelerometerPitchDegrees = latestAttitude.accelerometerPitchDegrees;
    telemetry.accelXG = latestImuSample.accelXG;
    telemetry.accelYG = latestImuSample.accelYG;
    telemetry.accelZG = latestImuSample.accelZG;
    telemetry.gyroXDps = latestImuSample.gyroXDps;
    telemetry.gyroYDps = latestImuSample.gyroYDps;
    telemetry.gyroZDps = latestImuSample.gyroZDps;
    telemetry.targetSpeedMps = latestEffectiveTargetSpeedMps;
    telemetry.filteredSpeedMps = velocityState.filteredSpeedMps;
    telemetry.speedErrorMps = velocityState.speedErrorMps;
    telemetry.speedPitchOffsetDegrees = latestVelocityPitchOffsetDegrees;
    telemetry.turnCommand = latestEffectiveDifferentialSpeedMps;
    telemetry.filteredDifferentialSpeedMps = turnState.filteredDifferentialSpeedMps;
    telemetry.differentialSpeedErrorMps = turnState.differentialSpeedErrorMps;
    telemetry.turnMotorCommand = latestTurnMotorCommand;
    telemetry.appliedTurnMotorCommand = latestMixedMotorCommand.appliedTurnCommand;
    telemetry.leftMotorCommand = latestMixedMotorCommand.left;
    telemetry.rightMotorCommand = latestMixedMotorCommand.right;
    telemetry.leftWheelSpeedMps = latestWheelSpeed.leftMetersPerSecond;
    telemetry.rightWheelSpeedMps = latestWheelSpeed.rightMetersPerSecond;
    telemetry.requestedPitchDegrees = balanceState.requestedPitchDegrees;
    telemetry.balancePitchErrorDegrees = balanceState.pitchErrorDegrees;
    telemetry.balanceProportionalTerm = balanceState.proportionalTerm;
    telemetry.balanceIntegralTerm = balanceState.integralTerm;
    telemetry.balanceDerivativeTerm = balanceState.derivativeTerm;
    telemetry.balanceMotorRaw = balanceState.motorCommandRaw;
    telemetry.balanceKp = balanceTuning.proportionalGain;
    telemetry.balanceKi = balanceTuning.integralGain;
    telemetry.balanceKd = balanceTuning.derivativeGain;
    telemetry.balanceTrimDegrees = balanceTuning.targetPitchDegrees;
    telemetry.speedKp = velocityTuning.proportionalGain;
    telemetry.speedKi = velocityTuning.integralGain;
    telemetry.speedInverted = velocityTuning.outputInverted;
    telemetry.turnKp = turnTuning.proportionalGain;
    telemetry.turnKi = turnTuning.integralGain;
    telemetry.maximumTurnMotorCommand = turnTuning.maximumTurnMotorCommand;
    telemetry.turnInverted = turnTuning.outputInverted;
    const balance_car::app::ClimbModeTuning climbTuning = climbModeManager.tuning();
    telemetry.maximumMotorCommand = latestClimbModeOutput.active
                                        ? latestClimbModeOutput.maximumMotorCommand
                                        : balanceTuning.maximumMotorCommand;
    telemetry.maximumPitchOffsetDegrees = latestClimbModeOutput.active
                                              ? climbTuning.maximumPitchOffsetDegrees
                                              : velocityTuning.maximumPitchOffsetDegrees;
    telemetry.headingDegrees = differentialOdometry.state().headingDegrees;
    telemetry.yawRateDegreesPerSecond = differentialOdometry.state().yawRateDegreesPerSecond;
    telemetry.airborneLandingEnabled = airborneLandingManager.isEnabled();
    telemetry.airborneLandingState = static_cast<uint8_t>(airborneLandingManager.state());
    telemetry.accelerationMagnitudeG =
        balance_car::app::AirborneLandingManager::accelerationMagnitudeG(latestImuSample);
    const balance_car::app::AirborneLandingTuning landingTuning = airborneLandingManager.tuning();
    telemetry.airborneAccelerationThresholdG = landingTuning.airborneAccelerationThresholdG;
    telemetry.airborneConfirmationMs = landingTuning.airborneConfirmationMs;
    telemetry.maximumAirborneMs = landingTuning.maximumAirborneMs;
    telemetry.landingAccelerationMinimumG = landingTuning.landingAccelerationMinimumG;
    telemetry.landingAccelerationMaximumG = landingTuning.landingAccelerationMaximumG;
    telemetry.landingSettleMs = landingTuning.landingSettleMs;
    telemetry.landingRecoveryTimeoutMs = landingTuning.landingRecoveryTimeoutMs;
    telemetry.motorRecoveryRampMs = landingTuning.motorRecoveryRampMs;
    wifiDebugServer.publish(telemetry, nowMs);
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
    if (!safetyManager.isBalancing() || !airborneLandingManager.allowMotionControl())
    {
      velocityController.reset();
      differentialSpeedController.reset();
      differentialOdometry.reset();
      climbModeManager.reset();
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
      latestClimbModeOutput = {};
      latestEffectiveTargetSpeedMps = 0.0F;
      latestEffectiveDifferentialSpeedMps = 0.0F;
      return;
    }
    const float measuredSpeedMps = 0.5F *
                                   (latestWheelSpeed.leftMetersPerSecond + latestWheelSpeed.rightMetersPerSecond);
    const float deltaSeconds = static_cast<float>(elapsedMs) / 1000.0F;
    updateWifiDriveTarget(deltaSeconds);
    latestEffectiveTargetSpeedMps =
        climbModeManager.limitTargetSpeedMps(motionCommand.targetSpeedMps());
    const float normalVelocityPitchOffsetDegrees = velocityController.update(
        latestEffectiveTargetSpeedMps, measuredSpeedMps, deltaSeconds);
    latestClimbModeOutput = climbModeManager.update(
        latestEffectiveTargetSpeedMps, velocityController.state().filteredSpeedMps,
        normalVelocityPitchOffsetDegrees, deltaSeconds);
    latestVelocityPitchOffsetDegrees = latestClimbModeOutput.pitchOffsetDegrees;
    latestEffectiveDifferentialSpeedMps =
        climbModeManager.limitDifferentialSpeedMps(motionCommand.targetDifferentialSpeedMps());
    latestTurnMotorCommand = differentialSpeedController.update(
        latestEffectiveDifferentialSpeedMps, latestWheelSpeed.leftMetersPerSecond,
        latestWheelSpeed.rightMetersPerSecond, deltaSeconds,
        latestClimbModeOutput.active ? latestClimbModeOutput.maximumTurnMotorCommand : -1.0F);
    differentialOdometry.update(latestWheelSpeed, deltaSeconds);
  }

  void updateBalanceControl(uint32_t nowMs)
  {
    if (nowMs - lastControlUpdateMs < balance_car::config::kBalanceConfiguration.controlPeriodMs)
    {
      return;
    }

    lastControlUpdateMs = nowMs;
    latestImuSample = imuDriver.read();
    handleAirborneLandingEvent(airborneLandingManager.update(latestImuSample, nowMs));
    latestAttitude = attitudeEstimator.update(
        latestImuSample, airborneLandingManager.useAccelerometerCorrection());
    safetyManager.monitorBalance(latestAttitude.pitchDegrees, latestAttitude.valid, imuDriver.isHealthy(),
                                 airborneLandingManager.enforcePitchLimit());

    if (!safetyManager.isBalancing())
    {
      latestBalanceMotorCommand = 0.0F;
      latestTurnMotorCommand = 0.0F;
      latestMixedMotorCommand = {};
      return;
    }

    if (airborneLandingManager.holdMotorOutput())
    {
      latestBalanceMotorCommand = 0.0F;
      latestTurnMotorCommand = 0.0F;
      latestMixedMotorCommand = {};
      motorDriver.setNormalized(0.0F, 0.0F);
      return;
    }

    const float maximumMotorCommand = latestClimbModeOutput.active
                                          ? latestClimbModeOutput.maximumMotorCommand
                                          : balanceController.tuning().maximumMotorCommand;
    latestBalanceMotorCommand = balanceController.update(
        latestAttitude, latestVelocityPitchOffsetDegrees, maximumMotorCommand);
    latestMixedMotorCommand = balance_car::control::MotorMixer::mix(
        latestBalanceMotorCommand, latestTurnMotorCommand,
        maximumMotorCommand * airborneLandingManager.motorOutputScale(nowMs));
    motorDriver.setNormalized(latestMixedMotorCommand.left, latestMixedMotorCommand.right);
  }

  void printTelemetry(uint32_t nowMs)
  {
    if (nowMs - lastTelemetryMs < kTelemetryPeriodMs)
    {
      return;
    }

    lastTelemetryMs = nowMs;
    const balance_car::control::VelocityState velocityState = velocityController.state();
    const balance_car::control::DifferentialSpeedState turnState = differentialSpeedController.state();
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
    Serial.print(latestEffectiveTargetSpeedMps, 3);
    Serial.print(" measured_speed_mps=");
    Serial.print(velocityState.filteredSpeedMps, 3);
    Serial.print(" speed_error_mps=");
    Serial.print(velocityState.speedErrorMps, 3);
    Serial.print(" speed_pitch_offset_deg=");
    Serial.print(latestVelocityPitchOffsetDegrees, 3);
    Serial.print(" target_diff_speed_mps=");
    Serial.print(latestEffectiveDifferentialSpeedMps, 3);
    Serial.print(" measured_diff_speed_mps=");
    Serial.print(turnState.filteredDifferentialSpeedMps, 3);
    Serial.print(" turn_output_requested_applied=");
    Serial.print(latestTurnMotorCommand, 3);
    Serial.print(',');
    Serial.print(latestMixedMotorCommand.appliedTurnCommand, 3);
    Serial.print(" motor_lr=");
    Serial.print(latestMixedMotorCommand.left, 3);
    Serial.print(',');
    Serial.print(latestMixedMotorCommand.right, 3);
    Serial.print(" wheel_mps_lr=");
    Serial.print(latestWheelSpeed.leftMetersPerSecond, 3);
    Serial.print(',');
    Serial.print(latestWheelSpeed.rightMetersPerSecond, 3);
    Serial.print(" tick_delta_lr=");
    Serial.print(latestWheelSpeed.leftTickDelta);
    Serial.print(',');
    Serial.print(latestWheelSpeed.rightTickDelta);
    Serial.print(" ticks_lr=");
    Serial.print(latestWheelSpeed.leftTicks);
    Serial.print(',');
    Serial.print(latestWheelSpeed.rightTicks);
    Serial.print(" climb=requested/active,ff,i=");
    Serial.print(latestClimbModeOutput.requested ? 1 : 0);
    Serial.print(',');
    Serial.print(latestClimbModeOutput.active ? 1 : 0);
    Serial.print(',');
    Serial.print(latestClimbModeOutput.feedforwardPitchDegrees, 3);
    Serial.print(',');
    Serial.println(latestClimbModeOutput.integralPitchDegrees, 3);
  }
} // namespace

void setup()
{
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("===== Balance Car Cascaded Control =====");
  offlineArmControl.begin();
  safetyManager.begin();
  const balance_car::app::SelfTestReport report = selfTest.run();
  balance_car::app::SelfTest::printReport(Serial, report);
  safetyManager.completeSelfTest(report);
  wifiDebugServer.begin();
  printStatus();
  printHelp();
}

void loop()
{
  const uint32_t nowMs = millis();
  processSerialInput();
  processWifiTuning();
  safetyManager.update(nowMs);
  processOfflineArmControl(nowMs);
  updateVelocityControl(nowMs);
  updateBalanceControl(nowMs);
  reportSafetyFaultTransition();
  printTelemetry(nowMs);
  publishWifiTelemetry(nowMs);
  delay(1);
}

#undef Serial
