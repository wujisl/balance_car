#include <Arduino.h>

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "app/airborne_landing_manager.h"
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
  balance_car::app::OfflineArmControl offlineArmControl(
      balance_car::config::kBalanceArmButtonPin, balance_car::config::kSafetyConfiguration.offlineArmHoldMs);
  balance_car::app::WifiDebugServer wifiDebugServer(balance_car::config::kWifiDebugConfiguration);
  TwoWire visionWire(1);
  balance_car::drivers::VisionI2cClient visionI2cClient(visionWire, balance_car::config::kVisionI2cPins);

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
  bool visionTrackingEnabled = false;
  bool visionCommandAccepted = false;
  bool visionCommandTransportHeld = false;
  float lastVisionDifferentialTargetMps = 0.0F;
  uint32_t lastVisionCommandMs = 0;
  // Keep the I2C state exchange fast (50 Hz), but do not let every camera
  // result replace a differential-speed step before the turn loop settles.
  // Apply a new filtered target every 400 ms by default. I2C state exchange
  // remains at 50 Hz and continues feeding the weighted sample window.
  constexpr uint32_t kDefaultVisionTargetUpdatePeriodMs = 400U;
  constexpr uint8_t kVisionTargetWindowSize = 5U;
  uint32_t visionTargetUpdatePeriodMs = kDefaultVisionTargetUpdatePeriodMs;
  bool visionTargetFilterEnabled = true;
  uint16_t visionTargetMaximumStepMmps = 0;  // 0 means no slew limit.
  float filteredVisionDifferentialTargetMps = 0.0F;
  bool visionTargetFilterInitialized = false;
  float visionTargetWindowMps[kVisionTargetWindowSize] = {};
  uint8_t visionTargetWindowCount = 0U;
  uint16_t lastVisionBufferedSampleSequence = 0;
  uint32_t lastVisionTargetUpdateMs = 0;
  // When a confirmed curve leaves the short camera view, keep the maximum
  // differential-speed target in the last reliable direction. A newly valid
  // line releases the hold immediately.
  bool visionCurveHoldTestEnabled = true;
  uint16_t visionCurveHoldTargetMmps = 120;
  bool visionCurveHoldLatched = false;
  float visionCurveHoldLatchedMps = 0.0F;
  int8_t visionCurveCandidateSign = 0;
  uint8_t visionCurveCandidateCount = 0;
  uint16_t lastVisionCurveDetectionSequence = 0;
  float wifiDriveRequestedSpeedMps = 0.0F;
  bool wifiDriveSlewActive = false;
  uint32_t lastControlUpdateMs = 0;
  uint32_t lastVelocityUpdateMs = 0;
  uint16_t latestBalanceControlPeriodMs = 0;
  uint16_t latestVelocityControlPeriodMs = 0;
  uint32_t lastTelemetryMs = 0;
  uint32_t lastVisionI2cMs = 0;
  uint16_t visionChassisSequence = 0;
  balance_car::drivers::VisionSample latestVisionSample = {};
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
    Serial.print(motionCommand.targetSpeedMps(), 3);
    Serial.print(" FILTERED_SPEED=");
    Serial.print(velocityState.filteredSpeedMps, 3);
    Serial.print(" TARGET_DIFF_SPEED=");
    Serial.print(motionCommand.targetDifferentialSpeedMps(), 3);
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
      motionCommand.clear();
      visionCommandAccepted = false;
      visionCommandTransportHeld = false;
      lastVisionDifferentialTargetMps = 0.0F;
      lastVisionCommandMs = 0;
      lastVisionBufferedSampleSequence = 0;
      lastVisionTargetUpdateMs = 0;
      filteredVisionDifferentialTargetMps = 0.0F;
      visionTargetFilterInitialized = false;
      motionCommand.setTargetSpeedMps(balance_car::config::kMotionConfiguration.initialTargetSpeedMps);
      wifiDriveRequestedSpeedMps = 0.0F;
      wifiDriveSlewActive = false;
      latestBalanceMotorCommand = 0.0F;
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
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
    motionCommand.clear();
    visionCommandAccepted = false;
    visionCommandTransportHeld = false;
    lastVisionDifferentialTargetMps = 0.0F;
    lastVisionCommandMs = 0;
    lastVisionBufferedSampleSequence = 0;
    lastVisionTargetUpdateMs = 0;
    filteredVisionDifferentialTargetMps = 0.0F;
    visionTargetFilterInitialized = false;
    wifiDriveRequestedSpeedMps = 0.0F;
    wifiDriveSlewActive = false;
    latestBalanceMotorCommand = 0.0F;
    latestVelocityPitchOffsetDegrees = 0.0F;
    latestTurnMotorCommand = 0.0F;
    latestMixedMotorCommand = {};
  }

  void cancelWifiDriveSlew()
  {
    wifiDriveRequestedSpeedMps = 0.0F;
    wifiDriveSlewActive = false;
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
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
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
    else if (strcmp(command.domain, "vision") == 0)
    {
      if (strcmp(command.parameter, "period_ms") == 0)
        visionTargetUpdatePeriodMs = static_cast<uint32_t>(
            constrain(lroundf(command.value), 100L, 5000L));
      else if (strcmp(command.parameter, "filter") == 0)
      {
        visionTargetFilterEnabled = command.value >= 0.5F;
        filteredVisionDifferentialTargetMps = 0.0F;
        visionTargetFilterInitialized = false;
        visionTargetWindowCount = 0U;
      }
      else if (strcmp(command.parameter, "max_step_mmps") == 0)
        visionTargetMaximumStepMmps = static_cast<uint16_t>(
            constrain(lroundf(command.value), 0L, 200L));
      else if (strcmp(command.parameter, "curve_hold") == 0)
      {
        visionCurveHoldTestEnabled = command.value >= 0.5F;
        if (!visionCurveHoldTestEnabled)
        {
          visionCurveHoldLatched = false;
          visionCurveHoldLatchedMps = 0.0F;
          visionCurveCandidateSign = 0;
          visionCurveCandidateCount = 0;
        }
      }
      else if (strcmp(command.parameter, "curve_hold_mmps") == 0)
        visionCurveHoldTargetMmps = static_cast<uint16_t>(
            constrain(lroundf(command.value), 20L, 200L));
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
    if (strcmp(command.domain, "vision") == 0 && strcmp(command.parameter, "period_ms") == 0)
    {
      return static_cast<float>(visionTargetUpdatePeriodMs);
    }
    if (strcmp(command.domain, "vision") == 0 && strcmp(command.parameter, "filter") == 0)
    {
      return visionTargetFilterEnabled ? 1.0F : 0.0F;
    }
    if (strcmp(command.domain, "vision") == 0 && strcmp(command.parameter, "max_step_mmps") == 0)
    {
      return static_cast<float>(visionTargetMaximumStepMmps);
    }
    if (strcmp(command.domain, "vision") == 0 && strcmp(command.parameter, "curve_hold") == 0)
    {
      return visionCurveHoldTestEnabled ? 1.0F : 0.0F;
    }
    if (strcmp(command.domain, "vision") == 0 && strcmp(command.parameter, "curve_hold_mmps") == 0)
    {
      return static_cast<float>(visionCurveHoldTargetMmps);
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
      else if (command.kind == balance_car::app::WifiCommandKind::MotorTest)
      {
        // The SafetyManager admits this only from STANDBY/MANUAL_TEST, limits
        // its magnitude, and drops motor enable after a one-second heartbeat
        // timeout.  The desktop host refreshes the command while a test runs.
        const bool accepted = safetyManager.requestManualMotorTest(
            command.value, command.value2, millis());
        if (accepted)
        {
          latestMixedMotorCommand.left = command.value;
          latestMixedMotorCommand.right = command.value2;
          latestBalanceMotorCommand = 0.0F;
          latestTurnMotorCommand = 0.0F;
        }
        wifiDebugServer.sendCommandResult(command.requestSequence, accepted,
                                          accepted ? "MOTOR_TEST_ACTIVE" : "MOTOR_TEST_REJECTED");
      }
      else if (command.kind == balance_car::app::WifiCommandKind::CalibrateImu)
      {
        // This operation samples for approximately one second, so it must be
        // performed with the vehicle motionless and motors disabled.
        const bool inStandby = safetyManager.state() == balance_car::app::SafetyState::Standby;
        const bool calibrated = inStandby && imuDriver.calibrateGyroscope();
        if (calibrated)
        {
          attitudeEstimator.reset();
          latestImuSample = imuDriver.read();
          latestAttitude = attitudeEstimator.update(latestImuSample, true);
        }
        wifiDebugServer.sendCommandResult(command.requestSequence, calibrated,
                                          calibrated ? "GYRO_CALIBRATED" :
                                                       (inStandby ? "GYRO_NOT_STATIONARY" : "NOT_STANDBY"));
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
          visionTrackingEnabled = false;
          visionCommandAccepted = false;
          visionCommandTransportHeld = false;
          lastVisionDifferentialTargetMps = 0.0F;
          lastVisionCommandMs = 0;
          lastVisionBufferedSampleSequence = 0;
          lastVisionTargetUpdateMs = 0;
          filteredVisionDifferentialTargetMps = 0.0F;
          visionTargetFilterInitialized = false;
          visionCurveHoldLatched = false;
          visionCurveHoldLatchedMps = 0.0F;
          visionCurveCandidateSign = 0;
          visionCurveCandidateCount = 0;
          lastVisionCurveDetectionSequence = 0;
          differentialSpeedController.reset();
          motionCommand.setTargetDifferentialSpeedMps(command.value);
          wifiDebugServer.sendCommandResult(command.requestSequence, true, "TURN_ACCEPTED");
        }
        else
        {
          wifiDebugServer.sendCommandResult(command.requestSequence, false, "NOT_BALANCING");
        }
      }
      else if (command.kind == balance_car::app::WifiCommandKind::Track)
      {
        if (command.value >= 0.5F && !safetyManager.isBalancing())
        {
          wifiDebugServer.sendCommandResult(command.requestSequence, false, "NOT_BALANCING");
        }
        else
        {
          visionTrackingEnabled = command.value >= 0.5F;
          visionCommandAccepted = false;
          visionCommandTransportHeld = false;
          lastVisionDifferentialTargetMps = 0.0F;
          lastVisionCommandMs = 0;
          lastVisionBufferedSampleSequence = 0;
          lastVisionTargetUpdateMs = 0;
          filteredVisionDifferentialTargetMps = 0.0F;
          visionTargetFilterInitialized = false;
          visionCurveHoldLatched = false;
          visionCurveHoldLatchedMps = 0.0F;
          visionCurveCandidateSign = 0;
          visionCurveCandidateCount = 0;
          lastVisionCurveDetectionSequence = 0;
          // A previous track/manual turn must not leave its filtered speed or
          // integral term active after the steering ownership changes.
          differentialSpeedController.reset();
          motionCommand.setTargetDifferentialSpeedMps(0.0F);
          Serial.printf("[TRACK] TRACKING=%s (WIFI)\n", visionTrackingEnabled ? "ON" : "OFF");
          wifiDebugServer.sendCommandResult(command.requestSequence, true,
                                            visionTrackingEnabled ? "TRACKING_ON" : "TRACKING_OFF");
        }
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
    telemetry.targetSpeedMps = motionCommand.targetSpeedMps();
    telemetry.filteredSpeedMps = velocityState.filteredSpeedMps;
    telemetry.speedErrorMps = velocityState.speedErrorMps;
    telemetry.speedPitchOffsetDegrees = latestVelocityPitchOffsetDegrees;
    telemetry.turnCommand = motionCommand.targetDifferentialSpeedMps();
    telemetry.filteredDifferentialSpeedMps = turnState.filteredDifferentialSpeedMps;
    telemetry.differentialSpeedErrorMps = turnState.differentialSpeedErrorMps;
    telemetry.turnMotorCommand = latestTurnMotorCommand;
    telemetry.appliedTurnMotorCommand = latestMixedMotorCommand.appliedTurnCommand;
    telemetry.leftMotorCommand = latestMixedMotorCommand.left;
    telemetry.rightMotorCommand = latestMixedMotorCommand.right;
    telemetry.leftWheelSpeedMps = latestWheelSpeed.leftMetersPerSecond;
    telemetry.rightWheelSpeedMps = latestWheelSpeed.rightMetersPerSecond;
    telemetry.leftEncoderTicks = latestWheelSpeed.leftTicks;
    telemetry.rightEncoderTicks = latestWheelSpeed.rightTicks;
    telemetry.leftEncoderTickDelta = latestWheelSpeed.leftTickDelta;
    telemetry.rightEncoderTickDelta = latestWheelSpeed.rightTickDelta;
    telemetry.leftEncoderTicksPerSecond = latestWheelSpeed.leftTicksPerSecond;
    telemetry.rightEncoderTicksPerSecond = latestWheelSpeed.rightTicksPerSecond;
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
    telemetry.maximumMotorCommand = balanceTuning.maximumMotorCommand;
    telemetry.maximumPitchOffsetDegrees = velocityTuning.maximumPitchOffsetDegrees;
    telemetry.headingDegrees = differentialOdometry.state().headingDegrees;
    telemetry.yawRateDegreesPerSecond = differentialOdometry.state().yawRateDegreesPerSecond;
    telemetry.visionTrackingEnabled = visionTrackingEnabled;
    telemetry.visionSampleFresh = visionI2cClient.isFresh();
    telemetry.visionCommandAccepted = visionCommandAccepted;
    telemetry.visionDeltaSpeedMps = latestVisionSample.deltaSpeedTargetMmps / 1000.0F;
    telemetry.visionTargetUpdatePeriodMs = static_cast<uint16_t>(visionTargetUpdatePeriodMs);
    telemetry.visionTargetFilterEnabled = visionTargetFilterEnabled;
    telemetry.visionTargetMaximumStepMmps = visionTargetMaximumStepMmps;
    telemetry.balanceInnerSaturated = balanceState.saturated;
    telemetry.velocityLoopSaturated = velocityState.saturated;
    telemetry.turnLoopSaturated = turnState.saturated;
    telemetry.encoderValid = encoderDriver.isInitialized();
    telemetry.imuCalibrated = imuDriver.isCalibrated();
    telemetry.balanceControlPeriodMs = latestBalanceControlPeriodMs;
    telemetry.velocityControlPeriodMs = latestVelocityControlPeriodMs;
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
    latestVelocityControlPeriodMs = static_cast<uint16_t>(elapsedMs > 65535U ? 65535U : elapsedMs);
    latestWheelSpeed = encoderDriver.sample(static_cast<float>(elapsedMs) / 1000.0F);
    if (!safetyManager.isBalancing() || !airborneLandingManager.allowMotionControl())
    {
      velocityController.reset();
      differentialSpeedController.reset();
      differentialOdometry.reset();
      latestVelocityPitchOffsetDegrees = 0.0F;
      latestTurnMotorCommand = 0.0F;
      return;
    }

    const float measuredSpeedMps = 0.5F *
                                   (latestWheelSpeed.leftMetersPerSecond + latestWheelSpeed.rightMetersPerSecond);
    updateWifiDriveTarget(static_cast<float>(elapsedMs) / 1000.0F);
    latestVelocityPitchOffsetDegrees = velocityController.update(
        motionCommand.targetSpeedMps(), measuredSpeedMps, static_cast<float>(elapsedMs) / 1000.0F);
    latestTurnMotorCommand = differentialSpeedController.update(
        motionCommand.targetDifferentialSpeedMps(), latestWheelSpeed.leftMetersPerSecond,
        latestWheelSpeed.rightMetersPerSecond, static_cast<float>(elapsedMs) / 1000.0F,
        latestMixedMotorCommand.appliedTurnCommand);
    differentialOdometry.update(latestWheelSpeed, static_cast<float>(elapsedMs) / 1000.0F);
  }

  // I2C v2 is a transport boundary: it sends measured wheel speeds to the
  // camera and accepts only the camera's right-minus-left speed setpoint.
  void updateVisionI2c(uint32_t nowMs)
  {
    if (nowMs - lastVisionI2cMs < 20U) return;
    lastVisionI2cMs = nowMs;
    balance_car::drivers::VisionChassisState state = {};
    state.sequence = ++visionChassisSequence;
    state.balancing = safetyManager.isBalancing();
    state.trackingEnabled = state.balancing && visionTrackingEnabled;
    state.wheelSpeedValid = encoderDriver.isInitialized();
    state.leftSpeedMmps = static_cast<int16_t>(constrain(lroundf(latestWheelSpeed.leftMetersPerSecond * 1000.0F), -32768L, 32767L));
    state.rightSpeedMmps = static_cast<int16_t>(constrain(lroundf(latestWheelSpeed.rightMetersPerSecond * 1000.0F), -32768L, 32767L));
    state.forwardTargetMmps = static_cast<int16_t>(constrain(lroundf(motionCommand.targetSpeedMps() * 1000.0F), -32768L, 32767L));
    state.timestampMs = nowMs;
    const bool exchanged = visionI2cClient.exchange(state, latestVisionSample);
    const bool usable = exchanged && latestVisionSample.trackValid && latestVisionSample.calibrated &&
                        latestVisionSample.qualityOk && !latestVisionSample.held;
    // This is the single hand-off from vision transport to the existing
    // differential-speed loop. A one-off I2C transaction failure should not
    // produce a 20 ms zero-steering notch. An explicit invalid/held camera
    // frame may enter the configured last-direction maximum-turn hold below.
    const bool controlEnabled = visionTrackingEnabled && state.balancing;
    const bool directAccepted = controlEnabled && usable;
    if (!controlEnabled)
    {
      visionCurveHoldLatched = false;
      visionCurveHoldLatchedMps = 0.0F;
      visionCurveCandidateSign = 0;
      visionCurveCandidateCount = 0;
    }
    // A genuinely valid line has returned. Release loss hold immediately and
    // allow this sample to bypass the normal 400 ms target-update gate.
    if (directAccepted && visionCurveHoldLatched)
    {
      visionCurveHoldLatched = false;
      visionCurveHoldLatchedMps = 0.0F;
      lastVisionTargetUpdateMs = 0U;
      filteredVisionDifferentialTargetMps = 0.0F;
      visionTargetFilterInitialized = false;
      visionTargetWindowCount = 0U;
    }
    if (directAccepted)
    {
      lastVisionCommandMs = nowMs;
      const bool newCameraSample = latestVisionSample.sequence != lastVisionBufferedSampleSequence;
      const bool updateDue = lastVisionTargetUpdateMs == 0U ||
                             nowMs - lastVisionTargetUpdateMs >= visionTargetUpdatePeriodMs;
      if (newCameraSample)
      {
        const float requestedMps = latestVisionSample.deltaSpeedTargetMmps / 1000.0F;
        if (!visionTargetFilterInitialized)
        {
          visionTargetWindowCount = 0U;
          visionTargetFilterInitialized = true;
        }
        if (visionTargetWindowCount < kVisionTargetWindowSize)
        {
          visionTargetWindowMps[visionTargetWindowCount++] = requestedMps;
        }
        else
        {
          for (uint8_t i = 1U; i < kVisionTargetWindowSize; ++i)
          {
            visionTargetWindowMps[i - 1U] = visionTargetWindowMps[i];
          }
          visionTargetWindowMps[kVisionTargetWindowSize - 1U] = requestedMps;
        }
        lastVisionBufferedSampleSequence = latestVisionSample.sequence;

        if (updateDue)
        {
          if (visionTargetFilterEnabled)
          {
            float weightedSum = 0.0F;
            float weightSum = 0.0F;
            for (uint8_t i = 0U; i < visionTargetWindowCount; ++i)
            {
              const float weight = static_cast<float>(i + 1U);
              weightedSum += visionTargetWindowMps[i] * weight;
              weightSum += weight;
            }
            filteredVisionDifferentialTargetMps = weightSum > 0.0F
                ? weightedSum / weightSum
                : requestedMps;
          }
          else
          {
            filteredVisionDifferentialTargetMps = requestedMps;
          }

          const float desiredMps = filteredVisionDifferentialTargetMps;
          if (visionTargetMaximumStepMmps == 0U)
          {
            lastVisionDifferentialTargetMps = desiredMps;
          }
          else
          {
            const float maximumStepMps = visionTargetMaximumStepMmps / 1000.0F;
            const float increment = constrain(desiredMps - lastVisionDifferentialTargetMps,
                                              -maximumStepMps, maximumStepMps);
            lastVisionDifferentialTargetMps += increment;
          }
          lastVisionTargetUpdateMs = nowMs;
        }
      }
    }
    // Remember a direction only after two fresh, same-direction valid frames.
    // Two near-zero valid frames clear the previous curve direction.
    if (directAccepted &&
        latestVisionSample.sequence != lastVisionCurveDetectionSequence)
    {
      lastVisionCurveDetectionSequence = latestVisionSample.sequence;
      const int16_t cameraDeltaMmps = latestVisionSample.deltaSpeedTargetMmps;
      if (abs(cameraDeltaMmps) >= 8)
      {
        const int8_t sign = cameraDeltaMmps > 0 ? 1 : -1;
        if (sign == visionCurveCandidateSign)
        {
          if (visionCurveCandidateCount < 2) visionCurveCandidateCount++;
        }
        else
        {
          visionCurveCandidateSign = sign;
          visionCurveCandidateCount = 1;
        }
      }
      else
      {
        if (visionCurveCandidateCount > 0) visionCurveCandidateCount--;
        if (visionCurveCandidateCount == 0) visionCurveCandidateSign = 0;
      }
    }
    const bool cameraReportedLost = controlEnabled && exchanged && !usable;
    if (cameraReportedLost)
    {
      filteredVisionDifferentialTargetMps = 0.0F;
      visionTargetFilterInitialized = false;
      visionTargetWindowCount = 0U;
    }
    if (visionCurveHoldTestEnabled && cameraReportedLost &&
        !visionCurveHoldLatched && visionCurveCandidateCount >= 2 &&
        visionCurveCandidateSign != 0)
    {
      visionCurveHoldLatched = true;
      visionCurveHoldLatchedMps = visionCurveCandidateSign *
                                  (visionCurveHoldTargetMmps / 1000.0F);
    }
    visionCommandTransportHeld = controlEnabled && !exchanged && lastVisionCommandMs != 0U &&
                                nowMs - lastVisionCommandMs <= 120U;
    visionCommandAccepted = directAccepted || visionCommandTransportHeld ||
                            (controlEnabled && visionCurveHoldTestEnabled && visionCurveHoldLatched);
    if (visionTrackingEnabled)
    {
      const float appliedVisionTargetMps =
          (visionCurveHoldTestEnabled && visionCurveHoldLatched)
              ? visionCurveHoldLatchedMps
              : lastVisionDifferentialTargetMps;
      motionCommand.setTargetDifferentialSpeedMps(
          visionCommandAccepted ? appliedVisionTargetMps : 0.0F);
    }
    static uint32_t lastPrintMs = 0;
    if (nowMs - lastPrintMs >= 100U)
    {
      lastPrintMs = nowMs;
      const float averageSpeedMps = 0.5F * (latestWheelSpeed.leftMetersPerSecond +
                                            latestWheelSpeed.rightMetersPerSecond);
      // Keep this line below the Wi-Fi console buffer capacity.  It is the
      // complete, intentionally small CSV source for tracking diagnosis.
      Serial.printf("[I2C] valid=%u dv=%d cmd_dv=%d vl=%.3f vr=%.3f measured_dv=%d vavg=%.3f vtarget=%.3f\n",
                    latestVisionSample.trackValid ? 1U : 0U,
                    latestVisionSample.deltaSpeedTargetMmps,
                    static_cast<int>(lroundf(motionCommand.targetDifferentialSpeedMps() * 1000.0F)),
                    latestWheelSpeed.leftMetersPerSecond,
                    latestWheelSpeed.rightMetersPerSecond,
                    static_cast<int>(lroundf((latestWheelSpeed.rightMetersPerSecond -
                                              latestWheelSpeed.leftMetersPerSecond) * 1000.0F)),
                    averageSpeedMps,
                    motionCommand.targetSpeedMps());
    }
  }

  void updateBalanceControl(uint32_t nowMs)
  {
    if (nowMs - lastControlUpdateMs < balance_car::config::kBalanceConfiguration.controlPeriodMs)
    {
      return;
    }

    const uint32_t elapsedMs = nowMs - lastControlUpdateMs;
    lastControlUpdateMs = nowMs;
    latestBalanceControlPeriodMs = static_cast<uint16_t>(elapsedMs > 65535U ? 65535U : elapsedMs);
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

    latestBalanceMotorCommand = balanceController.update(
        latestAttitude, latestVelocityPitchOffsetDegrees, static_cast<float>(elapsedMs) / 1000.0F);
    latestMixedMotorCommand = balance_car::control::MotorMixer::mix(
        latestBalanceMotorCommand, latestTurnMotorCommand,
        balanceController.tuning().maximumMotorCommand * airborneLandingManager.motorOutputScale(nowMs));
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
    Serial.print(motionCommand.targetSpeedMps(), 3);
    Serial.print(" measured_speed_mps=");
    Serial.print(velocityState.filteredSpeedMps, 3);
    Serial.print(" speed_error_mps=");
    Serial.print(velocityState.speedErrorMps, 3);
    Serial.print(" speed_pitch_offset_deg=");
    Serial.print(latestVelocityPitchOffsetDegrees, 3);
    Serial.print(" target_diff_speed_mps=");
    Serial.print(motionCommand.targetDifferentialSpeedMps(), 3);
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
  safetyManager.begin();
  const balance_car::app::SelfTestReport report = selfTest.run();
  balance_car::app::SelfTest::printReport(Serial, report);
  safetyManager.completeSelfTest(report);
  wifiDebugServer.begin();
  Serial.println(visionI2cClient.begin() ? "[I2C-INIT] vision v2 master ready"
                                         : "[I2C-INIT] vision v2 master init failed");
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
  updateVisionI2c(nowMs);
  updateBalanceControl(nowMs);
  reportSafetyFaultTransition();
  printTelemetry(nowMs);
  publishWifiTelemetry(nowMs);
  delay(1);
}

#undef Serial
