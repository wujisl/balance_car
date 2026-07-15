#pragma once

#include <Arduino.h>

namespace balance_car::config
{
  struct MotorConfiguration
  {
    uint32_t pwmFrequencyHz;
    uint8_t pwmResolutionBits;
    bool leftDirectionInverted;
    bool rightDirectionInverted;
  };

  struct EncoderConfiguration
  {
    float countsPerWheelRevolution;
    float wheelDiameterMeters;
    bool useInternalPullups;
    bool leftDirectionInverted;
    bool rightDirectionInverted;
  };

  struct ImuConfiguration
  {
    uint32_t i2cFrequencyHz;
    uint16_t calibrationSamples;
    uint16_t calibrationIntervalMs;
    float maximumStationaryGyroStdDevDps;
    uint16_t maximumSampleAgeMs;
  };

  struct SafetyConfiguration
  {
    uint16_t manualTestDurationMs;
    float manualTestPower;
    uint16_t offlineArmHoldMs;
    float balanceStartAngleLimitDegrees;
    float balanceFaultAngleDegrees;
  };

  // Optional protection for a confirmed loss of ground contact. It is disabled
  // by default so ordinary ground-driving and differential-speed tuning retain
  // their existing control path and timing.
  struct AirborneLandingConfiguration
  {
    bool enabled;
    float airborneAccelerationThresholdG;
    uint16_t airborneConfirmationMs;
    uint16_t maximumAirborneMs;
    float landingAccelerationMinimumG;
    float landingAccelerationMaximumG;
    uint16_t landingSettleMs;
    uint16_t landingRecoveryTimeoutMs;
    uint16_t motorRecoveryRampMs;
  };

  struct AttitudeConfiguration
  {
    enum class PitchAxis
    {
      X,
      Y,
    };

    PitchAxis pitchAxis;
    float complementaryFilterTimeConstantSeconds;
    float accelerometerAngleOffsetDegrees;
    bool pitchAngleInverted;
    bool pitchGyroInverted;
  };

  struct BalanceConfiguration
  {
    uint16_t controlPeriodMs;
    float targetPitchDegrees;
    float proportionalGain;
    float integralGain;
    float derivativeGain;
    float integralLimit;
    float maximumMotorCommand;
    bool motorOutputInverted;
  };

  struct VelocityConfiguration
  {
    uint16_t controlPeriodMs;
    float proportionalGain;
    float integralGain;
    float integralLimit;
    float maximumPitchOffsetDegrees;
    float measurementFilterAlpha;
    bool outputInverted;
  };

  // Controls the wheel-speed difference used for steering. A positive target
  // means that the right wheel should move faster than the left wheel.
  struct DifferentialSpeedConfiguration
  {
    float proportionalGain;
    float integralGain;
    float integralLimit;
    float maximumTurnMotorCommand;
    float measurementFilterAlpha;
    bool outputInverted;
  };

  // Differential-drive odometry uses the distance between the centers of the
  // two tire contact patches. Measure this on the assembled vehicle before
  // relying on its heading for path measurement.
  struct OdometryConfiguration
  {
    float wheelTrackMeters;
    float yawRateFilterAlpha;
  };

  struct MotionConfiguration
  {
    float maximumTargetSpeedMps;
    float initialTargetSpeedMps;
    float targetSpeedStepMps;
    float maximumTurnCommand;
    float turnCommandStep;
  };

  constexpr MotorConfiguration kMotorConfiguration = {
      .pwmFrequencyHz = 20000,
      .pwmResolutionBits = 10,
      .leftDirectionInverted = false,
      .rightDirectionInverted = false,
  };

  constexpr EncoderConfiguration kEncoderConfiguration = {
      // Measured with the current A-phase CHANGE interrupt counting method.
      .countsPerWheelRevolution = 530.0F,
      .wheelDiameterMeters = 0.064F,
      .useInternalPullups = true,
      .leftDirectionInverted = false,
      // The right encoder is mirror-mounted.  Invert it so both wheels report
      // the same sign when the vehicle moves forward.
      .rightDirectionInverted = true,
  };

  constexpr ImuConfiguration kImuConfiguration = {
      .i2cFrequencyHz = 400000,
      .calibrationSamples = 500,
      .calibrationIntervalMs = 2,
      .maximumStationaryGyroStdDevDps = 3.0F,
      .maximumSampleAgeMs = 40,
  };

  constexpr SafetyConfiguration kSafetyConfiguration = {
      .manualTestDurationMs = 1000,
      .manualTestPower = 0.15F,
      .offlineArmHoldMs = 1500,
      .balanceStartAngleLimitDegrees = 30.0F,
      .balanceFaultAngleDegrees = 60.0F,
  };

  constexpr AirborneLandingConfiguration kAirborneLandingConfiguration = {
      // Enable only after suspended and small-height landing tests have
      // validated the vehicle's mechanical strength and IMU thresholds.
      .enabled = false,
      .airborneAccelerationThresholdG = 0.35F,
      .airborneConfirmationMs = 20,
      .maximumAirborneMs = 500,
      .landingAccelerationMinimumG = 0.75F,
      .landingAccelerationMaximumG = 1.25F,
      .landingSettleMs = 60,
      .landingRecoveryTimeoutMs = 700,
      .motorRecoveryRampMs = 250,
  };

  constexpr AttitudeConfiguration kAttitudeConfiguration = {
      .pitchAxis = AttitudeConfiguration::PitchAxis::Y,
      .complementaryFilterTimeConstantSeconds = 0.25F,
      .accelerometerAngleOffsetDegrees = 1.5F,
      .pitchAngleInverted = false,
      .pitchGyroInverted = false,
  };

  constexpr BalanceConfiguration kBalanceConfiguration = {
      .controlPeriodMs = 5,
      // Initial mechanical-balance trim measured on the assembled vehicle.
      .targetPitchDegrees = -2.09F,
      .proportionalGain = 0.09F,
      // Start tuning with P-D control only. Enable a small Ki only after the
      // mechanical trim has been verified on the actual vehicle.
      .integralGain = 0.0F,
      .derivativeGain = 0.003F,
      .integralLimit = 5000.0F,
      .maximumMotorCommand = 0.45F,
      .motorOutputInverted = false,
  };

  constexpr VelocityConfiguration kVelocityConfiguration = {
      .controlPeriodMs = 40,
      // Vehicle-tested P-only starting point. Refine from this point with
      // small increments after confirming encoder direction.
      .proportionalGain = 13.0F,
      .integralGain = 0.0F,
      .integralLimit = 2.0F,
      .maximumPitchOffsetDegrees = 6.0F,
      .measurementFilterAlpha = 0.3F,
      .outputInverted = false,
  };

  constexpr DifferentialSpeedConfiguration kDifferentialSpeedConfiguration = {
      // At zero measured differential speed this retains the former
      // turn-command-to-motor-command scale, while encoder feedback removes
      // left/right motor and surface mismatch.
      .proportionalGain = 1.00F,
      .integralGain = 0.01F,
      .integralLimit = 0.20F,
      .maximumTurnMotorCommand = 0.20F,
      .measurementFilterAlpha = 0.30F,
      .outputInverted = false,
  };

  constexpr OdometryConfiguration kOdometryConfiguration = {
      // Initial measured estimate; calibrate with a slow, known-angle turn.
      .wheelTrackMeters = 0.200F,
      // Suppresses encoder-quantization noise while retaining turn response.
      .yawRateFilterAlpha = 0.35F,
  };

  constexpr MotionConfiguration kMotionConfiguration = {
      .maximumTargetSpeedMps = 0.25F,
      .initialTargetSpeedMps = 0.06F,
      .targetSpeedStepMps = 0.05F,
      // Target right-minus-left wheel speed for differential steering, in m/s.
      .maximumTurnCommand = 0.20F,
      .turnCommandStep = 0.03F,
  };
} // namespace balance_car::config
