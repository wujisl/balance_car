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

struct MotionConfiguration
{
  float maximumTargetSpeedMps;
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
    .rightDirectionInverted = false,
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
    .targetPitchDegrees = -2.25F,
    .proportionalGain = 0.15F,
    // Start tuning with P-D control only. Enable a small Ki only after the
    // mechanical trim has been verified on the actual vehicle.
    .integralGain = 0.0F,
    .derivativeGain = 0.003F,
    .integralLimit = 5000.0F,
    .maximumMotorCommand = 0.40F,
    .motorOutputInverted = false,
};

constexpr VelocityConfiguration kVelocityConfiguration = {
    .controlPeriodMs = 40,
    // Disable the outer speed loop for initial P-D-and-trim balance testing.
    .proportionalGain = 0.0F,
    .integralGain = 0.0F,
    .integralLimit = 2.0F,
    .maximumPitchOffsetDegrees = 6.0F,
    .measurementFilterAlpha = 0.3F,
    .outputInverted = false,
};

constexpr MotionConfiguration kMotionConfiguration = {
    .maximumTargetSpeedMps = 0.25F,
    .targetSpeedStepMps = 0.05F,
    .maximumTurnCommand = 0.20F,
    .turnCommandStep = 0.03F,
};
} // namespace balance_car::config
