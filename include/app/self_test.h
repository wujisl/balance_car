#pragma once

#include "drivers/encoder_driver.h"
#include "drivers/imu_driver.h"
#include "drivers/motor_driver.h"

#include <Arduino.h>

namespace balance_car::app
{
struct SelfTestReport
{
  bool motorDriverReady = false;
  bool encodersReady = false;
  bool imuReady = false;
  bool imuSampleValid = false;
  bool imuCalibrated = false;
  bool passed = false;
  drivers::ImuModel imuModel = drivers::ImuModel::Unknown;
  uint8_t imuAddress = 0;
};

class SelfTest
{
public:
  SelfTest(drivers::MotorDriver &motorDriver, drivers::EncoderDriver &encoderDriver,
           drivers::ImuDriver &imuDriver);

  SelfTestReport run();
  static void printReport(Stream &output, const SelfTestReport &report);

private:
  drivers::MotorDriver &_motorDriver;
  drivers::EncoderDriver &_encoderDriver;
  drivers::ImuDriver &_imuDriver;
};
} // namespace balance_car::app
