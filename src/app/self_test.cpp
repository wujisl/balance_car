#include "app/self_test.h"

namespace balance_car::app
{
SelfTest::SelfTest(drivers::MotorDriver &motorDriver, drivers::EncoderDriver &encoderDriver,
                   drivers::ImuDriver &imuDriver)
    : _motorDriver(motorDriver), _encoderDriver(encoderDriver), _imuDriver(imuDriver)
{
}

SelfTestReport SelfTest::run()
{
  SelfTestReport report = {};
  report.motorDriverReady = _motorDriver.begin();
  _motorDriver.setEnabled(false);
  report.encodersReady = _encoderDriver.begin();
  report.imuReady = _imuDriver.begin();
  if (report.imuReady)
  {
    // Do not require a stationary, multi-second gyro calibration at power-on.
    // A valid first sample is enough to enter STANDBY; arming still checks
    // current IMU health and the runtime monitor remains active.
    report.imuSampleValid = _imuDriver.read().valid;
    report.imuModel = _imuDriver.model();
    report.imuAddress = _imuDriver.address();
  }

  // Motor/encoder begin results remain visible for diagnosis, but do not lock
  // out balancing.  The minimum safe startup condition is a readable IMU.
  report.passed = report.imuReady && report.imuSampleValid;
  return report;
}

void SelfTest::printReport(Stream &output, const SelfTestReport &report)
{
  output.print("[SELFTEST] MOTOR=");
  output.println(report.motorDriverReady ? "READY" : "FAIL");
  output.print("[SELFTEST] ENCODERS=");
  output.println(report.encodersReady ? "READY" : "FAIL");
  output.print("[SELFTEST] IMU=");
  output.println(report.imuReady ? "READY" : "FAIL");
  if (report.imuReady)
  {
    output.print("[SELFTEST] IMU_MODEL=");
    switch (report.imuModel)
    {
    case drivers::ImuModel::Mpu6050:
      output.println("MPU6050");
      break;
    case drivers::ImuModel::Mpu6500:
      output.println("MPU6500");
      break;
    default:
      output.println("UNKNOWN");
      break;
    }
    output.print("[SELFTEST] IMU_ADDRESS=0x");
    output.println(report.imuAddress, HEX);
  }
  output.println("[SELFTEST] GYRO_CALIBRATION=SKIPPED");
  output.print("[SELFTEST] IMU_FIRST_SAMPLE=");
  output.println(report.imuSampleValid ? "PASS" : "FAIL");
  output.print("[SELFTEST] RESULT=");
  output.println(report.passed ? "PASS" : "FAIL");
}
} // namespace balance_car::app
