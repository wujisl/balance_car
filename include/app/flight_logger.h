#pragma once

#include <Arduino.h>

namespace balance_car::app
{
// One record is 64 bytes.  At 200 Hz, the in-memory ring buffer retains the
// most recent eight seconds before a stop or safety fault.
struct FlightLogSample
{
  uint32_t timestampMs = 0;
  float pitchDegrees = 0.0F;
  float accelerometerPitchDegrees = 0.0F;
  float pitchRateDps = 0.0F;
  float requestedPitchDegrees = 0.0F;
  float pitchErrorDegrees = 0.0F;
  float proportionalTerm = 0.0F;
  float integralTerm = 0.0F;
  float derivativeTerm = 0.0F;
  float unclampedBalanceCommand = 0.0F;
  float balanceCommand = 0.0F;
  float leftMotorCommand = 0.0F;
  float rightMotorCommand = 0.0F;
  float leftSpeedMps = 0.0F;
  float rightSpeedMps = 0.0F;
  uint8_t safetyState = 0;
  uint8_t faultCode = 0;
  uint8_t outputSaturated = 0;
  uint8_t reserved = 0;
};

static_assert(sizeof(FlightLogSample) == 64, "Flight log record size changed");

class FlightLogger
{
public:
  static constexpr uint16_t kRecordCapacity = 1600;

  bool begin();
  void startSession();
  void append(const FlightLogSample &sample);
  bool saveSession();
  bool isSessionActive() const;
  void printStatus(Stream &output) const;
  void dumpCsv(Stream &output) const;
  bool clearSavedLog();

private:
  struct FileHeader
  {
    uint32_t magic;
    uint16_t version;
    uint16_t recordSize;
    uint32_t recordCount;
    uint32_t startTimestampMs;
  };

  static constexpr uint32_t kFileMagic = 0x474C4342UL; // "BCLG" in little endian.
  static constexpr uint16_t kFileVersion = 1;
  static constexpr const char *kLogPath = "/last_flight.bin";
  static constexpr const char *kTemporaryLogPath = "/last_flight.tmp";

  bool readFileHeader(FileHeader &header) const;
  bool isFileHeaderValid(const FileHeader &header) const;
  void printCsvHeader(Stream &output) const;
  void printCsvSample(Stream &output, const FlightLogSample &sample) const;

  FlightLogSample _records[kRecordCapacity] = {};
  uint16_t _nextRecordIndex = 0;
  uint16_t _recordCount = 0;
  bool _storageReady = false;
  bool _sessionActive = false;
};
} // namespace balance_car::app
