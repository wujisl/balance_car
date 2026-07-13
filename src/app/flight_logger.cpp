#include "app/flight_logger.h"

#include <SPIFFS.h>

namespace balance_car::app
{
bool FlightLogger::begin()
{
  // The dedicated SPIFFS partition is blank on the first firmware upload.
  // Formatting only occurs when mounting fails, not on every boot.
  _storageReady = SPIFFS.begin(true);
  return _storageReady;
}

void FlightLogger::startSession()
{
  _nextRecordIndex = 0;
  _recordCount = 0;
  _sessionActive = true;
}

void FlightLogger::append(const FlightLogSample &sample)
{
  if (!_sessionActive)
  {
    return;
  }

  _records[_nextRecordIndex] = sample;
  _nextRecordIndex = static_cast<uint16_t>((_nextRecordIndex + 1U) % kRecordCapacity);
  if (_recordCount < kRecordCapacity)
  {
    ++_recordCount;
  }
}

bool FlightLogger::saveSession()
{
  if (!_sessionActive)
  {
    return true;
  }

  _sessionActive = false;
  if (!_storageReady || _recordCount == 0)
  {
    return false;
  }

  SPIFFS.remove(kTemporaryLogPath);
  File outputFile = SPIFFS.open(kTemporaryLogPath, FILE_WRITE);
  if (!outputFile)
  {
    return false;
  }

  const uint16_t firstRecordIndex = _recordCount == kRecordCapacity ? _nextRecordIndex : 0;
  const FileHeader header = {
      .magic = kFileMagic,
      .version = kFileVersion,
      .recordSize = static_cast<uint16_t>(sizeof(FlightLogSample)),
      .recordCount = _recordCount,
      .startTimestampMs = _records[firstRecordIndex].timestampMs,
  };

  bool writeSucceeded = outputFile.write(reinterpret_cast<const uint8_t *>(&header), sizeof(header)) == sizeof(header);
  for (uint16_t recordOffset = 0; writeSucceeded && recordOffset < _recordCount; ++recordOffset)
  {
    const uint16_t recordIndex = static_cast<uint16_t>((firstRecordIndex + recordOffset) % kRecordCapacity);
    writeSucceeded = outputFile.write(reinterpret_cast<const uint8_t *>(&_records[recordIndex]),
                                      sizeof(FlightLogSample)) == sizeof(FlightLogSample);
  }
  outputFile.close();

  if (!writeSucceeded)
  {
    SPIFFS.remove(kTemporaryLogPath);
    return false;
  }

  SPIFFS.remove(kLogPath);
  if (!SPIFFS.rename(kTemporaryLogPath, kLogPath))
  {
    SPIFFS.remove(kTemporaryLogPath);
    return false;
  }
  return true;
}

bool FlightLogger::isSessionActive() const
{
  return _sessionActive;
}

void FlightLogger::printStatus(Stream &output) const
{
  output.print("[LOG] STORAGE=");
  output.print(_storageReady ? "READY" : "UNAVAILABLE");
  output.print(" ACTIVE=");
  output.print(_sessionActive ? 1 : 0);
  output.print(" BUFFERED_RECORDS=");
  output.print(_recordCount);
  output.print(" CAPACITY=");
  output.print(kRecordCapacity);

  FileHeader header = {};
  if (readFileHeader(header))
  {
    output.print(" SAVED_RECORDS=");
    output.println(header.recordCount);
    return;
  }
  output.println(" SAVED_RECORDS=0");
}

void FlightLogger::dumpCsv(Stream &output) const
{
  FileHeader header = {};
  if (!_storageReady || !readFileHeader(header))
  {
    output.println("[LOG] DUMP=UNAVAILABLE");
    return;
  }

  File inputFile = SPIFFS.open(kLogPath, FILE_READ);
  if (!inputFile)
  {
    output.println("[LOG] DUMP=OPEN_FAILED");
    return;
  }
  inputFile.seek(sizeof(FileHeader));
  output.print("[LOG] DUMP=CSV RECORDS=");
  output.println(header.recordCount);
  printCsvHeader(output);

  FlightLogSample sample = {};
  for (uint32_t recordIndex = 0; recordIndex < header.recordCount; ++recordIndex)
  {
    if (inputFile.read(reinterpret_cast<uint8_t *>(&sample), sizeof(sample)) != sizeof(sample))
    {
      output.println("[LOG] DUMP=READ_FAILED");
      break;
    }
    printCsvSample(output, sample);
  }
  inputFile.close();
  output.println("[LOG] DUMP=END");
}

bool FlightLogger::clearSavedLog()
{
  if (!_storageReady)
  {
    return false;
  }
  return !SPIFFS.exists(kLogPath) || SPIFFS.remove(kLogPath);
}

bool FlightLogger::readFileHeader(FileHeader &header) const
{
  if (!_storageReady)
  {
    return false;
  }
  File inputFile = SPIFFS.open(kLogPath, FILE_READ);
  if (!inputFile)
  {
    return false;
  }
  const bool readSucceeded = inputFile.read(reinterpret_cast<uint8_t *>(&header), sizeof(header)) == sizeof(header);
  inputFile.close();
  return readSucceeded && isFileHeaderValid(header);
}

bool FlightLogger::isFileHeaderValid(const FileHeader &header) const
{
  return header.magic == kFileMagic && header.version == kFileVersion &&
         header.recordSize == sizeof(FlightLogSample) && header.recordCount > 0 &&
         header.recordCount <= kRecordCapacity;
}

void FlightLogger::printCsvHeader(Stream &output) const
{
  output.println("timestamp_ms,pitch_deg,accel_pitch_deg,pitch_rate_dps,requested_pitch_deg,pitch_error_deg,"
                 "p_term,i_term,d_term,unclamped_balance_cmd,balance_cmd,left_motor_cmd,right_motor_cmd,"
                 "left_speed_mps,right_speed_mps,safety_state,fault_code,output_saturated");
}

void FlightLogger::printCsvSample(Stream &output, const FlightLogSample &sample) const
{
  output.print(sample.timestampMs);
  output.print(',');
  output.print(sample.pitchDegrees, 4);
  output.print(',');
  output.print(sample.accelerometerPitchDegrees, 4);
  output.print(',');
  output.print(sample.pitchRateDps, 4);
  output.print(',');
  output.print(sample.requestedPitchDegrees, 4);
  output.print(',');
  output.print(sample.pitchErrorDegrees, 4);
  output.print(',');
  output.print(sample.proportionalTerm, 5);
  output.print(',');
  output.print(sample.integralTerm, 5);
  output.print(',');
  output.print(sample.derivativeTerm, 5);
  output.print(',');
  output.print(sample.unclampedBalanceCommand, 5);
  output.print(',');
  output.print(sample.balanceCommand, 5);
  output.print(',');
  output.print(sample.leftMotorCommand, 5);
  output.print(',');
  output.print(sample.rightMotorCommand, 5);
  output.print(',');
  output.print(sample.leftSpeedMps, 4);
  output.print(',');
  output.print(sample.rightSpeedMps, 4);
  output.print(',');
  output.print(sample.safetyState);
  output.print(',');
  output.print(sample.faultCode);
  output.print(',');
  output.println(sample.outputSaturated);
}
} // namespace balance_car::app
