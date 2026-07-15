#include "drivers/vision_i2c_client.h"

namespace balance_car::drivers
{
namespace
{
constexpr uint8_t kChassisMagic = 0xB6, kTrackingMagic = 0xB7, kVersion = 2;
constexpr uint8_t kFlagBalancing = 0x01, kFlagTrackingEnabled = 0x02, kFlagWheelSpeedValid = 0x04;
constexpr uint8_t kFlagTrackValid = 0x01, kFlagHeld = 0x02, kFlagCalibrated = 0x04, kFlagQualityOk = 0x08, kFlagFound = 0x10;
#pragma pack(push, 1)
struct ChassisPacket {
  uint8_t magic, version; uint16_t sequence; uint8_t flags;
  int16_t leftSpeedMmps, rightSpeedMmps, forwardTargetMmps;
  uint32_t timestampMs; uint8_t reserved[4]; uint8_t crc8;
};
struct TrackingPacket {
  uint8_t magic, version; uint16_t visionSequence, chassisSequence; uint8_t flags;
  int16_t lateralErrorMm, headingErrorCdeg, curvatureMilliperM, deltaSpeedTargetMmps;
  uint8_t quality, validRows, missedFrames; uint16_t lookaheadMm; uint8_t thresholdUsed;
  int16_t targetX; uint8_t crc8;
};
#pragma pack(pop)
static_assert(sizeof(ChassisPacket) == 20, "Chassis packet must be 20 bytes");
static_assert(sizeof(TrackingPacket) == 24, "Tracking packet must be 24 bytes");

uint8_t crc8(const uint8_t *data, size_t length)
{
  uint8_t crc = 0;
  while (length--) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0x07) : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}
} // namespace

VisionI2cClient::VisionI2cClient(TwoWire &wire, const config::VisionI2cPins &pins) : _wire(wire), _pins(pins) {}
bool VisionI2cClient::begin()
{
  _fresh = false;
  _lastVisionSequence = 0;
  _lastVisionAdvanceMs = 0;
  _healthy = _wire.begin(_pins.sda, _pins.scl, kFrequencyHz);
  return _healthy;
}

bool VisionI2cClient::exchange(const VisionChassisState &state, VisionSample &sample)
{
  ChassisPacket outgoing = {};
  outgoing.magic = kChassisMagic; outgoing.version = kVersion; outgoing.sequence = state.sequence;
  if (state.balancing) outgoing.flags |= kFlagBalancing;
  if (state.trackingEnabled) outgoing.flags |= kFlagTrackingEnabled;
  if (state.wheelSpeedValid) outgoing.flags |= kFlagWheelSpeedValid;
  outgoing.leftSpeedMmps = state.leftSpeedMmps; outgoing.rightSpeedMmps = state.rightSpeedMmps;
  outgoing.forwardTargetMmps = state.forwardTargetMmps; outgoing.timestampMs = state.timestampMs;
  outgoing.crc8 = crc8(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing) - 1);
  _wire.beginTransmission(kAddress);
  _wire.write(reinterpret_cast<const uint8_t *>(&outgoing), sizeof(outgoing));
  if (_wire.endTransmission(true) != 0) { _healthy = false; return false; }
  // ESP32 I2C slave receive callbacks are dispatched after STOP.  Leave a
  // bounded gap before the read phase so the camera can commit chassis state;
  // this costs 0.25 ms in a 20 ms transaction period.
  delayMicroseconds(250);

  TrackingPacket incoming = {};
  if (_wire.requestFrom(kAddress, static_cast<size_t>(sizeof(incoming)), true) != sizeof(incoming)) {
    while (_wire.available()) _wire.read(); _healthy = false; return false;
  }
  uint8_t *raw = reinterpret_cast<uint8_t *>(&incoming);
  for (size_t index = 0; index < sizeof(incoming); ++index) raw[index] = static_cast<uint8_t>(_wire.read());
  if (incoming.magic != kTrackingMagic || incoming.version != kVersion ||
      incoming.crc8 != crc8(raw, sizeof(incoming) - 1)) { _healthy = false; return false; }
  const uint32_t nowMs = state.timestampMs;
  if (!_fresh || incoming.visionSequence != _lastVisionSequence) {
    _lastVisionSequence = incoming.visionSequence;
    _lastVisionAdvanceMs = nowMs;
    _fresh = true;
  }
  // A valid CRC only proves that the slave replied.  Do not reuse an old
  // tracking command indefinitely when camera capture/publish has stopped.
  if (nowMs - _lastVisionAdvanceMs > 300U) {
    _healthy = false;
    _fresh = false;
    return false;
  }
  sample.sequence = incoming.visionSequence; sample.chassisSequence = incoming.chassisSequence;
  sample.trackValid = (incoming.flags & kFlagTrackValid) != 0; sample.held = (incoming.flags & kFlagHeld) != 0;
  sample.calibrated = (incoming.flags & kFlagCalibrated) != 0; sample.qualityOk = (incoming.flags & kFlagQualityOk) != 0;
  sample.found = (incoming.flags & kFlagFound) != 0;
  sample.lateralErrorMm = incoming.lateralErrorMm; sample.headingErrorCdeg = incoming.headingErrorCdeg;
  sample.curvatureMilliperM = incoming.curvatureMilliperM; sample.deltaSpeedTargetMmps = incoming.deltaSpeedTargetMmps;
  sample.quality = incoming.quality; sample.validRows = incoming.validRows; sample.missedFrames = incoming.missedFrames;
  sample.lookaheadMm = incoming.lookaheadMm; sample.thresholdUsed = incoming.thresholdUsed; sample.targetX = incoming.targetX;
  _healthy = true; return true;
}
bool VisionI2cClient::isHealthy() const { return _healthy; }
bool VisionI2cClient::isFresh() const { return _fresh; }
} // namespace balance_car::drivers
