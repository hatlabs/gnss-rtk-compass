#include "n2k_senders.h"

#include <N2kMessages.h>
#include <NMEA2000_esp32.h>

#include <memory>

#include "sensesp.h"

namespace gnss_rtk_compass {

namespace {

// SH-ESP32 CAN pins.
constexpr gpio_num_t kCanTxPin = GPIO_NUM_32;
constexpr gpio_num_t kCanRxPin = GPIO_NUM_34;

// Inputs are considered stale after this long without an update.
constexpr unsigned long kExpiry = 2000;

constexpr unsigned char kSID = 0xFF;  // sequence id unused

std::shared_ptr<tNMEA2000_esp32> nmea2000;

uint16_t DaysSince1970(time_t t) { return t / 86400; }
double SecondsSinceMidnight(time_t t) { return t % 86400; }

}  // namespace

N2kSenders::N2kSenders(uint8_t source_address)
    : heading_([this](float v) { heading_v_.update(v); }),
      attitude_([this](const AttitudeVector& v) { attitude_v_.update(v); }),
      position_([this](const Position& v) { position_v_.update(v); }),
      cog_([this](float v) { cog_v_.update(v); }),
      sog_([this](float v) { sog_v_.update(v); }),
      num_satellites_([this](int v) { num_satellites_v_.update(v); }),
      hdop_([this](float v) { hdop_v_.update(v); }),
      datetime_([this](time_t v) { datetime_v_.update(v); }),
      heading_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      attitude_v_(AttitudeVector(N2kDoubleNA, N2kDoubleNA, N2kDoubleNA), kExpiry,
                  AttitudeVector(N2kDoubleNA, N2kDoubleNA, N2kDoubleNA)),
      position_v_(Position(N2kDoubleNA, N2kDoubleNA), kExpiry,
                  Position(N2kDoubleNA, N2kDoubleNA)),
      cog_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      sog_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      num_satellites_v_(0, kExpiry, 0),
      hdop_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      datetime_v_(0, kExpiry, 0) {
  nmea2000 = std::make_shared<tNMEA2000_esp32>(kCanTxPin, kCanRxPin);

  nmea2000->SetProductInformation("00000001", 130, "GNSS RTK Compass", "1.0",
                                  "1.0");
  // Unique number 1; function 145 (GNSS), class 60 (Navigation), Hat Labs.
  nmea2000->SetDeviceInformation(1, 145, 60, 2046);
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, source_address);
  nmea2000->EnableForward(false);
  nmea2000->Open();

  auto* loop = event_loop().get();

  loop->onRepeat(5, []() { nmea2000->ParseMessages(); });

  // PGN 127250 Vessel Heading (true).
  loop->onRepeat(100, [this]() {
    tN2kMsg msg;
    SetN2kPGN127250(msg, kSID, heading_v_.get(), N2kDoubleNA, N2kDoubleNA,
                    N2khr_true);
    nmea2000->SendMsg(msg);
  });

  // PGN 127257 Attitude (yaw/pitch/roll).
  loop->onRepeat(100, [this]() {
    AttitudeVector a = attitude_v_.get();
    tN2kMsg msg;
    SetN2kAttitude(msg, kSID, a.yaw, a.pitch, a.roll);
    nmea2000->SendMsg(msg);
  });

  // PGN 129025 Position, Rapid Update.
  loop->onRepeat(250, [this]() {
    Position p = position_v_.get();
    tN2kMsg msg;
    SetN2kPGN129025(msg, p.latitude, p.longitude);
    nmea2000->SendMsg(msg);
  });

  // PGN 129026 COG & SOG, Rapid Update.
  loop->onRepeat(250, [this]() {
    tN2kMsg msg;
    SetN2kCOGSOGRapid(msg, kSID, N2khr_true, cog_v_.get(), sog_v_.get());
    nmea2000->SendMsg(msg);
  });

  // PGN 129029 GNSS Position Data.
  loop->onRepeat(1000, [this]() {
    Position p = position_v_.get();
    time_t t = datetime_v_.get();
    bool have_fix = p.latitude != N2kDoubleNA;
    double altitude =
        p.altitude == kPositionInvalidAltitude ? N2kDoubleNA : p.altitude;
    tN2kMsg msg;
    SetN2kGNSS(msg, kSID, DaysSince1970(t), SecondsSinceMidnight(t),
               p.latitude, p.longitude, altitude, N2kGNSSt_GPS,
               have_fix ? N2kGNSSm_GNSSfix : N2kGNSSm_noGNSS,
               num_satellites_v_.get(), hdop_v_.get());
    nmea2000->SendMsg(msg);
  });
}

}  // namespace gnss_rtk_compass
