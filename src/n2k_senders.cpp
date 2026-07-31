#include "n2k_senders.h"

#include <N2kMessages.h>
#include <NMEA2000_esp32.h>

#include <memory>

#include "sensesp.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp/ui/ui_controls.h"

#include "counting_nmea2000.h"

namespace gnss_rtk_compass {

namespace {

// SH-ESP32 CAN pins.
constexpr gpio_num_t kCanTxPin = GPIO_NUM_32;
constexpr gpio_num_t kCanRxPin = GPIO_NUM_34;

// Inputs are considered stale after this long without an update.
constexpr unsigned long kExpiry = 2000;

// Magnetic variation expires far more slowly. The receiver fills the RMC
// variation field only intermittently (~10-30 s) and declination is quasi-static,
// so a long timeout bridges the gaps without flapping to not-available, yet still
// degrades to not-available on a genuine dropout rather than holding a stale value.
constexpr unsigned long kVariationExpiry = 60000;

constexpr unsigned char kSID = 0xFF;  // sequence id unused

// PGNs this device transmits, advertised on request via PGN 126464. A
// translating gateway (e.g. the Raymarine Micro-Talk feeding Tacktick/Micronet)
// forwards a device's data only if it declares the matching PGNs; passive
// listeners like Signal K decode every frame regardless. Keep in sync with the
// senders in enable_senders(): every PGN here needs a matching sender and vice
// versa. Terminated with 0.
const unsigned long kTransmitPGNs[] = {
    127250UL,  // Vessel Heading
    127251UL,  // Rate of Turn
    129025UL,  // Position, Rapid Update
    129026UL,  // COG & SOG, Rapid Update
    129029UL,  // GNSS Position Data
    129539UL,  // GNSS DOP
    129540UL,  // GNSS Satellites in View
    127258UL,  // Magnetic Variation
    126992UL,  // System Time
    129044UL,  // Datum
    0};

std::shared_ptr<CountingNMEA2000> nmea2000;

// N2K diagnostics surfaced on /api/info (parity with ais/wind).
ObservableValue<int> n2k_rx_counter{0};
unsigned long n2k_last_rx_ms = 0;
std::shared_ptr<StatusPageItem<int>> n2k_rx_status;
std::shared_ptr<StatusPageItem<int>> n2k_tx_status;
std::shared_ptr<CheckboxConfig> n2k_watchdog_config;

uint16_t DaysSince1970(time_t t) { return t / 86400; }
double SecondsSinceMidnight(time_t t) { return t % 86400; }

}  // namespace

N2kSenders::N2kSenders(uint8_t source_address)
    : heading_([this](float v) { heading_v_.update(v); }),
      rate_of_turn_([this](float v) { rate_of_turn_v_.update(v); }),
      position_([this](const Position& v) { position_v_.update(v); }),
      cog_([this](float v) { cog_v_.update(v); }),
      sog_([this](float v) { sog_v_.update(v); }),
      num_satellites_([this](int v) { num_satellites_v_.update(v); }),
      hdop_([this](float v) { hdop_v_.update(v); }),
      datetime_([this](time_t v) { datetime_v_.update(v); }),
      satellites_([this](const std::vector<nmea0183::GNSSSatellite>& v) {
        satellites_v_.update(v);
      }),
      variation_([this](float v) { variation_v_.update(v); }),
      heading_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      rate_of_turn_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      position_v_(Position(N2kDoubleNA, N2kDoubleNA), kExpiry,
                  Position(N2kDoubleNA, N2kDoubleNA)),
      cog_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      sog_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      num_satellites_v_(0, kExpiry, 0),
      hdop_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      datetime_v_(0, kExpiry, 0),
      satellites_v_({}, kExpiry, {}),
      variation_v_(N2kDoubleNA, kVariationExpiry, N2kDoubleNA) {
  nmea2000 = std::make_shared<CountingNMEA2000>(kCanTxPin, kCanRxPin);

  nmea2000->SetProductInformation("00000001", 130, "GNSS RTK Compass", "1.0",
                                  "1.0");
  // Unique number 1; function 145 (GNSS), class 60 (Navigation), Hat Labs.
  nmea2000->SetDeviceInformation(1, 145, 60, 2046);
  nmea2000->SetMode(tNMEA2000::N2km_NodeOnly, source_address);
  nmea2000->SetMsgHandler([](const tN2kMsg&) {
    n2k_rx_counter.set(n2k_rx_counter.get() + 1);
    n2k_last_rx_ms = millis();
  });
  nmea2000->EnableForward(false);
  nmea2000->ExtendTransmitMessages(kTransmitPGNs);
  nmea2000->Open();

  auto* loop = event_loop().get();

  loop->onRepeat(5, []() { nmea2000->ParseMessages(); });

  // NMEA 2000 message counters on /api/info (parity with ais/wind). TX is
  // tallied by CountingNMEA2000 on each SendMsg; RX by the handler above.
  n2k_rx_status = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Received Messages", 0, "NMEA 2000", 300);
  n2k_rx_counter.connect_to(n2k_rx_status);
  n2k_tx_status = std::make_shared<StatusPageItem<int>>(
      "NMEA 2000 Transmitted Messages", 0, "NMEA 2000", 310);
  nmea2000->tx_count_.connect_to(n2k_tx_status);

  // Optional N2K watchdog: reboot if no N2K message arrives for two minutes
  // (default off). Matches the ais/wind interfaces.
  n2k_watchdog_config = std::make_shared<CheckboxConfig>(
      false, "Enable NMEA 2000 Watchdog", "/NMEA 2000/Enable Watchdog");
  ConfigItem(n2k_watchdog_config)
      ->set_title("NMEA 2000 Watchdog")
      ->set_description(
          "Reboot the device if no NMEA 2000 message is received for two "
          "minutes. Requires a restart to take effect.")
      ->set_sort_order(320);
  if (n2k_watchdog_config->get_value()) {
    loop->onRepeat(1000, []() {
      if (millis() - n2k_last_rx_ms > 120000) {
        ESP_LOGE("NMEA2000", "No messages received in 2 minutes. Restarting.");
        delay(10);
        ESP.restart();
      }
    });
  }
}

// Periodic data-publishing senders. Started from WireOutputs() only after the
// UM982 is configured, so no nav data is published before then.
void N2kSenders::enable_senders() {
  auto* loop = event_loop().get();

  // PGN 127250 Vessel Heading (true).
  loop->onRepeat(100, [this]() {
    tN2kMsg msg;
    SetN2kPGN127250(msg, kSID, heading_v_.get(), N2kDoubleNA, N2kDoubleNA,
                    N2khr_true);
    nmea2000->SendMsg(msg);
  });

  // PGN 127251 Rate of Turn. Derived from the heading stream (the UM982 has no
  // gyro); positive = turning to starboard.
  loop->onRepeat(100, [this]() {
    tN2kMsg msg;
    SetN2kRateOfTurn(msg, kSID, rate_of_turn_v_.get());
    nmea2000->SendMsg(msg);
  });

  // PGN 129025 Position, Rapid Update, at the NMEA 2000 standard 100 ms (10 Hz).
  loop->onRepeat(100, [this]() {
    Position p = position_v_.get();
    tN2kMsg msg;
    SetN2kPGN129025(msg, p.latitude, p.longitude);
    nmea2000->SendMsg(msg);
  });

  // PGN 129026 COG & SOG, Rapid Update, at 100 ms (10 Hz) to match position --
  // above the 250 ms N2K standard, for COG/SOG as responsive as position.
  loop->onRepeat(100, [this]() {
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

  // PGN 129539 GNSS DOP (HDOP only; the GNSS sentences don't expose V/TDOP).
  loop->onRepeat(1000, [this]() {
    tN2kMsg msg;
    SetN2kPGN129539(msg, kSID, N2kGNSSdm_Auto, N2kGNSSdm_3D, hdop_v_.get(),
                    N2kDoubleNA, N2kDoubleNA);
    nmea2000->SendMsg(msg);
  });

  // PGN 129540 GNSS Satellites in View.
  loop->onRepeat(1000, [this]() {
    auto satellites = satellites_v_.get();
    tN2kMsg msg;
    SetN2kPGN129540(msg, kSID, N2kDD072_Unavailable);
    for (const auto& s : satellites) {
      tSatelliteInfo info;
      info.PRN = s.id;
      info.Elevation =
          s.elevation.is_valid() ? (double)s.elevation * DEG_TO_RAD : N2kDoubleNA;
      info.Azimuth =
          s.azimuth.is_valid() ? (double)s.azimuth * DEG_TO_RAD : N2kDoubleNA;
      info.SNR = s.snr;
      info.RangeResiduals = N2kDoubleNA;
      info.UsageStatus = N2kDD124_NotTracked;
      AppendN2kPGN129540(msg, info);
    }
    nmea2000->SendMsg(msg);
  });

  // PGN 127258 Magnetic Variation. From the receiver's GPRMC variation field,
  // which arrives only intermittently; variation_v_ bridges the gaps (long
  // expiry) and goes not-available on a genuine dropout. The age-of-service date
  // is not-available until a fix supplies the time (mirrors 126992 below).
  loop->onRepeat(1000, [this]() {
    time_t t = datetime_v_.get();
    tN2kMsg msg;
    SetN2kMagneticVariation(msg, kSID, N2kmagvar_Calc,
                            t == 0 ? N2kUInt16NA : DaysSince1970(t),
                            variation_v_.get());
    nmea2000->SendMsg(msg);
  });

  // PGN 126992 System Time (from GNSS). Sent as not-available until a fix
  // provides the date/time.
  loop->onRepeat(1000, [this]() {
    time_t t = datetime_v_.get();
    tN2kMsg msg;
    if (t == 0) {
      SetN2kSystemTime(msg, kSID, N2kUInt16NA, N2kDoubleNA);
    } else {
      SetN2kSystemTime(msg, kSID, DaysSince1970(t), SecondsSinceMidnight(t));
    }
    nmea2000->SendMsg(msg);
  });

  // PGN 129044 Datum. Static WGS84 declaration (the datum our position PGNs are
  // referenced to). The library has no setter, so hand-build it: Local Datum,
  // three zero deltas (no offset from the reference), Reference Datum. Datum IDs
  // are IHO S-60 codes -- "W84" is WGS84; the 4th byte is the subdivision (none).
  // 10 s is the NMEA 2000 standard interval for this quasi-static PGN.
  loop->onRepeat(10000, [this]() {
    tN2kMsg msg;
    msg.SetPGN(129044L);
    msg.Priority = 6;
    msg.AddStr("W84", 4, false, ' ');  // Local Datum
    msg.Add4ByteDouble(0.0, 1e-7);     // Delta Latitude
    msg.Add4ByteDouble(0.0, 1e-7);     // Delta Longitude
    msg.Add4ByteDouble(0.0, 0.01);     // Delta Altitude (cm)
    msg.AddStr("W84", 4, false, ' ');  // Reference Datum
    nmea2000->SendMsg(msg);
  });
}

}  // namespace gnss_rtk_compass
