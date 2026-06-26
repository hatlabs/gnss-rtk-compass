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

constexpr unsigned char kSID = 0xFF;  // sequence id unused

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
      attitude_([this](const AttitudeVector& v) { attitude_v_.update(v); }),
      position_([this](const Position& v) { position_v_.update(v); }),
      cog_([this](float v) { cog_v_.update(v); }),
      sog_([this](float v) { sog_v_.update(v); }),
      num_satellites_([this](int v) { num_satellites_v_.update(v); }),
      hdop_([this](float v) { hdop_v_.update(v); }),
      datetime_([this](time_t v) { datetime_v_.update(v); }),
      satellites_([this](const std::vector<nmea0183::GNSSSatellite>& v) {
        satellites_v_.update(v);
      }),
      heading_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      attitude_v_(AttitudeVector(N2kDoubleNA, N2kDoubleNA, N2kDoubleNA), kExpiry,
                  AttitudeVector(N2kDoubleNA, N2kDoubleNA, N2kDoubleNA)),
      position_v_(Position(N2kDoubleNA, N2kDoubleNA), kExpiry,
                  Position(N2kDoubleNA, N2kDoubleNA)),
      cog_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      sog_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      num_satellites_v_(0, kExpiry, 0),
      hdop_v_(N2kDoubleNA, kExpiry, N2kDoubleNA),
      datetime_v_(0, kExpiry, 0),
      satellites_v_({}, kExpiry, {}) {
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
          "minutes. Only enable on a bus with other active talkers -- on a bus "
          "where this device is the sole talker it will reboot every two "
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
// UM982 is configured, so no nav data is published before then. Idempotent: a
// second call is a no-op rather than registering a duplicate set of senders.
void N2kSenders::enable_senders() {
  if (senders_enabled_) {
    return;
  }
  senders_enabled_ = true;

  auto* loop = event_loop().get();

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
}

}  // namespace gnss_rtk_compass
