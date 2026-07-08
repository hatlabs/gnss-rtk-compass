// GNSS RTK Compass — UM982 dual-antenna satellite compass on SH-ESP32.
//
// Reads the UM982 serial output, derives true heading from the antenna
// baseline, and publishes heading, attitude, position, speed/course and fix
// quality to both Signal K and NMEA 2000. See SPEC.md.
//
// Boot order matters: the UM982 settings (mode, baseline, offset, anti-jam,
// anti-spoof, smoothing) are applied and ACK-confirmed before any output is
// wired, so no heading/position is published until the module is configured.

#include <time.h>

#include <memory>
#include <vector>

#include "sensesp.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_types.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"
#include "sensesp/ui/status_page_item.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/wiring.h"

#include "gnhpr_parser.h"
#include "n2k_senders.h"
#include "um982_config.h"

using namespace sensesp;
using namespace sensesp::nmea0183;
using namespace gnss_rtk_compass;

namespace sensesp {
// Serialise the satellite list as a Signal K navigation.gnss.satellitesInView
// object ({count, satellites[]}); the generic SKOutput body can't convert a
// std::vector, so this specialisation (declared before the type is instantiated
// in CreateSKOutputs) replaces it -- the same pattern the library uses for
// SKOutput<Position>.
template <>
void SKOutput<std::vector<nmea0183::GNSSSatellite>>::as_signalk_json(
    JsonDocument& doc) {
  doc["path"] = this->get_sk_path();
  JsonObject value = doc["value"].to<JsonObject>();
  value["count"] = static_cast<int>(output_.size());
  JsonArray arr = value["satellites"].to<JsonArray>();
  for (const auto& s : output_) {
    JsonObject o = arr.add<JsonObject>();
    o["id"] = s.id;
    if (s.elevation.is_valid()) {
      o["elevation"] = static_cast<float>(s.elevation) * DEG_TO_RAD;
    }
    if (s.azimuth.is_valid()) {
      o["azimuth"] = static_cast<float>(s.azimuth) * DEG_TO_RAD;
    }
    o["SNR"] = s.snr;
  }
}
}  // namespace sensesp

// UM982 UART, ESP32 side. RX21 <- UM982 TXD, TX18 -> UM982 RXD.
constexpr int kUM982RxPin = 21;
constexpr int kUM982TxPin = 18;
constexpr int kUM982ResetPin = 26;
constexpr int kUM982PPSPin = 27;
constexpr uint32_t kUM982BaudRate = 115200;

constexpr uint8_t kN2kSourceAddress = 25;

// Program-lifetime roots (registered by raw pointer elsewhere).
std::shared_ptr<NMEA0183IO> nmea_io;
std::shared_ptr<UM982CommandAckParser> ack_parser;
std::shared_ptr<IntConfig> n2k_address;
std::vector<std::shared_ptr<UM982SettingBase>> um982_settings;
std::shared_ptr<GNSSData> gnss_data;
std::shared_ptr<UnicoreHPRSentenceParser> hpr;
std::shared_ptr<N2kSenders> n2k;

// Signal K outputs. Constructed in setup() -- before SKDeltaQueue snapshots the
// emitter list on the first event-loop tick -- and connected to the parsers
// later in WireOutputs(). Building them in WireOutputs() (after the boot config)
// is too late: they would never be attached to the delta queue, so no deltas are
// ever sent.
std::shared_ptr<SKOutputFloat> sk_heading;
std::shared_ptr<SKOutputAttitudeVector> sk_attitude;
std::shared_ptr<SKOutputString> sk_heading_quality;
std::shared_ptr<SKOutputPosition> sk_position;
std::shared_ptr<SKOutputFloat> sk_sog;
std::shared_ptr<SKOutputFloat> sk_cog;
std::shared_ptr<SKOutputInt> sk_satellites;
std::shared_ptr<SKOutputFloat> sk_hdop;
std::shared_ptr<SKOutputString> sk_datetime;
std::shared_ptr<SKOutput<std::vector<GNSSSatellite>>> sk_satellites_in_view;

// Boot-config sequencer state.
volatile int boot_index = 0;
volatile bool boot_active = false;

// UTC seconds -> ISO 8601 for navigation.datetime.
String TimeToISO8601(const time_t& t) {
  if (t == 0) {
    return String("");
  }
  struct tm tm_utc;
  gmtime_r(&t, &tm_utc);
  char buf[24];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
  return String(buf);
}

// Construct the Signal K outputs. Must run in setup() (see the declarations).
void CreateSKOutputs() {
  sk_heading = std::make_shared<SKOutputFloat>("navigation.headingTrue",
                                               "/SK Path/Heading True");
  sk_attitude = std::make_shared<SKOutputAttitudeVector>(
      "navigation.attitude", "/SK Path/Attitude");
  sk_heading_quality = std::make_shared<SKOutputString>(
      "navigation.gnss.headingQuality", "/SK Path/Heading Quality");
  sk_position = std::make_shared<SKOutputPosition>("navigation.position",
                                                   "/SK Path/Position");
  sk_sog = std::make_shared<SKOutputFloat>("navigation.speedOverGround",
                                           "/SK Path/SOG");
  sk_cog = std::make_shared<SKOutputFloat>("navigation.courseOverGroundTrue",
                                           "/SK Path/COG");
  sk_satellites = std::make_shared<SKOutputInt>("navigation.gnss.satellites",
                                                "/SK Path/Satellites");
  sk_hdop = std::make_shared<SKOutputFloat>("navigation.gnss.horizontalDilution",
                                            "/SK Path/HDOP");
  sk_datetime = std::make_shared<SKOutputString>("navigation.datetime",
                                                 "/SK Path/Datetime");
  sk_satellites_in_view =
      std::make_shared<SKOutput<std::vector<GNSSSatellite>>>(
          "navigation.gnss.satellitesInView", "/SK Path/Satellites In View");
}

// Enable the NMEA output sentences once the module is configured. Period
// argument is in seconds (0.1 = 10 Hz).
void EnableUM982Output() {
  nmea_io->set("GPHPR 0.1");  // heading/pitch/roll/quality at 10 Hz
  nmea_io->set("GPGGA 1");    // position, fix quality, satellites at 1 Hz
  nmea_io->set("GPRMC 1");    // position, SOG, time at 1 Hz
  nmea_io->set("GPVTG 1");    // course over ground at 1 Hz
  nmea_io->set("GPGSV 1");    // satellites in view (constellation) at 1 Hz
}

// Wire the data path. Called only after all UM982 settings are ACK'd, so
// nothing is published before the module is configured.
void WireOutputs() {
  auto* parser = &nmea_io->parser_;

  gnss_data = std::make_shared<GNSSData>();
  ConnectGNSS(parser, gnss_data.get());

  hpr = std::make_shared<UnicoreHPRSentenceParser>(parser);

  // True heading (already offset-corrected on the module) feeds SK and N2K. The
  // parser only emits attitude_ once the dual-antenna baseline is RTK-solved.
  auto yaw = std::make_shared<LambdaTransform<AttitudeVector, float>>(
      [](const AttitudeVector& a) { return a.yaw; });
  hpr->attitude_.connect_to(yaw);
  yaw->connect_to(sk_heading);
  yaw->connect_to(&n2k->heading_);

  hpr->attitude_.connect_to(sk_attitude);
  hpr->attitude_.connect_to(&n2k->attitude_);

  hpr->heading_quality_.connect_to(sk_heading_quality);

  // GNSS position, velocity, fix quality and constellation -- to both SK and N2K.
  gnss_data->position.connect_to(sk_position);
  gnss_data->position.connect_to(&n2k->position_);
  gnss_data->speed.connect_to(sk_sog);
  gnss_data->speed.connect_to(&n2k->sog_);
  gnss_data->true_course.connect_to(sk_cog);
  gnss_data->true_course.connect_to(&n2k->cog_);
  gnss_data->num_satellites.connect_to(sk_satellites);
  gnss_data->num_satellites.connect_to(&n2k->num_satellites_);
  gnss_data->horizontal_dilution.connect_to(sk_hdop);
  gnss_data->horizontal_dilution.connect_to(&n2k->hdop_);
  gnss_data->datetime.connect_to(&n2k->datetime_);
  auto datetime_iso = std::make_shared<LambdaTransform<time_t, String>>(
      [](const time_t& t) { return TimeToISO8601(t); });
  gnss_data->datetime.connect_to(datetime_iso);
  datetime_iso->connect_to(sk_datetime);
  gnss_data->satellites.connect_to(sk_satellites_in_view);
  gnss_data->satellites.connect_to(&n2k->satellites_);

  // Inputs are wired; start publishing N2K now (the bus/diagnostics already came
  // up in setup()).
  n2k->enable_senders();

  EnableUM982Output();
  ESP_LOGI("UM982cfg", "Configuration complete; outputs enabled");
}

// Send the current boot-config command and arm a retry. The sequence advances
// the moment an ACK arrives (see OnBootAck); the timeout only resends a command
// that went unanswered, so a slow/absent module self-heals and outputs stay
// dark until every setting is confirmed.
void SendBootStep(int step) {
  // The UM982 can answer with a burst of ACKs that advances boot_index past the
  // last setting before this queued send runs; bail rather than indexing out of
  // bounds (the LoadProhibited crash this guard replaced).
  if (step < 0 || step >= (int)um982_settings.size()) {
    return;
  }
  String command = um982_settings[step]->command();
  if (command.isEmpty()) {
    // A setting at its auto/default value (e.g. baseline length 0) has no command
    // to push; advance without sending so a non-applicable step can't stall boot.
    ESP_LOGI("UM982cfg", "Skipping [%d]: auto/default, nothing to send", step);
    boot_index = step + 1;
    if (boot_index >= (int)um982_settings.size()) {
      boot_active = false;
      event_loop()->onDelay(0, []() { WireOutputs(); });
    } else {
      int next = boot_index;
      event_loop()->onDelay(0, [next]() { SendBootStep(next); });
    }
    return;
  }
  ESP_LOGI("UM982cfg", "Applying [%d]: %s", step, command.c_str());
  nmea_io->set(command);
  event_loop()->onDelay(2500, [step]() {
    if (boot_active && boot_index == step) {
      ESP_LOGW("UM982cfg", "No ACK for step %d, retrying", step);
      SendBootStep(step);
    }
  });
}

// Called on every command ACK. During boot, advance to the next setting once
// the awaited command is confirmed (or wire outputs when all are confirmed).
void OnBootAck(bool ok) {
  if (!boot_active) {
    return;
  }
  // Correlate: act only on the ACK for the command this step is waiting on (the
  // UM982 echoes the accepted command verbatim). A duplicate from a retried send
  // or a stray ACK from a web-UI save echoes a different command, so it is
  // ignored rather than advancing the sequence.
  if (ack_parser->last_command() != um982_settings[boot_index]->command()) {
    return;
  }
  // An explicit rejection (echo matches, response not OK) is a definitive answer:
  // resending the identical command only repeats the rejection, so advance rather
  // than retry forever. The parser already logged the module's stated reason.
  if (!ok) {
    ESP_LOGW("UM982cfg", "Step %d (%s) rejected by module; skipping",
             boot_index, um982_settings[boot_index]->command().c_str());
  }
  boot_index++;
  if (boot_index >= (int)um982_settings.size()) {
    boot_active = false;
    event_loop()->onDelay(0, []() { WireOutputs(); });
  } else {
    // Snapshot the step: another ACK can advance boot_index before this deferred
    // send runs.
    int next = boot_index;
    event_loop()->onDelay(0, [next]() { SendBootStep(next); });
  }
}

void setup() {
  // Log at INFO, not the default DEBUG: at DEBUG every NMEA line and every full
  // Signal K delta (kilobytes) is written synchronously to the 115200 console,
  // which blocks the event loop for hundreds of ms and corrupts UART input.
  SetupLogging(ESP_LOG_INFO);

  pinMode(kUM982ResetPin, OUTPUT);
  digitalWrite(kUM982ResetPin, HIGH);  // release reset (active low)

  // Enlarge the RX ring buffer (default 256 B ~= 22 ms at 115200) so a brief
  // event-loop stall doesn't overrun it and merge/corrupt UM982 sentences.
  Serial2.setRxBufferSize(1024);
  Serial2.begin(kUM982BaudRate, SERIAL_8N1, kUM982RxPin, kUM982TxPin);

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("gnss-rtk-compass")
                    ->enable_ota("thisisfine")
                    ->get_app();

  // Read NMEA 0183 on the main event loop (NMEA0183IO). The parser outputs feed
  // the N2K senders and SK outputs directly, so reading off a separate task is a
  // cross-core hazard; NMEA0183IO keeps the whole path single-threaded.
  nmea_io = std::make_shared<NMEA0183IO>(&Serial2);
  ack_parser = std::make_shared<UM982CommandAckParser>(&nmea_io->parser_);
  ack_parser->connect_to(std::make_shared<LambdaConsumer<bool>>(OnBootAck));

  // Debug aid: log every raw line read from the UM982 (gated at debug level) so
  // reception is visible on /api/log and serial -- distinguishes "no sentences
  // arriving" from "arriving but not parsed/emitted".
  nmea_io->line_producer_->connect_to(std::make_shared<LambdaConsumer<String>>(
      [](String line) { ESP_LOGD("NMEA0183", "RX: %s", line.c_str()); }));

  // UM982 device settings, configurable from the web UI. Each save() sends the
  // command and waits for the module ACK; the boot sequence applies them all.
  auto* io = nmea_io.get();
  auto* ack = ack_parser.get();

  auto mode_config = std::make_shared<UM982Setting<String>>(
      io, ack, String("FIXLENGTH"), UM982HeadingModeCommand, "mode",
      R"JSON({"type":"object","properties":{"mode":{"title":"Heading mode","type":"array","format":"select","uniqueItems":true,"items":{"type":"string","enum":["FIXLENGTH","VARIABLELENGTH","STATIC","LOWDYNAMIC","TRACTOR"]}}}})JSON",
      "/UM982/Heading Mode");
  ConfigItem(mode_config)
      ->set_title("Heading mode")
      ->set_description(
          "Dual-antenna heading mode. FIXLENGTH suits a rigid, fixed antenna "
          "baseline (the usual boat install).")
      ->set_sort_order(100);

  auto baseline_config = std::make_shared<UM982Setting<int>>(
      io, ack, 0, UM982BaselineLengthCommand, "length_cm",
      R"JSON({"type":"object","properties":{"length_cm":{"title":"Baseline length (cm, 0 = auto)","type":"integer"}}})JSON",
      "/UM982/Baseline Length");
  ConfigItem(baseline_config)
      ->set_title("Antenna baseline length")
      ->set_description(
          "Fixed distance between the two antennas, in cm, to speed up and "
          "stabilise the heading solution. 0 lets the module estimate it.")
      ->set_sort_order(110);

  auto offset_config = std::make_shared<UM982Setting<float>>(
      io, ack, 0.0f, UM982HeadingOffsetCommand, "offset_deg",
      R"JSON({"type":"object","properties":{"offset_deg":{"title":"Heading offset (degrees)","type":"number","multipleOf":0.1}}})JSON",
      "/UM982/Heading Offset");
  ConfigItem(offset_config)
      ->set_title("Heading offset")
      ->set_description(
          "Correction added on the module to the GNSS heading, in degrees "
          "(-180..180), to align the antenna baseline with the vessel heading.")
      ->set_sort_order(120);

  auto antijam_config = std::make_shared<UM982Setting<String>>(
      io, ack, String("AUTO"), UM982AntiJamCommand, "antijam",
      R"JSON({"type":"object","properties":{"antijam":{"title":"Anti-jamming","type":"array","format":"select","uniqueItems":true,"items":{"type":"string","enum":["DISABLE","AUTO","FORCE"]}}}})JSON",
      "/UM982/Anti-Jamming");
  ConfigItem(antijam_config)
      ->set_title("Anti-jamming")
      ->set_description(
          "GNSS anti-jamming mode. AUTO is the default; FORCE always on "
          "(higher power draw).")
      ->set_sort_order(130);

  auto antispoof_config = std::make_shared<UM982Setting<String>>(
      io, ack, String("DISABLE"), UM982AntiSpoofCommand, "antispoof",
      R"JSON({"type":"object","properties":{"antispoof":{"title":"Anti-spoofing","type":"array","format":"select","uniqueItems":true,"items":{"type":"string","enum":["DISABLE","ENABLE"]}}}})JSON",
      "/UM982/Anti-Spoofing");
  ConfigItem(antispoof_config)
      ->set_title("Anti-spoofing")
      ->set_description("GNSS anti-spoofing protection.")
      ->set_sort_order(140);

  auto smoothing_config = std::make_shared<UM982Setting<int>>(
      io, ack, 0, UM982SmoothHeadingCommand, "smoothing",
      R"JSON({"type":"object","properties":{"smoothing":{"title":"Heading smoothing (epochs, 0 = off)","type":"integer"}}})JSON",
      "/UM982/Heading Smoothing");
  ConfigItem(smoothing_config)
      ->set_title("Heading smoothing")
      ->set_description(
          "Heading smoothing window in epochs (0-100). 0 turns smoothing off.")
      ->set_sort_order(150);

  um982_settings = {mode_config,    baseline_config,  offset_config,
                    antijam_config, antispoof_config, smoothing_config};

  // N2K source address (applied when N2kSenders is constructed, after config).
  n2k_address = std::make_shared<IntConfig>(kN2kSourceAddress, "Source address",
                                            "/NMEA 2000/Source Address");
  ConfigItem(n2k_address)
      ->set_title("NMEA 2000 source address")
      ->set_description(
          "Device address on the NMEA 2000 bus (0-251). Must be unique. Takes "
          "effect after a restart.")
      ->set_requires_restart(true)
      ->set_sort_order(300);

  // Bring up NMEA 2000 (bus, message counters, watchdog) now, before the event
  // loop starts ticking -- so the status/config registries are populated before
  // the HTTP server serves, and the bus + watchdog exist regardless of whether
  // the UM982 boot config later succeeds. Only the data senders are gated on
  // configuration (enable_senders(), called from WireOutputs()).
  n2k = std::make_shared<N2kSenders>(
      static_cast<uint8_t>(n2k_address->get_value()));

  // Construct the Signal K outputs now, before the event loop starts, so they
  // register with the SK delta queue. WireOutputs() connects them once the UM982
  // is configured.
  CreateSKOutputs();

  // Heap and main-loop-stack diagnostics on /api/info (parity with ais/wind).
  auto largest_block_status = std::make_shared<StatusPageItem<int>>(
      "Largest free block (bytes)", 0, "System", 250);
  event_loop()->onRepeat(2000, [largest_block_status]() {
    largest_block_status->set(static_cast<int>(ESP.getMaxAllocHeap()));
  });

  auto main_loop_stack_status = std::make_shared<StatusPageItem<int>>(
      "Main loop min free stack (bytes)", 0, "System", 260);
  event_loop()->onRepeat(2000, [main_loop_stack_status]() {
    main_loop_stack_status->set(
        static_cast<int>(uxTaskGetStackHighWaterMark(nullptr)));
  });

  // Give the module time to boot, then apply config; outputs wire up once all
  // settings are ACK'd.
  boot_active = true;
  event_loop()->onDelay(500, []() { SendBootStep(0); });
}

void loop() { event_loop()->tick(); }
