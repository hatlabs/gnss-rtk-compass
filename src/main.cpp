// GNSS RTK Compass — UM982 dual-antenna satellite compass on SH-ESP32.
//
// Reads the UM982 serial output, derives true heading from the antenna
// baseline, and publishes heading, attitude, position, speed/course and fix
// quality to both Signal K and NMEA 2000. See SPEC.md.
//
// Boot order matters: the UM982 settings (mode, baseline, offset, anti-jam,
// anti-spoof, smoothing) are applied and ACK-confirmed before any output is
// wired, so no heading/position is published until the module is configured.

#include <memory>
#include <vector>

#include "sensesp.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_types.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp/ui/config_item.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/wiring.h"

#include "gnhpr_parser.h"
#include "n2k_senders.h"
#include "um982_config.h"

using namespace sensesp;
using namespace sensesp::nmea0183;
using namespace gnss_rtk_compass;

// UM982 UART, ESP32 side. RX21 <- UM982 TXD, TX18 -> UM982 RXD.
constexpr int kUM982RxPin = 21;
constexpr int kUM982TxPin = 18;
constexpr int kUM982ResetPin = 26;
constexpr int kUM982PPSPin = 27;
constexpr uint32_t kUM982BaudRate = 115200;

constexpr uint8_t kN2kSourceAddress = 25;

// Program-lifetime roots (registered by raw pointer elsewhere).
std::shared_ptr<NMEA0183IOTask> nmea_io;
std::shared_ptr<UM982CommandAckParser> ack_parser;
std::shared_ptr<IntConfig> n2k_address;
std::vector<std::shared_ptr<UM982SettingBase>> um982_settings;
std::shared_ptr<GNSSData> gnss_data;
std::shared_ptr<UnicoreHPRSentenceParser> hpr;
std::shared_ptr<N2kSenders> n2k;

// Boot-config sequencer state.
volatile int boot_index = 0;
volatile bool boot_active = false;

// Enable the NMEA output sentences once the module is configured. Period
// argument is in seconds (0.1 = 10 Hz).
void EnableUM982Output() {
  nmea_io->set("GPHPR 0.1");  // heading/pitch/roll/quality at 10 Hz
  nmea_io->set("GPGGA 1");    // position, fix quality, satellites at 1 Hz
  nmea_io->set("GPRMC 1");    // position, SOG, time at 1 Hz
  nmea_io->set("GPVTG 1");    // course over ground at 1 Hz
}

// Wire the data path. Called only after all UM982 settings are ACK'd, so
// nothing is published before the module is configured.
void WireOutputs() {
  auto* parser = &nmea_io->parser_;

  gnss_data = std::make_shared<GNSSData>();
  ConnectGNSS(parser, gnss_data.get());

  hpr = std::make_shared<UnicoreHPRSentenceParser>(parser);
  n2k = std::make_shared<N2kSenders>(
      static_cast<uint8_t>(n2k_address->get_value()));

  // True heading (already offset-corrected on the module) feeds SK and N2K.
  auto yaw = std::make_shared<LambdaTransform<AttitudeVector, float>>(
      [](const AttitudeVector& a) { return a.yaw; });
  hpr->attitude_.connect_to(yaw);

  auto sk_heading = std::make_shared<SKOutputFloat>("navigation.headingTrue",
                                                    "/SK Path/Heading True");
  SKMetadata heading_meta("rad", "True Heading", "GNSS dual-antenna true heading",
                          "Heading", 30);
  sk_heading->set_metadata(&heading_meta);
  yaw->connect_to(sk_heading);
  yaw->connect_to(&n2k->heading_);

  hpr->attitude_.connect_to(std::make_shared<SKOutputAttitudeVector>(
      "navigation.attitude", "/SK Path/Attitude"));
  hpr->attitude_.connect_to(&n2k->attitude_);

  hpr->heading_quality_.connect_to(std::make_shared<SKOutputString>(
      "navigation.gnss.headingQuality", "/SK Path/Heading Quality"));

  gnss_data->position.connect_to(&n2k->position_);
  gnss_data->true_course.connect_to(&n2k->cog_);
  gnss_data->speed.connect_to(&n2k->sog_);
  gnss_data->num_satellites.connect_to(&n2k->num_satellites_);
  gnss_data->horizontal_dilution.connect_to(&n2k->hdop_);
  gnss_data->datetime.connect_to(&n2k->datetime_);

  EnableUM982Output();
  ESP_LOGI("UM982cfg", "Configuration complete; outputs enabled");
}

// Send the current boot-config command and arm a retry. The sequence advances
// the moment an ACK arrives (see OnBootAck); the timeout only resends a command
// that went unanswered, so a slow/absent module self-heals and outputs stay
// dark until every setting is confirmed.
void SendBootStep() {
  int step = boot_index;
  String command = um982_settings[step]->command();
  ESP_LOGI("UM982cfg", "Applying: %s", command.c_str());
  nmea_io->set(command);
  event_loop()->onDelay(2500, [step]() {
    if (boot_active && boot_index == step) {
      ESP_LOGW("UM982cfg", "No ACK for step %d, retrying", step);
      SendBootStep();
    }
  });
}

// Called on every command ACK. During boot, advance to the next setting
// immediately (or wire outputs once all are confirmed).
void OnBootAck(bool ok) {
  if (!boot_active || !ok) {
    return;
  }
  boot_index++;
  if (boot_index >= (int)um982_settings.size()) {
    boot_active = false;
    event_loop()->onDelay(0, []() { WireOutputs(); });
  } else {
    event_loop()->onDelay(0, []() { SendBootStep(); });
  }
}

void setup() {
  SetupLogging();

  pinMode(kUM982ResetPin, OUTPUT);
  digitalWrite(kUM982ResetPin, HIGH);  // release reset (active low)

  Serial2.begin(kUM982BaudRate, SERIAL_8N1, kUM982RxPin, kUM982TxPin);

  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    ->set_hostname("gnss-rtk-compass")
                    ->enable_ota("thisisfine")
                    ->get_app();

  nmea_io = std::make_shared<NMEA0183IOTask>(&Serial2);
  ack_parser = std::make_shared<UM982CommandAckParser>(&nmea_io->parser_);
  ack_parser->connect_to(std::make_shared<LambdaConsumer<bool>>(OnBootAck));

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

  // Give the module time to boot, then apply config; outputs wire up once all
  // settings are ACK'd.
  boot_active = true;
  event_loop()->onDelay(500, []() { SendBootStep(); });
}

void loop() { event_loop()->tick(); }
