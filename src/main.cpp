// GNSS RTK Compass — UM982 dual-antenna satellite compass on SH-ESP32.
//
// Reads the UM982 serial output, derives true heading from the antenna
// baseline, and publishes heading, attitude, position, speed/course and fix
// quality to both Signal K and NMEA 2000. See SPEC.md.

#include <memory>

#include "sensesp.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/signalk/signalk_types.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/transforms/angle_correction.h"
#include "sensesp/transforms/lambda_transform.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/wiring.h"

#include "gnhpr_parser.h"
#include "n2k_senders.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

// UM982 UART, ESP32 side. RX21 <- UM982 TXD, TX18 -> UM982 RXD.
constexpr int kUM982RxPin = 21;
constexpr int kUM982TxPin = 18;
constexpr int kUM982ResetPin = 26;
constexpr int kUM982PPSPin = 27;
constexpr uint32_t kUM982BaudRate = 115200;

// Unique address on the NMEA 2000 bus (see SPEC.md).
constexpr uint8_t kN2kSourceAddress = 25;

// Held for the program lifetime: the NMEA 0183 parser registers sentence
// parsers by raw pointer and ConnectGNSS references the GNSSData by raw
// pointer, so these roots must not be destroyed.
std::shared_ptr<NMEA0183IOTask> nmea_io;
std::shared_ptr<GNSSData> gnss_data;
std::shared_ptr<gnss_rtk_compass::UnicoreHPRSentenceParser> hpr;
std::shared_ptr<gnss_rtk_compass::N2kSenders> n2k;

// Put the UM982 into dual-antenna heading mode and enable the NMEA sentences
// we consume. Sent on every boot so the module is self-configuring after a
// power loss or replacement. Period argument is in seconds (0.1 = 10 Hz).
void ConfigureUM982(Stream* stream) {
  static const char* commands[] = {
      "MODE HEADING2 FIXLENGTH",  // dual-antenna heading, fixed baseline
      "GPHPR 0.1",                // heading/pitch/roll/quality at 10 Hz
      "GPGGA 1",                  // position, fix quality, satellites at 1 Hz
      "GPRMC 1",                  // position, SOG, time at 1 Hz
      "GPVTG 1",                  // course over ground at 1 Hz
  };
  // Config is reapplied on every boot, so it is not persisted with SAVECONFIG
  // (which would wear the module's flash).
  for (const char* command : commands) {
    stream->printf("%s\r\n", command);
    delay(100);
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

  ConfigureUM982(&Serial2);

  nmea_io = std::make_shared<NMEA0183IOTask>(&Serial2);

  // TEMP (hardware bring-up): log every raw line received from the UM982.
  nmea_io->line_producer_->connect_to(
      std::make_shared<LambdaConsumer<String>>([](const String& line) {
        ESP_LOGI("UM982", "RX: %s", line.c_str());
      }));

  auto* parser = &nmea_io->parser_;

  // Standard GNSS sentences -> Signal K (position, SOG, COG, sats, quality).
  gnss_data = std::make_shared<GNSSData>();
  ConnectGNSS(parser, gnss_data.get());

  // Dual-antenna heading/attitude from the UM982 $GNHPR sentence.
  hpr = std::make_shared<gnss_rtk_compass::UnicoreHPRSentenceParser>(parser);

  n2k = std::make_shared<gnss_rtk_compass::N2kSenders>(kN2kSourceAddress);

  // True heading: a single corrected value feeds both Signal K and NMEA 2000.
  auto heading_true =
      hpr->attitude_
          .connect_to(std::make_shared<LambdaTransform<AttitudeVector, float>>(
              [](const AttitudeVector& a) { return a.yaw; }))
          ->connect_to(std::make_shared<AngleCorrection>(0, 0,
                                                         "/Heading/Offset"));
  auto sk_heading = std::make_shared<SKOutputFloat>("navigation.headingTrue",
                                                    "/SK Path/Heading True");
  // set_metadata copies its argument, so a stack-local is safe (avoids `new`).
  SKMetadata heading_meta("rad", "True Heading", "GNSS dual-antenna true heading",
                          "Heading", 30);
  sk_heading->set_metadata(&heading_meta);
  heading_true->connect_to(sk_heading);
  heading_true->connect_to(&n2k->heading_);

  // The "/Heading/Offset" correction applies to the heading outputs above.
  // Attitude reports the raw baseline (roll/pitch/yaw straight from the module).
  hpr->attitude_.connect_to(std::make_shared<SKOutputAttitudeVector>(
      "navigation.attitude", "/SK Path/Attitude"));
  hpr->attitude_.connect_to(&n2k->attitude_);

  hpr->heading_quality_.connect_to(std::make_shared<SKOutputString>(
      "navigation.gnss.headingQuality", "/SK Path/Heading Quality"));

  // NMEA 2000 position/COG/SOG/GNSS inputs (Signal K outputs are wired by
  // ConnectGNSS above).
  gnss_data->position.connect_to(&n2k->position_);
  gnss_data->true_course.connect_to(&n2k->cog_);
  gnss_data->speed.connect_to(&n2k->sog_);
  gnss_data->num_satellites.connect_to(&n2k->num_satellites_);
  gnss_data->horizontal_dilution.connect_to(&n2k->hdop_);
  gnss_data->datetime.connect_to(&n2k->datetime_);
}

void loop() { event_loop()->tick(); }
