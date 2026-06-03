// GNSS RTK Compass — UM982 dual-antenna satellite compass on SH-ESP32.
//
// Reads the UM982 serial output, derives true heading from the antenna
// baseline, and publishes heading, position, speed/course and fix quality to
// Signal K (and, once implemented, NMEA 2000). See SPEC.md.

#include <memory>

#include "sensesp.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/wiring.h"

#include "gnhpr_parser.h"

using namespace sensesp;
using namespace sensesp::nmea0183;

// UM982 UART, ESP32 side. RX21 <- UM982 TXD, TX18 -> UM982 RXD.
constexpr int kUM982RxPin = 21;
constexpr int kUM982TxPin = 18;
constexpr int kUM982ResetPin = 26;
constexpr int kUM982PPSPin = 27;
constexpr uint32_t kUM982BaudRate = 115200;

std::shared_ptr<NMEA0183IOTask> nmea_io;

// Put the UM982 into dual-antenna heading mode and enable the NMEA sentences
// we consume. Sent on every boot so the module is self-configuring after a
// power loss or replacement. Period argument is in seconds (0.1 = 10 Hz).
void ConfigureUM982(Stream* stream) {
  static const char* commands[] = {
      "MODE HEADING2 FIXLENGTH",  // dual-antenna heading, fixed baseline
      "GPHPR 0.1",                // heading/pitch/roll/quality at 10 Hz
      "GPGGA 1",                  // position, fix quality, satellites at 1 Hz
      "GPRMC 1",                  // position, SOG, COG, time at 1 Hz
      "SAVECONFIG",
  };
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

  // TEMP (hardware bring-up): log every raw line received from the UM982 so we
  // can confirm wiring and baud rate before relying on the parsers.
  nmea_io->line_producer_->connect_to(
      new LambdaConsumer<String>([](const String& line) {
        ESP_LOGI("UM982", "RX: %s", line.c_str());
      }));

  // Standard GNSS sentences: position, SOG, COG, satellites, fix quality.
  ConnectGNSS(&nmea_io->parser_, new GNSSData());

  // Dual-antenna heading/attitude from the UM982 $GNHPR sentence.
  gnss_rtk_compass::ConnectUM982Heading(&nmea_io->parser_);

  // TODO(SPEC.md): NMEA 2000 senders for PGNs 127250, 127257, 129025, 129026,
  // 129029. Port the sender pattern to the SensESP 3.x API.
}

void loop() { event_loop()->tick(); }
