#pragma once

#include <NMEA2000.h>

#include <vector>

#include "sensesp/system/expiring_value.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/types/position.h"
#include "sensesp_nmea0183/data/gnss_data.h"

namespace gnss_rtk_compass {

using namespace sensesp;

/**
 * @brief NMEA 2000 output for the GNSS RTK compass.
 *
 * Initializes the ESP32 CAN/N2K interface and periodically transmits heading,
 * rate of turn, position, COG/SOG and GNSS position data. Inputs arrive through the
 * public LambdaConsumers and expire if they stop updating, so a stale value is
 * sent as "not available" rather than frozen.
 *
 * Inputs use Signal K SI units: angles in radians, speed in m/s, position in
 * degrees.
 */
class N2kSenders {
 public:
  // The constructor brings up the N2K bus, message counters, and watchdog (call
  // it from setup(), before the event loop starts, so the status/config
  // registries are populated before the HTTP server serves). The periodic
  // data-publishing senders stay dark until enable_senders() is called -- gate
  // that on the sensor being configured so no nav data is published early.
  explicit N2kSenders(uint8_t source_address);

  void enable_senders();

  LambdaConsumer<float> heading_;        // true heading, rad
  LambdaConsumer<float> rate_of_turn_;   // yaw rate, rad/s
  LambdaConsumer<Position> position_;
  LambdaConsumer<float> cog_;  // course over ground, rad true
  LambdaConsumer<float> sog_;  // speed over ground, m/s
  LambdaConsumer<int> num_satellites_;
  LambdaConsumer<float> hdop_;
  LambdaConsumer<time_t> datetime_;
  LambdaConsumer<std::vector<nmea0183::GNSSSatellite>> satellites_;
  LambdaConsumer<float> variation_;  // magnetic variation, rad

 private:
  ExpiringValue<double> heading_v_;
  ExpiringValue<double> rate_of_turn_v_;
  ExpiringValue<Position> position_v_;
  ExpiringValue<double> cog_v_;
  ExpiringValue<double> sog_v_;
  ExpiringValue<int> num_satellites_v_;
  ExpiringValue<double> hdop_v_;
  ExpiringValue<time_t> datetime_v_;
  ExpiringValue<std::vector<nmea0183::GNSSSatellite>> satellites_v_;
  // Magnetic variation expires on kVariationExpiry (much longer than kExpiry):
  // the receiver fills the RMC variation field only intermittently, so a short
  // expiry flaps to not-available in the gaps while no expiry would broadcast a
  // stale declination forever if the receiver stops. A long expiry bridges the
  // gaps yet still degrades to not-available on a genuine dropout.
  ExpiringValue<double> variation_v_;
};

}  // namespace gnss_rtk_compass
