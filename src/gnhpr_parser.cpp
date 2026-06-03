#include "gnhpr_parser.h"

#include <Arduino.h>

#include <functional>

#include "sensesp_nmea0183/sentence_parser/field_parsers.h"

namespace gnss_rtk_compass {

// Indexed by the UM982 HPR quality field.
static const char* kHeadingQualityStrings[] = {
    "no fix",    "single",         "DGNSS",          "PPP",
    "RTK fixed", "RTK float",      "dead reckoning", "manual",
    "wide-lane", "SBAS"};
constexpr int kNumQualityStrings =
    sizeof(kHeadingQualityStrings) / sizeof(kHeadingQualityStrings[0]);

bool UnicoreHPRSentenceParser::parse_fields(const char* field_strings,
                                            const int field_offsets[],
                                            int num_fields) {
  // utc, heading, pitch, roll, quality, satellites are the fields we use.
  if (num_fields < 7) {
    return false;
  }

  float utc = 0;  // consumed but unused; time comes from GGA/RMC
  float heading_deg = 0;
  float pitch_deg = 0;
  float roll_deg = 0;
  int quality = 0;
  int num_satellites = 0;

  std::function<bool(const char*)> fps[] = {
      FLDP_OPT(Float, &utc),
      FLDP_OPT(Float, &heading_deg),
      FLDP_OPT(Float, &pitch_deg),
      FLDP_OPT(Float, &roll_deg),
      FLDP(Int, &quality),
      FLDP_OPT(Int, &num_satellites),
  };

  bool ok = true;
  for (int i = 1; i <= sizeof(fps) / sizeof(fps[0]); i++) {
    ok &= fps[i - 1](field_strings + field_offsets[i]);
  }
  if (!ok) {
    return false;
  }

  if (quality >= 0 && quality < kNumQualityStrings) {
    heading_quality_.set(kHeadingQualityStrings[quality]);
  }
  num_satellites_.set(num_satellites);

  // The baseline solution yields a usable heading when fixed (4) or float (5).
  // Quality is published regardless so a degraded solution is visible.
  if (quality == 4 || quality == 5) {
    attitude_.set(AttitudeVector(roll_deg * DEG_TO_RAD, pitch_deg * DEG_TO_RAD,
                                 heading_deg * DEG_TO_RAD));
  }

  return true;
}

}  // namespace gnss_rtk_compass
