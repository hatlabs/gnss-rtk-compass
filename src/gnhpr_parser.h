#pragma once

#include "sensesp/system/observablevalue.h"
#include "sensesp/types/position.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace gnss_rtk_compass {

using namespace sensesp;
using namespace sensesp::nmea0183;

/**
 * @brief Parser for the Unicore $--HPR sentence (GPHPR log).
 *
 * Emitted by the UM982 in dual-antenna heading mode. Carries true heading,
 * pitch and roll derived from the antenna baseline, plus a solution-quality
 * indicator. Angles are converted to radians (Signal K SI) before emitting.
 *
 * Example: $GNHPR,074615.00,320.9610,-66.1712,000.0000,4,47,0.00,0999*45
 *          fields: utc, heading, pitch, roll, quality, satellites, ...
 */
class UnicoreHPRSentenceParser : public SentenceParser {
 public:
  UnicoreHPRSentenceParser(NMEA0183Parser* nmea) : SentenceParser(nmea) {}

  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final;
  const char* sentence_address() override { return "..HPR"; }

  ObservableValue<AttitudeVector> attitude_;  // radians
  ObservableValue<String> heading_quality_;
  ObservableValue<int> num_satellites_;
};

}  // namespace gnss_rtk_compass
