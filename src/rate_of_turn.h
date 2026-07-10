#pragma once

#include <Arduino.h>

#include <deque>
#include <utility>

#include "sensesp/transforms/transform.h"

namespace gnss_rtk_compass {

using namespace sensesp;

// Derives rate of turn (yaw rate, rad/s) from a stream of true-heading samples
// (rad). Fits a least-squares slope over a sliding time window, so it tolerates
// the receiver's irregular sample timing and rate. Heading is unwrapped across
// the 0/2pi seam; a gap longer than the window resets the estimator so a stale
// sample can't inject a spurious jump on recovery. Positive = turning to
// starboard, matching NMEA 2000 PGN 127251 and Signal K navigation.rateOfTurn.
class HeadingRateOfTurn : public Transform<float, float> {
 public:
  explicit HeadingRateOfTurn(unsigned long window_ms)
      : Transform<float, float>(""), window_ms_(window_ms) {}

  void set(const float& heading) override {
    unsigned long now = millis();

    if (have_last_ && now - last_ms_ > window_ms_) {
      buf_.clear();
      have_last_ = false;
    }

    if (!have_last_) {
      unwrapped_ = heading;
    } else {
      double d = heading - last_raw_;
      while (d > PI) d -= TWO_PI;
      while (d < -PI) d += TWO_PI;
      unwrapped_ += d;
    }
    last_raw_ = heading;
    last_ms_ = now;
    have_last_ = true;

    buf_.emplace_back(now, unwrapped_);
    while (!buf_.empty() && now - buf_.front().first > window_ms_) {
      buf_.pop_front();
    }

    if (buf_.size() < kMinSamples ||
        buf_.back().first - buf_.front().first < kMinSpanMs) {
      return;
    }

    // Least-squares slope of heading (rad) against time (s) = rate of turn.
    unsigned long t0 = buf_.front().first;
    double n = buf_.size(), st = 0, sy = 0, stt = 0, sty = 0;
    for (const auto& s : buf_) {
      double t = (s.first - t0) / 1000.0;
      st += t;
      sy += s.second;
      stt += t * t;
      sty += t * s.second;
    }
    double denom = n * stt - st * st;
    if (denom <= 0) return;
    this->emit((n * sty - st * sy) / denom);
  }

 private:
  // Two samples define a slope; the span guard keeps a bursty pair from
  // producing a wild finite difference.
  static constexpr size_t kMinSamples = 2;
  static constexpr unsigned long kMinSpanMs = 250;

  unsigned long window_ms_;
  std::deque<std::pair<unsigned long, double>> buf_;
  double unwrapped_ = 0;
  float last_raw_ = 0;
  unsigned long last_ms_ = 0;
  bool have_last_ = false;
};

}  // namespace gnss_rtk_compass
