#pragma once

#include <math.h>

#include "sensesp.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp/system/saveable.h"
#include "sensesp/system/semaphore_value.h"
#include "sensesp/system/serializable.h"
#include "sensesp_nmea0183/nmea0183.h"
#include "sensesp_nmea0183/sentence_parser/field_parsers.h"
#include "sensesp_nmea0183/sentence_parser/sentence_parser.h"

namespace gnss_rtk_compass {

using namespace sensesp;

/**
 * @brief Parses UM982 command acknowledgements.
 *
 * The module echoes accepted commands as "$command,<cmd>,response: OK*cs".
 * Emits true on OK, false otherwise. Checksum is ignored; the response content
 * is what matters.
 */
class UM982CommandAckParser : public nmea0183::SentenceParser {
 public:
  UM982CommandAckParser(nmea0183::NMEA0183Parser* nmea) : SentenceParser(nmea) {
    ignore_checksum(true);
  }
  const char* sentence_address() override final { return "command"; }
  bool parse_fields(const char* field_strings, const int field_offsets[],
                    int num_fields) override final {
    if (num_fields < 3) {
      return false;
    }
    String response;
    nmea0183::ParseString(&response,
                          field_strings + field_offsets[num_fields - 1]);
    emit(response.indexOf("OK") >= 0);
    return true;
  }
};

// --- Command builders -----------------------------------------------------

inline String UM982HeadingModeCommand(const String& mode) {
  return "MODE HEADING2 " + mode;
}
inline String UM982AntiJamCommand(const String& mode) {
  return "CONFIG ANTIJAM " + mode;
}
inline String UM982AntiSpoofCommand(const String& mode) {
  return "CONFIG ANTISPOOF " + mode;
}
inline String UM982BaselineLengthCommand(const float& length_cm) {
  // 0 restores automatic baseline estimation.
  char buf[48];
  snprintf(buf, sizeof(buf), "CONFIG HEADING LENGTH %d",
           length_cm > 0 ? (int)lroundf(length_cm) : 0);
  return buf;
}
inline String UM982HeadingOffsetCommand(const float& heading_deg) {
  char buf[48];
  snprintf(buf, sizeof(buf), "CONFIG HEADING OFFSET %.1f 0", heading_deg);
  return buf;
}
inline String UM982SmoothHeadingCommand(const float& window) {
  char buf[48];
  snprintf(buf, sizeof(buf), "CONFIG SMOOTH HEADING %d", (int)lroundf(window));
  return buf;
}

// --- Settings -------------------------------------------------------------

/// Non-template interface so the boot sequencer can hold a list of settings.
class UM982SettingBase {
 public:
  virtual ~UM982SettingBase() = default;
  virtual String command() const = 0;
};

/**
 * @brief A UM982 setting: a value persisted to flash and pushed to the module.
 *
 * save() (web UI edit) persists, sends the command, and waits for the ACK.
 * The boot sequencer instead reads command() and drives the ACK-gated apply
 * itself (it runs on the event-loop task and must not block).
 */
template <typename T>
class UM982Setting : public FileSystemSaveable,
                     public UM982SettingBase,
                     virtual public Serializable {
 public:
  using Builder = String (*)(const T&);

  UM982Setting(nmea0183::NMEA0183IOTask* io_task, UM982CommandAckParser* ack,
               T default_value, Builder builder, const char* json_key,
               const char* schema, String config_path)
      : FileSystemSaveable(config_path),
        io_task_{io_task},
        value_{default_value},
        builder_{builder},
        json_key_{json_key},
        schema_{schema} {
    load();
    ack->connect_to(&ack_semaphore_);
  }

  bool to_json(JsonObject& doc) override {
    doc[json_key_] = value_;
    return true;
  }
  bool from_json(const JsonObject& config) override {
    if (!config[json_key_].is<JsonVariant>()) {
      return false;
    }
    value_ = config[json_key_].as<T>();
    return true;
  }

  String command() const override { return builder_(value_); }
  const char* get_config_schema() const { return schema_; }

  // Web UI edit: persist, send, wait for ACK. Called off the event-loop task,
  // so the deferred send runs while take() blocks here.
  bool save() override {
    FileSystemSaveable::save();
    String sentence = command();
    ack_semaphore_.clear();
    event_loop()->onDelay(0, [this, sentence]() { io_task_->set(sentence); });
    return ack_semaphore_.take(2000);
  }

 protected:
  nmea0183::NMEA0183IOTask* io_task_;
  T value_;
  Builder builder_;
  const char* json_key_;
  const char* schema_;
  SemaphoreValue<bool> ack_semaphore_;
};

template <typename T>
const String ConfigSchema(const UM982Setting<T>& obj) {
  return obj.get_config_schema();
}

}  // namespace gnss_rtk_compass
