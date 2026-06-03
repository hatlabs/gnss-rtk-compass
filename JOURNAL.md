# Work Journal

## 2026-06-03 — Project kickoff and scaffolding

### GNSS RTK Compass (UM982 on SH-ESP32)

- Phase 0: system-profile.md exists (Hurma, C&C 36-1, 12V, N2K backbone,
  Signal K on halos.hurma). No existing gnss-rtk-compass project. User pointed
  at a half-finished reference repo at /Users/mairas/projects/mairas/sensesp-rtk
  (started as an AIS transponder interface, partially adapted to RTK) — read-only
  inspiration, not the basis repo.
- Phase 1: Requirements gathered.
  - Data scope: GNSS heading, position/SOG/COG, RTK fix status, + pitch/roll
    (user opted to add pitch/roll since $GNHPR provides it for free).
  - Outputs: both Signal K (WiFi) and NMEA 2000.
  - RTK corrections: NONE. Device runs the UM982 in dual-antenna moving-base /
    heading mode — two antennas on the module, only the relative baseline vector
    is solved. No NTRIP. Absolute position stays single-point (~1.5 m); "RTK fix"
    here = heading/baseline solution quality.
- Phase 2: Hardware.
  - Board SH-ESP32 (ESP32-WROOM-32E, 4 MB flash).
  - Wiring (ESP32 side, after user corrected TX/RX — original recital used UM982
    board pin labels): RX=GPIO21 (<- UM982 TXD), TX=GPIO18 (-> UM982 RXD),
    RST=GPIO26 (output), PPS=GPIO27 (input). All free SH-ESP32 pins; no conflict
    with CAN 32/34, I2C 16/17, 1-Wire 4, opto 33/35. UM982 is 3.3 V logic.
  - Grounded UM982 specifics in the manuals (Google Drive Reference/UM982):
    MODE HEADING2 FIXLENGTH for dual-antenna heading; $GNHPR (GPHPR log) gives
    heading/pitch/roll + solution-quality (4=RTK fix, 5=float, 1=single);
    standard $GNGGA/$GNRMC for position/SOG/COG. Log rate syntax is
    `<MSG> <period_seconds>` (0.1 = 10 Hz).
- Phase 3: Wrote SPEC.md. User confirmed, with two decisions:
  - Publish pitch/roll too (navigation.attitude + PGN 127257).
  - $GNHPR parser implemented project-locally as a SentenceParser subclass
    (not contributed upstream for now).
- Phase 4: Scaffolded project — platformio.ini (shesp32 default env, SensESP
  3.3 + NMEA0183 + NMEA2000 + NMEA2000_twai deps), src/main.cpp skeleton
  (UART2 on 21/18, UM982 boot config, ConnectGNSS wired), JOURNAL.md, git init.

### Key finding / next work
- The NMEA0183 library (ref/NMEA0183) has RTKData (attitude, baseline_*,
  rtk_quality) and wiring helpers (ConnectGNSS, ConnectQuectelRTK) but NO
  $GNHPR parser. The reference project's ConnectQuectelRTK call was a
  placeholder that would not parse UM982 output.
- Next: (1) project-local GNHPRSentenceParser → RTKData → SK headingTrue/
  attitude; (2) NMEA 2000 senders (127250, 127257, 129025, 129026, 129029)
  ported to SensESP 3.x API; (3) build + flash + bench test.

### Build + hardware bring-up (2026-06-03, same day)
- Build blocked by a SensESP 3.3.0 bug: StreamLineProducer::receive_line()
  calls `emit(buf_)` with buf_ a unique_ptr<char[]> where emit wants const
  String&. New file in 3.3.0, never compiled (no example exercises it).
  NMEA0183 3.1.1 hard-depends on StreamLineProducer, so no published version
  combo builds. Fix: `emit(String(buf_.get()))`. Verified the fix compiles.
  Decision: fix upstream in SignalK/SensESP and release 3.3.1 (PR opened).
- Fixed our own main.cpp: published NMEA0183 3.1.1 class is NMEA0183IOTask,
  not NMEA0183IO (ref/NMEA0183 is a newer diverged copy).
- Flash 86.8% (1.71 MB) on min_spiffs — watch headroom as N2K senders land.
- Flashed to /dev/cu.usbserial-2010. Serial capture CONFIRMS hardware:
  $GNHPR at ~10 Hz + $GNGGA/$GNRMC at 1 Hz — the exact rates we configured,
  proving both RX (GPIO21) and TX (GPIO18, module accepted MODE HEADING2 +
  log config) are wired correctly. All fields zero / QF=0 / 0 sats: no
  satellite fix yet (antennas need open sky). Link + config fully validated.
- Stale SK server config on device points at oppi4.hal:3000 (doesn't resolve).
  WiFi is up. Repoint to halos.hurma via the web UI later. Not a code issue.
- Added a TEMP raw-line logger to main.cpp for bring-up; remove before finalize.

### $GNHPR parser implemented (2026-06-03)
- SensESP fix PR #966 MERGED to main (9c69ba4); 3.3.1 release deferred (user
  reviewing other SensESP PRs first). Project lib_deps pinned to that commit;
  clean rebuild from git confirms no manual .pio patch needed.
- Added project-local src/gnhpr_parser.{h,cpp}: UnicoreHPRSentenceParser
  (address "..HPR") parsing utc/heading/pitch/roll/quality/satellites. Converts
  deg->rad (SK is SI). Emits attitude only when quality 4 (fixed) or 5 (float);
  always publishes quality string. Mirrors the library's QuectelPQTMTAR pattern.
- ConnectUM982Heading wires: attitude.yaw -> AngleCorrection("/Heading/Offset")
  -> navigation.headingTrue (rad, with metadata); attitude ->
  navigation.attitude; quality -> navigation.gnss.headingQuality.
- Verified on device: clean boot, no crash, parser integrated. UM982 ACKs
  "MODE HEADING2 FIXLENGTH response: OK". Occasional corrupt UART frames are
  correctly rejected by checksum validation. QF=0 indoors so no heading emitted
  (gating correct). Flash now 87.5%.
- NOT YET VERIFIED: actual heading values — needs outdoor satellite fix (QF>=4).

### NMEA 2000 senders implemented (2026-06-03)
- Refactored: all SK+N2K wiring now in main.cpp (removed ConnectUM982Heading
  helper); gnhpr_parser is parser-only. main keeps GNSSData + HPR parser handles
  and fans out a single corrected true-heading value to both SK and N2K.
- Added src/n2k_senders.{h,cpp}: N2kSenders inits tNMEA2000_esp32 on the
  SH-ESP32 CAN pins (TX GPIO32 / RX GPIO34), NodeOnly, source address 25.
  Periodic senders: 127250 heading (100ms), 127257 attitude (100ms), 129025
  position (250ms), 129026 COG/SOG (250ms), 129029 GNSS data (1s). Inputs via
  ExpiringValue (2s) so stale data sends as N2k-NA, not frozen.
- Enabled GPVTG on the UM982 (COG in GNSSData comes from VTG, not RMC).
- Verified on device: clean boot, no crash. TWAI driver starts. With NO bus
  attached it enters expected bus-error recovery loop ("CANSendFrame - not
  open" spam) because CAN needs another node to ACK. GNSS/HPR path unaffected.
- Flash now 89.5% (1.76MB / 1.97MB app partition on min_spiffs). Headroom
  tightening on this 4MB board; watch it.
- NOT YET VERIFIED: actual PGN transmission — needs the N2K backbone + analyzer
  or plotter. Heading/attitude values also still need an outdoor fix.

### make_shared refactor (2026-06-03)
- Replaced all `new` in src/ with std::make_shared (per request).
- Lifetime model: connect_to(shared_ptr) makes the producer own the consumer,
  so chain objects (transforms, SK outputs) stay alive via their upstream
  producer. Root objects (gnss_data, hpr parser, n2k) are held by RAW pointers
  elsewhere (NMEA0183Parser registers sentence parsers raw; ConnectGNSS
  references GNSSData raw; N2kSenders onRepeat captures this), so they are kept
  in program-lifetime global shared_ptrs in main.cpp. A local shared_ptr there
  would dangle when setup() returns.
- SKOutputFloat (SKOutputNumeric) has no shared_ptr<SKMetadata> ctor, only the
  raw one (takes ownership). Used the (path, config, units) ctor with "rad"
  instead — clears the metadata warning without `new` in our code.
- Verified on device: clean boot, no crash, HPR still parsed (parser alive).
  Flash 89.8%.

### No-bus reboot + CAN spam fixed in the driver (2026-06-03)
- Symptom: with no N2K bus, huge "CANSendFrame/CANGetFrame - not open" spam and
  a reboot after a while.
- Root cause (driver, mairas/NMEA2000_twai): no bus -> bus-off -> errorMonitorTask
  reinits TWAI every ~2s. (1) per-call "not open" + per-cycle reinit logs flooded
  the log; (2) the monitor task ran twai_driver_uninstall_v2() while the event
  loop was inside twai_transmit/receive on the same handle (is_open_ checked
  without a lock, cleared only after uninstall) -> use-after-free -> Guru
  Meditation panic (rst:0xc SW_CPU_RESET, ~1/min).
- Fix (fork branch fix/busoff-teardown-race, commit 03aa7d4): mutex guarding the
  handle + is_open_ in CANSendFrame/CANGetFrame/CAN_init/CAN_deinit; clear
  is_open_ before teardown; report "not open"/"bus-off" once at ERROR (re-armed
  only on a received frame); silence per-cycle reinit logs. Reinit cadence
  unchanged (every 2s) per decision.
- Project pinned to the local fork via symlink in platformio.ini (temporary,
  until the fork branch is merged; then restore a repo URL and delete the
  dangling NMEA2000_twai dir at the workspace root).
- Verified on bench (no bus, 150s): 0 panics (was ~1/min); CAN logs down from
  thousands to ~5 (1 not-open + 1 startup init + 3x the pre-existing 1/min
  bus-off heartbeat). Device runs as a Signal-K-only compass with no N2K.
- BONUS: real heading observed this session -- $GNHPR quality 5 (RTK float),
  heading ~309 deg, 15 sats. The HPR parser works end-to-end on live data.

### Web UI config exposure (2026-06-03)
- Symptom: config options (heading offset) not showing in the web UI.
- Cause: in SensESP 3.x a config_path only makes an object persistable; the web
  UI lists it only if registered via ConfigItem(obj). The AngleCorrection was
  created inline and never registered.
- Fix: ConfigItem(heading_true) with title/description/sort order. Verified at
  boot: "Registering ConfigItemT with path /Heading/Offset".
- Note: ConfigItem requires a ConfigSchema(T&) overload. AngleCorrection has one;
  SKOutput<float/String/AttitudeVector> do NOT, so SK output paths can't be
  ConfigItem'd without adding a schema (library territory). N2K source address
  is a constexpr; could be exposed via a NumberConfig if wanted later.
- Exposed the N2K source address via a NumberConfig (NumberConfig has a
  ConfigSchema), requires_restart=true (applied at NMEA2000 init). Verified:
  "Registering ConfigItemT with path /NMEA 2000/Source Address".
- SK output path config is a real SensESP gap: SKOutput::to_json/from_json
  fully support reconfiguring sk_path (was UI-editable in 2.x), but 3.x ships
  no ConfigSchema(SKOutput<T>&), so ConfigItem can't render it. A small
  templated ConfigSchema overload in SignalK/SensESP would restore it.

### Still TODO
- Implement HPR parser ($GNHPR -> RTKData -> SK headingTrue/attitude) and
  N2K senders (Phase: implement). HPR field order confirmed from live data and
  manual: utc, heading, pitch, roll, QF, sats, ... (QF 4=fix,5=float,1=single).
- Repoint SK server to halos.hurma; outdoor fix test for real heading.
- Remove TEMP raw-line logger.
- Review before finalizing.
