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

### Still TODO
- Implement HPR parser ($GNHPR -> RTKData -> SK headingTrue/attitude) and
  N2K senders (Phase: implement). HPR field order confirmed from live data and
  manual: utc, heading, pitch, roll, QF, sats, ... (QF 4=fix,5=float,1=single).
- Repoint SK server to halos.hurma; outdoor fix test for real heading.
- Remove TEMP raw-line logger.
- Review before finalizing.
