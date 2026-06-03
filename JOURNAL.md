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

### Still TODO
- Implement HPR parser and N2K senders (Phase: implement).
- Build/flash/monitor verification (build-flash).
- Review before finalizing.
