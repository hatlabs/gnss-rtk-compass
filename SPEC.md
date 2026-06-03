# GNSS RTK Compass Specification

## Goal

Turn a UM982 dual-antenna GNSS module into a satellite compass for *Hurma*. The
device reads the UM982's serial output, derives **true heading** from the
relative vector between the two antennas (no magnetic deviation), and publishes
heading, position, speed/course, and fix-quality status to both Signal K and the
NMEA 2000 backbone.

## How it works (and what to expect)

The UM982 runs in **dual-antenna heading mode** (`MODE HEADING2`). It computes
the baseline vector between its master (ANT1) and slave (ANT2) antennas and
reports the baseline's bearing as true heading, plus pitch along the baseline.
This is the "moving base" arrangement: both antennas move with the vessel, and
only their *relative* geometry is solved, so **no external RTCM/NTRIP correction
stream is required**.

Consequences to keep expectations straight:

- **Heading** is high-accuracy (≈0.1° per metre of antenna baseline) once the
  baseline solution reaches RTK-fixed quality.
- **Absolute position** is ordinary single-point GNSS (~1.5 m), *not* cm-level —
  there is no external base station feeding corrections.
- **"RTK fix status"** here describes the quality of the heading/baseline
  solution (fixed / float / single), not positional RTK.

Antenna mounting matters: the ANT1→ANT2 baseline should run fore-and-aft along
the vessel centreline. Any mounting offset is removed with a configurable
heading-offset correction.

## Hardware

- **Board**: SH-ESP32 (ESP32-WROOM-32E, 4 MB flash)
- **Sensor**: Unicore UM982 dual-antenna GNSS/heading module, two antennas
- **Interface**: UART, 115200 baud (UM982 default)
- **Outputs**: Signal K over WiFi + NMEA 2000 on the existing backbone

## Connections

Pin labels below are from the **ESP32 side**.

| ESP32 GPIO | Function | Connects to UM982 | Notes |
|-----------|----------|-------------------|-------|
| 21 | UART RX | UM982 TXD | ESP32 receives NMEA/data from module |
| 18 | UART TX | UM982 RXD | ESP32 sends config commands to module |
| 26 | RST (output) | UM982 reset | Drive low to reset module on boot if needed |
| 27 | PPS (input) | UM982 PPS | 1 pulse/second timing signal (optional use) |
| 32 | CAN TX | — | NMEA 2000 (board default) |
| 34 | CAN RX | — | NMEA 2000 (board default, input only) |

All four UM982 GPIOs (18, 21, 26, 27) are free pins on the SH-ESP32 header and
do not collide with CAN (32/34), I2C (16/17), 1-Wire (4), or the opto I/O
(33/35). UM982 logic is 3.3 V — compatible with the ESP32 directly.

## UM982 configuration (sent over UART at startup)

The firmware configures the module on boot so the install is plug-and-play:

- `MODE HEADING2 FIXLENGTH` — dual-antenna heading, fixed baseline (antennas are
  rigidly mounted on the boat).
- Enable NMEA output sentences at suitable rates:
  - `$GNGGA`, `$GNRMC` — position, time, speed/course, fix quality, satellites.
  - `$GNHPR` — heading, pitch, roll, and solution-quality indicator (the
    dual-antenna attitude log).
  - optional `$GNGSA` / `$GNGSV` for satellite detail.
- Persist with `SAVECONFIG` (or re-send every boot — decided in planning).

Heading is targeted at roughly 10 Hz; position at ~1–5 Hz. Final rates settled
during implementation.

## Outputs

### Signal K (over WiFi to `halos.hurma`)

| Data | Signal K path |
|------|---------------|
| True heading | `navigation.headingTrue` |
| Position | `navigation.position` |
| Speed over ground | `navigation.speedOverGround` |
| Course over ground | `navigation.courseOverGroundTrue` |
| Fix / baseline quality | `navigation.gnss.methodQuality` (+ satellites, HDOP) |
| Pitch / roll | `navigation.attitude` |

### NMEA 2000

| Data | PGN |
|------|-----|
| Vessel heading (true) | 127250 |
| Attitude (yaw/pitch/roll) | 127257 |
| Position, rapid update | 129025 |
| COG & SOG, rapid update | 129026 |
| GNSS position data (incl. fix quality, sats) | 129029 |

Device uses a unique N2K source address to avoid bus conflicts.

## Expected behavior

1. On power-up the ESP32 brings up WiFi (web config UI for credentials), then
   configures the UM982 over UART.
2. Once the module acquires satellites and a baseline solution, true heading
   appears on both Signal K and N2K. Heading accuracy improves to full spec when
   the baseline solution reaches RTK-fixed.
3. Position, SOG, and COG stream continuously at single-point accuracy.
4. Fix-quality status reports single / float / fixed so a degraded heading
   solution is visible rather than silently wrong.

## Dependencies (PlatformIO `lib_deps`)

- `SignalK/SensESP @ ^3.3.0`
- `SensESP/NMEA0183` — GNSS/RTK sentence parsing and Signal K wiring
- `ttlappalainen/NMEA2000-library`
- `NMEA2000_twai` — ESP32 CAN driver

## Open implementation notes (resolved during planning, not requirements)

- The NMEA0183 library has the data structures (`RTKData.attitude`,
  `baseline_*`, `rtk_quality`) and wiring helpers (`ConnectGNSS`,
  `ConnectQuectelRTK`) but **no `$GNHPR` parser**. Decision: implement a
  Unicore HPR sentence parser **project-locally** as a `SentenceParser`
  subclass feeding `RTKData` (heading, pitch, roll, quality). No library
  release needed.
- N2K sender code in the reference project `sensesp-rtk` predates SensESP 3.x and
  needs porting to the current API.
- Reference project `/Users/mairas/projects/mairas/sensesp-rtk` is read-only
  inspiration (UART-in, TCP streaming, N2K scaffolding) — not the basis repo.
