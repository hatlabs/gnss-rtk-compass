# GNSS RTK Compass

A dual-antenna GNSS satellite compass for boats, built on the
[SensESP](https://github.com/SignalK/SensESP) framework. A Unicore UM982
module computes true heading from the baseline vector between its two
antennas, and an SH-ESP32 publishes heading, attitude, position, speed,
course, and fix quality to both Signal K (over WiFi) and NMEA 2000.

Because heading comes from satellite geometry, there is no magnetic
deviation, no calibration swing, and no drift.

## How it works

The UM982 runs in dual-antenna heading mode: it solves the relative vector
between its master (ANT1) and slave (ANT2) antennas and reports the
baseline's bearing as true heading, plus pitch along the baseline. This is a
"moving base" arrangement — only the relative geometry of the two antennas
is solved, so no external RTCM/NTRIP correction stream is needed.

What to expect:

- **Heading** is high-accuracy (≈0.1° per metre of antenna baseline) once
  the baseline solution reaches RTK-fixed quality.
- **Absolute position** is ordinary single-point GNSS (~1.5 m), not
  cm-level — there is no base station feeding corrections.
- The reported RTK fix status (fixed / float / single) describes the
  quality of the heading solution, not positional RTK.

The orientation of the antenna axis doesn't matter — the offset between
the baseline and the vessel heading is calibrated out with a configurable
heading offset. What does matter is separation: the further apart the
antennas, the more accurate the heading. The antennas are usually sensitive
enough to work well even inside a fiberglass boat.

## Hardware

### Parts

| Part | Source |
|------|--------|
| Mozihao UM982 GNSS module | [AliExpress](https://www.aliexpress.com/item/1005011761606321.html) |
| 2× active GNSS antenna with SMA male connector | generic |
| SH-ESP32 marine ESP32 development board | [Hat Labs shop](https://shop.hatlabs.fi/products/sh-esp32) |
| SH-ESP32 waterproof enclosure (100 × 68 × 50 mm) | [Hat Labs shop](https://shop.hatlabs.fi/products/sh-esp32-enclosure) |
| NMEA 2000 panel pigtail connector (male) | [Hat Labs shop](https://shop.hatlabs.fi/products/nmea-2000-panel-pigtail-connector-male) |

### Assembly

The build is minimal: the UM982 module is attached on top of the SH-ESP32
with double-sided tape, and Port 3 of the module is wired to the SH-ESP32
GPIO header with the wire supplied with the module. The NMEA 2000 panel
pigtail connector is mounted through the enclosure wall and connects the
SH-ESP32 to the bus, which both powers the device and carries the data
output. The whole stack goes into the 100 × 68 × 50 mm plastic enclosure
for waterproofing.

### Wiring

UM982 Port 3 to the SH-ESP32 GPIO header (pin numbers are ESP32 GPIOs; see
`src/main.cpp`):

| ESP32 GPIO | Function | UM982 Port 3 |
|-----------|----------|--------------|
| 21 | UART RX | TXD |
| 18 | UART TX | RXD |
| 26 | Reset (output, active low) | RST |
| 27 | PPS (input) | PPS |
| 3.3 V | Power | VCC |
| GND | Ground | GND |

The UM982 logic level is 3.3 V, directly compatible with the ESP32. The
CAN/NMEA 2000 interface uses the SH-ESP32 board defaults (GPIO 32/34).

## Software

The firmware is a [SensESP](https://github.com/SignalK/SensESP) application.
On boot it configures the UM982 over UART — heading mode, antenna baseline
length, heading offset, anti-jamming, anti-spoofing, and heading smoothing —
and confirms each command with the module's acknowledgement before moving to
the next. Data outputs are wired up only after every setting is confirmed,
so nothing is published from an unconfigured module. Unacknowledged commands
are retried, so a slow-booting module self-heals.

All module settings are adjustable at runtime in the SensESP web UI, along
with WiFi credentials, the Signal K server connection, and the NMEA 2000
source address.

### Data outputs

Heading is published at 10 Hz; position, speed, course, and satellite data
at 1 Hz.

Signal K paths:

| Data | Path |
|------|------|
| True heading | `navigation.headingTrue` |
| Attitude (yaw/pitch/roll) | `navigation.attitude` |
| Heading solution quality | `navigation.gnss.headingQuality` |
| Position | `navigation.position` |
| Speed over ground | `navigation.speedOverGround` |
| Course over ground | `navigation.courseOverGroundTrue` |
| Fix quality, satellites, HDOP | `navigation.gnss.*` |

NMEA 2000 PGNs:

| Data | PGN |
|------|-----|
| Vessel heading (true) | 127250 |
| Attitude | 127257 |
| Position, rapid update | 129025 |
| COG & SOG, rapid update | 129026 |
| GNSS position data | 129029 |
| GNSS DOPs | 129539 |
| GNSS satellites in view | 129540 |

## Building and flashing

This is a [PlatformIO](https://platformio.org/) project:

```bash
# Build
pio run

# Upload over USB
pio run -t upload

# Monitor serial output
pio device monitor
```

After the first flash, the device starts a WiFi access point named
`gnss-rtk-compass`. Connect to it and use the web configuration UI to set
your WiFi network and Signal K server. Subsequent updates can be done
over the air.

## License

Apache 2.0 — see [LICENSE](LICENSE).
