# Outdoor field test

The firmware flashed and runs, but heading needs satellites: indoors the UM982
reports quality `0` and emits no heading. This test confirms real heading once
the antennas see open sky.

## Setup

1. **Antennas** — connect both to the UM982 (ANT1 and ANT2). Mount them:
   - With a **clear view of the sky** for both (no overhang, away from walls).
   - On a **rigid baseline** at a fixed separation. Longer is better: heading
     accuracy is roughly **0.1° per metre** of antenna separation.
   - Roughly **fore-and-aft** if you want the reported heading to match the
     boat's heading. Any constant mounting rotation is removed later with the
     offset (see step 5).
2. **Power + USB** — power the SH-ESP32 and connect it to the laptop over USB.
3. **Watch the serial output** from the project directory:

   ```
   python3 serial_monitor.py /dev/cu.usbserial-2010 -t 120
   ```

## What to watch for

The raw `$GNHPR` lines are logged directly. Field layout:

```
$GNHPR, utc, heading, pitch, roll, quality, satellites, ...
```

- **quality**: `0` = no fix → climbs as satellites lock. Heading is published
  only at **`4` (RTK fixed)** or **`5` (RTK float)**.
- **heading**: degrees, `0–360`. Stays `000.0000` until quality reaches 4/5,
  then shows the live baseline bearing.

Success = quality reaches **4**, and `heading` tracks reality when you rotate
the baseline. First fix usually takes seconds to a couple of minutes with good
sky.

## Sanity checks

4. Point the antenna baseline at a known bearing (e.g. along a wall you know the
   compass direction of) and confirm the `heading` value is close.
5. **Heading offset**: if the reading is consistently rotated (antennas not
   aligned fore-aft), open the SensESP web UI and set `/Heading/Offset` to cancel
   the difference. This corrects `navigation.headingTrue` and N2K PGN 127250.

## Signal K (optional, needs network)

The device currently has a **stale Signal K server** set to `oppi4.hal:3000`
(won't resolve). To see data on `halos.hurma`:

- Connect to the device's WiFi AP on first boot (or its web UI on the boat
  network) and set the Signal K server to `halos.hurma`.
- Then check `navigation.headingTrue`, `navigation.attitude`, and
  `navigation.gnss.headingQuality` on the dashboard.

If the boat network isn't reachable in the yard, the **serial output above is
the primary check** — it needs no network.

## NMEA 2000 (only if a bus is attached)

Without a backbone the CAN driver logs `CANSendFrame - not open` and loops on
bus errors — expected, harmless. On the backbone it transmits PGNs 127250,
127257, 129025, 129026, 129029. Verify on a plotter or N2K analyzer.
