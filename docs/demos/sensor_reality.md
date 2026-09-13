# sensor_reality — a sensor reports something besides its number

Six knobs, every one of them off in `DEFAULT_CONFIG`. Off means not one random number is drawn that the
sensors of the graded tasks did not draw — measured message for message, not assumed
(`tests/test_sensor_reality.py`). This file asks for all six at once, so one drive shows what a real
sensor delivers on top of its measurement.

```bash
./lab sim --world production --config config/demo_sensor_reality.json   # drive the arena yourself
./lab run --world production --config config/demo_sensor_reality.json \
          --robot muster --controller student/solution.py --seconds 30   # the readout from outside
ros2 launch mecanum_lab demo_sensor_reality.launch.py                    # same through ROS, plus RViz
```

To get the same numbers into the CSV the report is written from:

```bash
./lab grade --world production --config config/demo_sensor_reality.json \
            --task kinematik --controller student/solution.py --log messung.csv
```

## What it changes

| key | value | what it decides |
|---|---|---|
| `gps.dropout` | `0.15` | probability per message that the transport loses it. The coin is tossed before anything is measured, so the pattern of holes belongs to `--seed` and not to where the robot stood |
| `gps.latency` | `0.25` | seconds on the wire, ±50 % jittered. A fix that is still travelling keeps the stamp it was measured with, so a filter can see `now − fix.t` and predict over the gap |
| `gps.sats`, `gps.sats_min` | `8`, `4` | how many anchors the sky is worth on open floor, and the floor below which there is no fix |
| `gps.zones` | two rectangles | here for `quality` and `sats`: 8 anchors (q2) on open floor, 3 between them and the pallet rack (q1), none behind the loading-dock door (q0 — and a receiver with no fix sends nothing at all) |
| `imu.temp_motor`, `temp_tau`, `temp_walk`, `temp_walk_gyro` | `14.0`, `25.0`, `0.0035`, `0.00012` | the chip warms 14 °C towards the temperature the drive current asks for with a 25 s time constant, and the bias walks 3.5 mm/s² and 0.12 mrad/s per °C |
| `lidar.reflectivity_min` | `0.25` | a beam only echoes if the wall returns a quarter of what it got — `|cos|` of the incidence angle |
| `odom.jitter` | `0.35` | each report reaches the bus up to 35 % of a period late. Never early, never out of order, so only the stamps become uneven |

`debug_truth: true` publishes `/<robot>/truth`, and `view.layers` asks for `scan`, `gps` and `trails` —
this demo is about the measurements themselves, so it opens the layers the clean view keeps empty.

## What you see

The readout line (`h`) carries all of it: `q2 8 sats`, `lost 7`, the chip temperature, the count of beams
that came back with no echo. Below it the `sensor_state:` line prints `q_gps`, `lost_gps`, `temp_imu` and
`intensity_poi` as four sparklines — what the instruments reported about themselves, each on its own
scale. Over ROS the same facts are on `/<robot>/sensor/info`, one JSON object per `gps.rate` with
`quality`, `sats`, `lost`, `latency_ms`, `temp` and `scan_gaps`, because none of the three standard
messages can carry them: `/gps` is a `PoseStamped`, `/imu` an `Imu`, `/scan` a `LaserScan`.

## What it measured

One 25 s straight drive:

| knob | what it changes | measured |
|---|---|---|
| `gps.sats`, `gps.sats_min`, `gps.zones` | what the fix is worth **where the robot is standing** | q2 with 8 anchors on open floor, q1 with 3 between the racks, and in the dock (`"block": true`) the window reads q0 while `/gps` publishes nothing at all — from x = 16.6 m |
| `gps.dropout = 0.15` | probability per message that the transport loses it | 12 of 122 emissions gone, `lost 12` in the readout; the pattern belongs to `--seed`, not to where the robot stood |
| `gps.latency = 0.25` | seconds on the wire, ±50 % jittered | a fix is 0.45 s old when it arrives (0.25 s of wire + the wait for the next slot), at most 1.0 s, and it is still the position it measured then |
| `imu.temp_*` | the chip warms up and the bias walks with it | 24.00 → 29.52 °C, `az` bias +0.024 m/s², `gz` bias +0.00069 rad/s; standing still stays cold and `az` stays +9.81 |
| `lidar.reflectivity_min = 0.25` | a wall echoes only above the threshold | 0.5 m off a 30 m wall: seen 7.17 m down its length by default, 1.93 m here; 20 of 360 beams come back empty (`Scan.missing`) |
| `odom.jitter = 0.35` | an encoder report arrives when it arrives | stamps 13.3…26.8 ms instead of exactly 20.0 ms (σ 2.8 ms), values and message count unchanged |

## The exercise

Drive the same line twice, once with this file and once without, and compare `q_gps`, `lost_gps`,
`temp_imu` and `noecho_scan` against the error of your own filter. Two of the rows are the lesson rather
than the decoration:

- **No fix and no message are different faults.** Quality 0 belongs to a *place*: `GpsSensor.sky()` says
  what the sky at a position is worth, and a receiver standing in a place worth 0 returns `None` — there
  is no `/gps` message to attach a quality to. A dropped packet looks the same from outside and is
  counted separately, in `lost`. A filter that treats every gap as a blackout cannot tell the dock from
  a radio failure.
- **Thermal drift is an offset, not noise**, so averaging does not remove it: it follows the load, grows
  while the robot drives and walks back while it stands.

Model, the `sensor/info` payload and the determinism rule: `docs/CONTRACT.md` §6.4. The graded task
driven above is `kinematik` (T1).

See also: [docs/demos.md](../demos.md) (all six demos and how fast a run goes),
[demos/gps_shadow.md](gps_shadow.md) (the GPS zones on their own), [docs/window.md](../window.md)
(every segment of the readout line).
