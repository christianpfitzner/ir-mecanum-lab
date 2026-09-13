# sensor_reality — a sensor reports something besides its number

Six knobs, all of them off in `DEFAULT_CONFIG`. Off means not one random number is drawn that the sensors
of the graded tasks did not draw — measured message for message in `tests/test_sensor_reality.py`. This
file asks for all six at once, so one drive shows what a real sensor delivers on top of its measurement.

## Start

```bash
ros2 launch mecanum_lab demo_sensor_reality.launch.py
```

The readout from outside while the reference solution drives — `q2 8 sats`, `lost 7`, the chip temperature
and the empty beams come from the same numbers the topics carry:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_sensor_reality.json \
    controller:=student/solution.py seconds:=30
```

The same numbers in the CSV the report is written from:

```bash
./lab grade --config config/demo_sensor_reality.json --task kinematik \
            --controller student/solution.py --log messung.csv
```

Without ROS 2: `./lab sim --config config/demo_sensor_reality.json`.

## What it changes

The keys and their meaning are in `docs/CONTRACT.md` §6.4; the shipped values:

| key | value | note |
|---|---|---|
| `gps.dropout` | `0.15` | probability per message. The coin is tossed before anything is measured, so the holes belong to `--seed`, not to where the robot stood |
| `gps.latency` | `0.25` | seconds on the wire, ±50 % jittered. A fix in transit keeps the stamp it was measured with |
| `gps.sats`, `gps.sats_min` | `8`, `4` | anchors on open floor, and the floor below which there is no fix |
| `gps.zones` | two rectangles | q1 between the racks, q0 behind the loading-dock door |
| `imu.temp_*` | `14.0`, `25.0`, `0.0035`, `0.00012` | the chip warms 14 °C with a 25 s time constant; the bias walks 3.5 mm/s² and 0.12 mrad/s per °C |
| `lidar.reflectivity_min` | `0.25` | a beam echoes only above this `|cos|` of the incidence angle |
| `odom.jitter` | `0.35` | up to 35 % of a period late. Never early, never out of order |

`debug_truth: true` publishes `/<robot>/truth`, and `view.layers` opens `scan`, `gps` and `trails` — this
demo is about the measurements, so it shows the layers the clean view keeps empty.

## In the window

The readout line (`h`) carries all of it: `q2 8 sats`, `lost 7`, the chip temperature, the beams that came
back empty. Below it `sensor_state:` prints `q_gps`, `lost_gps`, `temp_imu` and `intensity_poi` as four
sparklines — what the instruments reported about themselves, each on its own scale.

None of that fits in the three standard messages: `/gps` is a `PoseStamped`, `/imu` an `Imu`, `/scan` a
`LaserScan`. `/<robot>/sensor/info` carries it — one JSON object per `gps.rate` with `quality`, `sats`,
`lost`, `latency_ms`, `temp` and `scan_gaps`, so `ros2 topic echo /alice/sensor/info` answers what the
readout line answers.

## Measured

One 25 s straight drive:

| knob | measured |
|---|---|
| `gps.sats`, `sats_min`, `zones` | q2 with 8 anchors on open floor, q1 with 3 between the racks, q0 in the dock (`"block": true`) — where the window reads q0 and `/gps` publishes nothing at all, from x = 16.6 m |
| `gps.dropout = 0.15` | 12 of 122 emissions gone, `lost 12` in the readout |
| `gps.latency = 0.25` | a fix is 0.45 s old when it arrives (0.25 s of wire plus the wait for the next slot), at most 1.0 s, and it is still the position it measured then |
| `imu.temp_*` | 24.00 → 29.52 °C, `az` bias +0.024 m/s², `gz` bias +0.00069 rad/s. Standing still stays cold and `az` stays +9.81 |
| `lidar.reflectivity_min = 0.25` | 0.5 m off a 30 m wall: seen 7.17 m down its length by default, 1.93 m here. 20 of 360 beams come back empty (`Scan.missing`) |
| `odom.jitter = 0.35` | stamps 13.3…26.8 ms instead of exactly 20.0 ms (σ 2.8 ms), values and count unchanged |

On a straight line the dropouts cost 5.56 m of GPS error against 0.18 m without them, and each one is
counted in the `lost` column of the log.

## The exercise

Drive the same line twice, with this file and without it, and compare `q_gps`, `lost_gps`, `temp_imu` and
`noecho_scan` against the error of your own filter. Two of the rows are the lesson:

* **No fix and no message are different faults.** Quality 0 belongs to a place: `GpsSensor.sky()` says what
  the sky at a position is worth, and a receiver in a place worth 0 returns `None` — there is no `/gps`
  message to attach a quality to. A dropped packet looks the same from outside and is counted in `lost`. A
  filter that treats every gap as a blackout cannot tell the dock from a radio failure.
* **Thermal drift is an offset, not noise**, so averaging does not remove it. It follows the load: it grows
  while the robot drives and walks back while it stands.

Model, the `sensor/info` payload, the determinism rule: `docs/CONTRACT.md` §6.4. The graded task above is
`kinematik` (T1).

Also: [docs/demos.md](../demos.md), [demos/gps_shadow.md](gps_shadow.md), [docs/window.md](../window.md).
