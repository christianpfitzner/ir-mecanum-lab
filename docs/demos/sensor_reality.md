# sensor_reality — a sensor reports something besides its number

Six knobs, all off in `DEFAULT_CONFIG` — off means not one random number the graded sensors did not draw,
measured message for message in `tests/test_sensor_reality.py`. This file asks for all six at once.

## Start

```bash
ros2 launch mecanum_lab demo_sensor_reality.launch.py
```

Watch the readout: `q2 8 sats`, `lost 7`, chip temperature and empty beams all come from the numbers the
topics carry.

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_sensor_reality.json \
    controller:=student/solution.py seconds:=30
```

Into the CSV the report is written from:

```bash
./lab grade --config config/demo_sensor_reality.json --task kinematik \
            --controller student/solution.py --log messung.csv
```

Without ROS 2: `./lab sim --config config/demo_sensor_reality.json`.

## What it changes

Keys and meanings: `docs/CONTRACT.md` §6.4. The shipped values:

| key | value | note |
|---|---|---|
| `gps.dropout` | `0.15` | probability per message, tossed before anything is measured — the holes belong to `--seed`, not to where the robot stood |
| `gps.latency` | `0.25` | seconds on the wire, ±50 % jittered; a fix in transit keeps its measured stamp |
| `gps.sats`, `gps.sats_min` | `8`, `4` | anchors on open floor, and the floor below which there is no fix |
| `gps.zones` | two rectangles | q1 between the racks, q0 behind the loading-dock door |
| `imu.temp_*` | `14.0`, `25.0`, `0.0035`, `0.00012` | warms 14 °C with a 25 s time constant; bias walks 3.5 mm/s² and 0.12 mrad/s per °C |
| `lidar.reflectivity_min` | `0.25` | a beam echoes only above this `|cos|` of the incidence angle |
| `odom.jitter` | `0.35` | up to 35 % of a period late; never early, never out of order |

`debug_truth: true` publishes `/<robot>/truth`; `view.layers` opens `scan`, `gps` and `trails`.

## In the window

The readout line (`h`) carries those four numbers, and `sensor_state:` below it prints `q_gps`,
`lost_gps`, `temp_imu` and `intensity_poi` as sparklines, each on its own scale. None of that fits in
`PoseStamped`, `Imu` and `LaserScan`, so `/<robot>/sensor/info` carries it: one JSON object per `gps.rate`
with `quality`, `sats`, `lost`, `latency_ms`, `temp` and `scan_gaps` (payload: `docs/CONTRACT.md` §6.4).

## Measured

One 25 s straight drive:

| knob | measured |
|---|---|
| `gps.sats`, `sats_min`, `zones` | q2 with 8 anchors on open floor, q1 with 3 between the racks, q0 in the dock (`"block": true`) — the window reads q0 while `/gps` publishes nothing, from x = 16.6 m |
| `gps.dropout = 0.15` | 12 of 122 emissions gone, `lost 12` in the readout |
| `gps.latency = 0.25` | a fix arrives 0.45 s old (0.25 s of wire plus the wait for the next slot), at most 1.0 s — still the position it measured then |
| `imu.temp_*` | 24.00 → 29.52 °C, `az` bias +0.024 m/s², `gz` bias +0.00069 rad/s; standing still stays cold, `az` stays +9.81 |
| `lidar.reflectivity_min = 0.25` | 0.5 m off a 30 m wall: seen 7.17 m down its length by default, 1.93 m here; 20 of 360 beams empty (`Scan.missing`) |
| `odom.jitter = 0.35` | stamps 13.3…26.8 ms instead of exactly 20.0 ms (σ 2.8 ms), values and count unchanged |

On a straight line the dropouts cost 5.56 m of GPS error against 0.18 m without them, each counted in
`lost`.

## The exercise

Drive one line twice, with this file and without, and compare `q_gps`, `lost_gps`, `temp_imu` and
`noecho_scan` against the error of your own filter. Two rows are the lesson:

* **No fix and no message are different faults.** Quality 0 belongs to a place (`GpsSensor.sky()`) and a
  receiver there returns `None` — no `/gps` message exists to attach a quality to. A dropped packet looks
  the same but is counted in `lost`, and a filter that calls every gap a blackout cannot tell the dock from
  a radio failure.
* **Thermal drift is an offset, not noise** — averaging does not remove it, and it follows the load.

The graded task above is `kinematik` (T1).

Also: [docs/demos.md](../demos.md), [demos/gps_shadow.md](gps_shadow.md), [docs/window.md](../window.md).
