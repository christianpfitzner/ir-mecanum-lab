# The six demo configurations, and how fast a run goes

The front page lists the demos; this page is what each one is for, what it changes in the message
stream, and what a run with it actually measured. Nothing here is graded: all six ship as separate
files under `config/` because the graded tasks are calibrated on the plain sensor settings of each
experiment, and a default that lies would move what every filter is graded against.

Each file changes one block of the config and leaves the graded defaults alone, so starting a demo is a
`--config` and not an edit — in the window of one process, or beside RViz through the launch file named in
the table. One page each, with the commands, the keys, what the window shows and what was measured:

| demo | what it changes | page |
|---|---|---|
| `gps_shadow` | `gps.zones`: two shadowed rectangles and a blackout; hatched shadow (`x`) | [demos/gps_shadow.md](demos/gps_shadow.md) |
| `odom_error` | `odom.geometry`: the odometry believes radius ×1.05, lever ×0.97 | [demos/odom_error.md](demos/odom_error.md) |
| `open_odrift` | the same wrong radius in the empty hall, GPS switched off for the run | [demos/open_odrift.md](demos/open_odrift.md) |
| `sensor_reality` | latency, dropout, staleness, chip temperature | [demos/sensor_reality.md](demos/sensor_reality.md) |
| `wifi` | the radio link and its access point | [demos/wifi.md](demos/wifi.md) |
| `poi_exploration` | a radiation source somewhere in the hall | [demos/poi_exploration.md](demos/poi_exploration.md) |

What is left on this page is the one drift that needs no config file at all, and the two knobs that
decide how fast any of these runs finish.

### Wheel slip: odometry you can watch lying

Press the robot against a wall and the odometry keeps counting metres that were never driven.
That is the drift of Experiment 1 in its purest form — no sensor noise needed. The model is
physical: against an obstacle the body stands still while the wheels keep the speed the motor
demands, and odometry integrates wheel speeds (`sensors.py` integrates what the wheels report,
never the truth). `robot.slip` scales it: `1` = full slip (default), `0` = ideal static friction
with honest odometry. Measured in `arena`, 4 s of full throttle east into the wall, seed 5:

| `robot.slip` | body really moved | odometry counted | phantom distance |
|---|---|---|---|
| `1` (default) | 0.69 m | 1.55 m | **+0.86 m** |
| `0` | 0.69 m | 0.69 m | −0.003 m |

By hand: `./lab sim --world arena`, then `Up` into the east wall — the `odom x` in the readout
climbs while the robot's dot stays where it is. To drive it yourself with nothing but the counter —
no `/gps`, no truth, no `distance` field — `student/poi_seek_example.py` is the worked example:

```bash
./lab run --world open --config config/demo_poi_exploration.json \
          --controller student/poi_seek_example.py --seconds 100 --log poi.csv
```

It climbs while the reading rises, arcs when it stops rising, and drives back to where it was loudest
when the field goes silent; the why of all three rules is in the file, and `intensity_poi`/`name_poi`
in `poi.csv` is what `tools/kfplot.py` plots afterwards. To record the IMU instead: `./lab grade --task
kinematik --controller student/solution.py --log messung.csv` writes `ax_imu, ay_imu, gz_imu` next to
the other columns (`python3 tools/kfplot.py messung.csv --list` shows all of them, and
`sensor_state:` prints `q_gps`, `lost_gps`, `temp_imu` and `intensity_poi` as four sparklines — what
the instruments reported about themselves, each on its own scale).

### Odometry with the wrong wheel radius: the model-error demo (not the default)

Slip is one way to make odometry lie; the commonest one is that the wheel constants are wrong —
worn tyres, a reprinted hub, the diameter used where the radius belongs. `odom.geometry` lets the
odometry integrator believe a geometry of its own (`wheel_radius_scale`, `lever_scale`,
`wheel_base_scale`, `scale_xy`, `bias_xy`; see `docs/CONTRACT.md` §6.4), while the chassis keeps
the true one. Default `{}` = correct wheel constants, so nothing graded moved.
`config/demo_odom_error.json` is the demo (radius 1.05, lever 0.97):

```bash
./lab sim --world track --config config/demo_odom_error.json      # drive straight, watch `o`
./lab grade --config config/demo_odom_error.json --task alle --controller student/solution.py
```

Measured with the reference solution: after 12 m of straight lane the ghost is **0.61 m** ahead of
the robot (5 % too many metres), one commanded revolution ends **28°** rotated, and the graded run
drops to 70/100 — T1 and T2 stay green (T2 corrects its own drift), but T3 navigates by odometry
between the tables and comes out in one: **26 wall contacts**. That is the argument for why this is
a demo file and not a default.

### What a sensor says besides its number: the reality demo (not the default)

Every knob here is off in `DEFAULT_CONFIG`, and off means that not one random number is drawn that
the sensors of the graded tasks did not draw — measured message for message, not assumed
(`tests/test_sensor_reality.py`). `config/demo_sensor_reality.json` asks for all of them at once,
so one drive shows what a real sensor delivers on top of its number:

```bash
./lab sim --world production --config config/demo_sensor_reality.json    # drive it yourself
./lab grade --world production --config config/demo_sensor_reality.json \
            --task kinematik --controller student/solution.py --log messung.csv
```

| knob in the demo | what it changes | measured on one 25 s straight drive |
|---|---|---|
| `gps.sats = 8`, `gps.sats_min = 4`, `gps.zones` | what the fix is worth **where the robot is standing** | q2 with 8 anchors on open floor, q1 with 3 between the racks, and in the dock (`"block": true`) the window reads q0 while `/gps` publishes nothing at all — from x = 16.6 m |
| `gps.dropout = 0.15` | probability per message that the transport loses it | 12 of 122 emissions gone, `lost 12` in the readout; the pattern belongs to `--seed`, not to where the robot stood |
| `gps.latency = 0.25` | seconds on the wire, ±50 % jittered | a fix is 0.45 s old when it arrives (0.25 s of wire + the wait for the next slot), at most 1.0 s, and it is still the position it measured then |
| `imu.temp_*` | the chip warms up and the bias walks with it | 24.00 → 29.52 °C, `az` bias +0.024 m/s², `gz` bias +0.00069 rad/s; standing still stays cold and `az` stays +9.81 |
| `lidar.reflectivity_min = 0.25` | a wall echoes only if the cosine of the incidence angle reaches the threshold | 0.5 m off a 30 m wall: seen 7.17 m down its length by default, 1.93 m here; 20 of 360 beams come back empty (`Scan.missing`) |
| `odom.jitter = 0.35` | an encoder report arrives when it arrives | stamps 13.3…26.8 ms instead of exactly 20.0 ms (σ 2.8 ms), values and message count unchanged |

Two of these are the exercise, not the decoration. **No fix and no message are different faults.**
Quality 0 belongs to a *place*, not to a message: `GpsSensor.sky()` says what the sky at a position is
worth, the window, the log and `/sensor/info` repeat that answer, and a receiver standing in a place
worth 0 sends **nothing** — `fix()` returns `None` and there is no `/gps` message to attach a quality
to. A dropped packet looks the same from outside and is counted separately, in `lost`. So a filter that
treats every message as equally good is a filter tuned for a sensor that does not exist, and one that
treats every gap as a blackout cannot tell the dock from a radio failure. With `q_gps`, `sats_gps`,
`lost_gps`, `temp_imu` and `noecho_scan` in the same CSV as the errors of the student's own filter, the
drive becomes an argument instead of a demo.

Over ROS 2 none of the three standard messages can carry any of it — `/gps` is a `PoseStamped`, `/imu`
an `Imu`, `/scan` a `LaserScan` — which is what `/<robot>/sensor/info` is for: one JSON object per
`gps.rate` with `quality`, `sats`, `lost`, `latency_ms`, `temp` and `scan_gaps`, so
`ros2 topic echo /alice/sensor/info` answers what the readout line answers (`docs/CONTRACT.md` §6.4).

---

### How fast a run goes: `--speed` and `--fixed-step`

`./lab grade` runs in real time, because that is what the lab course does. For supervisors who want
the same run again and again: `--speed 4` takes four simulation seconds per wall second,
`--fixed-step` steps exactly 1/`rate` per round and never sleeps (≈36× realtime, and it switches
the sleeps of the in-process bus off, so a student node can keep up). Both keep the fixed physics
step and the seed, and a run that cannot keep up says so once with the seconds it lost instead of
quietly dropping them. What each pace measures — including why experiment 2 must stay near real
speed (`rate_hz` is the node's message rate per **sim** second): `docs/CONTRACT.md` §9.1 and
`docs/CONTRACT-KF.md` §5.1.

```bash
./lab grade --task alle --controller student/solution.py --speed 4      # 104 s of sim in 26 s
./lab grade --task alle --controller student/solution.py --fixed-step   # the same in 5 s
python3 tools/fastgrade.py --task kf_alle --speed 8                     # without the window at all
```

The same determinism is what makes a **screenshot** reproducible, and these two pictures are built
that way rather than photographed off a monitor: `--frame-max N` stops after N drawn frames and
`--screenshot FILE.png` saves the last one through `pygame.image.save`, so a figure is a build product
of the build that checks it —

```bash
./lab sim --world production --config config/demo_gps_shadow.json --robots muster \
          --headless --fixed-step --frame-max 30 --screenshot docs/img/readout-gps-shadow.png
./lab sim --world production --config config/demo_wifi.json --robots muster \
          --headless --fixed-step --frame-max 30 --screenshot docs/img/readout-radio.png
```

Both lines run in the headless CI (dummy video driver). `tests/test_readout_pictures_w7.py` reads them
out of this file — a test with its own copy of a command proves nothing about the command in the
documentation — runs them, and compares the picture in the repository with what the command draws
today, byte for byte below the two text lines. It also insists that a robot is drawn in both, because a
frame of an empty hall once passed every size and colour check there was. `tools/worldpic.py` draws the
arena picture the same way, and a figure that cannot be regenerated by the command next to it is a
figure that is quietly lying.

![Frame 30 of `config/demo_gps_shadow.json` on `production`, 0.60 s of simulation time: the robot at its
spawn pose, and the readout line at the top of the window making the point of the demo — its own
odometer says `x=+2.25 y=+2.75` while the GPS fix for the same instant says `x=+2.29 y=+2.76`, five
centimetres of error at a robot that has not moved, with `q2 8 sats`. `kf -` is a dash because the
estimate is the student's node and this run is the simulator alone. The hatched rectangles are the GPS
shadow zones, each labelled with what it does to a fix; the robot starts outside all of them.](docs/img/readout-gps-shadow.png)

![Frame 30 of `config/demo_wifi.json`, the same 0.60 s: the access point at the left wall, the robot at
the spawn pose with the link-quality bar above it, and the `wifi q 1.00 -43.6` segment at the right end
of the readout line — the best the link gets in this hall, which is the other half of the story the
table below measures.](docs/img/readout-radio.png)

*What these two frames are and are not:* they are the readout line and the map at the spawn pose, which
is the state a headless build reaches without anyone driving. A picture of a robot **inside** the shadow
or **under** the failsafe needs a controller and a few thousand frames — `--frame-max` counts drawn
frames, and 30 of them is 0.60 s — so the in-flight numbers in this file stay quoted transcripts of
driven runs (`./lab run … --controller student/link_autonomy_example.py`, `./lab grade … --log`), and
`tests/test_wifi_w6.py` asserts the positions those transcripts name. What the two frames above prove
is the part a screenshot can prove: that the segments exist, that they are drawn from the same numbers
the topics carry, and that the picture is rebuildable.

*Both frames are 1120 × 700 pixels — `view.width` × `view.height` of `config/default.json`, which is
what the `--screenshot` line saves and what the test compares against — at the default `view.scale` of
54 px/m, the ruler the σ segment prints.* The size is a config value and not a constant: change the
window, and both the picture and the assertion change with it.
