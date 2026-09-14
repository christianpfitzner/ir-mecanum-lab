# The six demos, the one drift that needs no file, and how fast a run goes

Each demo changes one block of the config and leaves the graded defaults alone. All six ship as files under
`config/`, because the graded tasks are calibrated on the plain sensor settings.

| demo | what it changes | page |
|---|---|---|
| `gps_shadow` | `gps.zones`: two shadowed rectangles and a blackout | [demos/gps_shadow.md](demos/gps_shadow.md) |
| `odom_error` | `odom.geometry`: odometry believes radius ×1.05, lever ×0.97 | [demos/odom_error.md](demos/odom_error.md) |
| `open_odrift` | the same wrong radius in the empty hall, GPS off | [demos/open_odrift.md](demos/open_odrift.md) |
| `sensor_reality` | latency, dropout, staleness, chip temperature | [demos/sensor_reality.md](demos/sensor_reality.md) |
| `wifi` | the radio link and its access point | [demos/wifi.md](demos/wifi.md) |
| `poi_exploration` | a radiation source somewhere in the hall | [demos/poi_exploration.md](demos/poi_exploration.md) |

## Wheel slip: odometry you can watch lying

Press the robot into a wall. The body stands still, the wheels keep the speed the motor demands, and
`sensors.py` integrates what the wheels report — never the truth. `robot.slip` scales the effect: `1` is
full slip and the default, `0` is ideal static friction with honest odometry.

Measured in `arena`, 4 s of full throttle east into the wall, seed 5:

| `robot.slip` | body really moved | odometry counted | phantom distance |
|---|---|---|---|
| `1` (default) | 0.69 m | 1.55 m | **+0.86 m** |
| `0` | 0.69 m | 0.69 m | −0.003 m |

By hand:

```bash
ros2 launch mecanum_lab lab.launch.py world:=arena
```

then `Up` into the east wall, and watch the `odom x` in the readout climb while the dot stays put. Want it
on the counter alone — no `/gps`, no truth, no `distance` field? `student/poi_seek_example.py`, on
[demos/poi_exploration.md](demos/poi_exploration.md).

## How fast a run goes

`./lab grade` runs in real time — that is what the lab course does. Two knobs change the pace and neither
changes the measurement. `--speed 4` takes four simulation seconds per wall second. `--fixed-step` steps
exactly 1/`rate` per round and never sleeps; that also switches the sleeps of the in-process bus off, so a
student node can keep up. Both keep the fixed physics step and the seed, and a run that cannot keep up says
so once, with the seconds it lost.

```bash
./lab grade --task alle --controller student/solution.py --speed 4
```

104 s of simulated drive in 26 s of wall. For a CI box without a window:

```bash
./lab grade --task alle --controller student/solution.py --fixed-step
```

The same report in 5 s, about 36× realtime. Without pygame at all, for a loop over many seeds:

```bash
python3 tools/fastgrade.py --task kf_alle --speed 8
```

Experiment 2 has to stay near real speed: `rate_hz` is the node's message rate per **sim** second, see
`docs/CONTRACT.md` §9.1 and `docs/CONTRACT-KF.md` §5.1.

## How the two figures are built

A deterministic run makes a screenshot reproducible, so both figures are build products rather than
photographs of a monitor. `--frame-max N` stops after N drawn frames, `--screenshot FILE.png` saves the last:

```bash
./lab sim --world production --config config/demo_gps_shadow.json --robots muster \
          --headless --fixed-step --frame-max 30 --screenshot docs/img/readout-gps-shadow.png
```

```bash
./lab sim --world production --config config/demo_wifi.json --robots muster \
          --headless --fixed-step --frame-max 30 --screenshot docs/img/readout-radio.png
```

`tests/test_readout_pictures_w7.py` reads these two commands out of this file and runs them headless on the
dummy video driver. Below the two text rows it compares committed pixels with today's drawing byte for byte
— the fps counter in row 1 is the one number a headless run cannot promise — and it insists a robot is
drawn in both. `tools/worldpic.py` draws the arena figures the same way.

![Frame 30 of `config/demo_gps_shadow.json` on `production`, 0.60 s of simulation time: the robot at its
spawn pose, the readout line above the map. Its odometer says `x=+2.25 y=+2.75`, the GPS fix of the same
instant `x=+2.29 y=+2.76` — five centimetres of error at a robot that has not moved, `q2 8 sats`. The
hatched rectangles are the GPS shadow zones, each labelled with what it does to a fix; the robot starts
outside all of them.](docs/img/readout-gps-shadow.png)

![Frame 30 of `config/demo_wifi.json`, the same 0.60 s: the access point at the left wall, the robot at the
spawn pose with its link-quality bar above it, and `wifi q 1.00 -43.6` at the right end of the readout
line — the best the link gets in this hall.](docs/img/readout-radio.png)

Both frames show the spawn pose, because that is what a headless run reaches without anyone driving. A
robot *inside* the shadow or *under* the failsafe needs a controller and a few thousand frames, so the
in-flight numbers on the demo pages stay quoted transcripts of driven runs — `tests/test_wifi_w6.py`
asserts the positions those transcripts name. Both are 1120 × 700 px (`view.width` × `view.height` of
`config/default.json`) at the default `view.scale` of 54 px/m.
