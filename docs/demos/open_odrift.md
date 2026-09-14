# open_odrift — the empty hall where drift stays a number

`open` is 30 × 20 m of floor with a border wall and nothing else: no obstacle, no floor marking, no goal,
two spawns on the centre line. A wrong wheel constant hides behind a collision everywhere else; here it
stays a number.

## Start

Hold `Up` and watch `o`: the ghost walks east while the dot stays where the robot really is.

```bash
ros2 launch mecanum_lab demo_open_odrift.launch.py
```

Record it instead: 60 s, reference solution, truth on the bus so the log holds both:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_open_odrift.json \
    controller:=student/solution.py seconds:=60 truth:=true
```

Or type the wrong number yourself, which is what `--set` is for:
`./lab sim --world open --set odom.geometry.wheel_radius_scale=1.05`. Without ROS 2: `./lab sim --config
config/demo_open_odrift.json`, same arguments on `./lab run`.

## What it changes

| key | value | why it is there |
|---|---|---|
| `world` | `open` | the hall with nothing in it |
| `gps.gap` | `[0.0, 1e9]` | the outage window of `docs/CONTRACT.md` §6.4, opened to the length of the exercise: with fixes on screen the ghost is one candidate among three, and drift is the subject |
| `odom.geometry.wheel_radius_scale` | `1.05` | one wrong wheel constant, 5 % |

No slip, no jitter, no dropout, no thermal drift — the 5 % in the file is the 5 % on screen.
`view.layers` asks for `ghost` and `trails`: the ghost walking away is the measurement.

## In the window

* `o` — the ghost ahead on the line of travel; `odom off` in the readout names the gap in metres.
* `t` — its trail, dashed; the solid line in the same colour is the truth.
* the dot, where the robot really is, with no fix near it.

## Measured

20.00 m driven straight east at 0.5 m/s from the spawn, seed 1, 40.1 s of simulation time:

| `odom.geometry.wheel_radius_scale` | the odometry reports | ghost away from the robot | wall contacts |
|---|---|---|---|
| `1.0` (default) | 20.01 m | 0.01 m | 0 |
| `1.05` (this demo) | 21.01 m | **1.00 m** | 0 |

Halfway, after 12 m, the readout reads `odom off 0.60 m`: 5 cm per metre, all the way. Furnished,
the same error ends in [demos/odom_error.md](odom_error.md) — 26 wall contacts for T3.

## The exercise

Drive the 20 m, read the drift, and decide before looking whether it is the 5 % you asked for. Which
graded task would have caught it? `quadrat` (T2) drives on odometry and corrects itself by closing the
square; `korridor` (T3) does not. Every Experiment 2 filter that fuses odometry inherits this bias —
`kf_fusion` measures it during its GPS outage.

Model: `docs/CONTRACT.md` §6.4. Hall and passage widths of all six arenas: [docs/worlds.md](../worlds.md).

Also: [docs/demos.md](../demos.md), [demos/odom_error.md](odom_error.md),
[demos/poi_exploration.md](poi_exploration.md) (the same hall with one thing in it to find).
