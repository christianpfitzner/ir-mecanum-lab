# open_odrift — the empty hall where drift stays a number

`open` is 30 × 20 m of floor with a border wall and nothing else: no obstacle, no floor marking, no goal,
two spawns on the centre line. Every other arena hides a wrong wheel constant behind a collision. Here it
stays what it is: a number.

## Start

Hold `Up` and watch `o`: the ghost walks east along the lane while the dot stays where the robot really is.

```bash
ros2 launch mecanum_lab demo_open_odrift.launch.py
```

The same drive recorded instead of watched — 60 s of the reference solution, with the truth on the bus so
the log holds both of them:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_open_odrift.json \
    controller:=student/solution.py seconds:=60 truth:=true
```

Without the file the same run is one wrong number on a command line, which is what `--set` is for:
`./lab sim --world open --set odom.geometry.wheel_radius_scale=1.05`. Without ROS 2 the two runs above are
`./lab sim --config config/demo_open_odrift.json` and the same arguments given to `./lab run`.

## What it changes

| key | value | why it is there |
|---|---|---|
| `world` | `open` | the hall with nothing in it |
| `gps.gap` | `[0.0, 1e9]` | the outage window of `docs/CONTRACT.md` §6.4, opened to the length of the exercise: with fixes on screen the ghost is one candidate among three, and drift is the subject |
| `odom.geometry.wheel_radius_scale` | `1.05` | one wrong wheel constant, 5 % |

Nothing else is turned on — no slip, no jitter, no dropout, no thermal drift — so the 5 % in the file is
the 5 % on the screen. The IMU keeps its default 100 Hz, which makes the same drive a second check as
well: the integral of `gz` against the yaw the odometry reports.

## In the window

* `o` — the ghost ahead on the line of travel, with `odom off` in the readout naming the gap in metres;
* `t` — the trail of that belief, dashed; the solid line in the same colour is the truth;
* the dot, which is where the robot really is, and no fix anywhere near it.

## Measured

20.00 m driven straight east at 0.5 m/s from the spawn, seed 1, 40.1 s of simulation time:

| `odom.geometry.wheel_radius_scale` | the odometry reports | ghost away from the robot | wall contacts |
|---|---|---|---|
| `1.0` (default) | 20.01 m | 0.01 m | 0 |
| `1.05` (this demo) | 21.01 m | **1.00 m** | 0 |

Halfway, after 12 m, the readout reads `odom off 0.60 m` — the same 5 cm per metre all the way. Both runs
end with 0 wall contacts, because there is nothing to collide with. In a furnished hall the same class of
error ends differently: [demos/odom_error.md](odom_error.md) measures 26 wall contacts for T3 with a 5 %
radius and a 3 % lever arm. `view.layers` in this file asks for `ghost` and `trails`: the ghost walking
away from the robot is the measurement.

## The exercise

Drive the 20 m, read the drift, and decide before looking whether the number is the 5 % you asked for.
Then say which graded task would have caught it: `quadrat` (T2) drives on odometry only and corrects
itself by closing the square, `korridor` (T3) does not. Every filter in Experiment 2 that fuses odometry
inherits this bias; `kf_fusion` measures it during its GPS outage.

Model: `docs/CONTRACT.md` §6.4. The hall and the passage widths of all five arenas:
[docs/worlds.md](../worlds.md).

Also: [docs/demos.md](../demos.md), [demos/odom_error.md](odom_error.md),
[demos/poi_exploration.md](poi_exploration.md) (the same hall with one thing in it to find).
