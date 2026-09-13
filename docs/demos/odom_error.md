# odom_error — the wheel constants the odometry believes are wrong

The odometry integrator used to be handed the *true* chassis geometry, so the commonest real fault —
a wheel radius that is not what the drawing says — could not be expressed at all. `odom.geometry` lets
the integrator build a geometry of its own while the chassis keeps the correct one
(`sensors.odom_geometry`).

Drive a straight line and watch `o`: the ghost pulls ahead by 5 cm per metre while the truth stays on
its line. The hall of this file is `production`, with the tables the graded tasks drive between:

```bash
ros2 launch mecanum_lab demo_odom_error.launch.py
```

For a lane with nothing in it, the track hall is one argument:

```bash
ros2 launch mecanum_lab demo_odom_error.launch.py world:=track
```

The same error on the ghost of the reference solution, with no keyboard in the way:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_odom_error.json \
    controller:=student/solution.py seconds:=30
```

What it does to the graded tasks is the interesting run, and grading is the one-process command — same
config file, one clock for grader, simulator and node:

```bash
./lab grade --config config/demo_odom_error.json --task alle --controller student/solution.py
```

Without ROS 2 the drives above are `./lab sim --config config/demo_odom_error.json`, with `--world track`
or `--controller student/solution.py --seconds 30` on the end.

## What it changes

`odom.geometry` — the odometry's own belief about the machine:

| key | value | what it does |
|---|---|---|
| `wheel_radius_scale` | `1.05` | every wheel speed converts to 5 % more distance, so the path scales — and so does the yaw rate, because the same radius sits in the turn equation |
| `lever_scale` | `0.97` | the assumed lever arm `a = lx + ly` is 3 % short, so the turn rate comes out 3 % too big |

Left at their defaults here, and worth knowing because they are the other three ways to be wrong:
`wheel_base_scale` mis-measures the vehicle length `lx`, so `vx`, `vy` and `omega` mix in the wrong ratio
while the speed stays about right; `scale_xy` makes the body velocity too fast or too slow while the yaw
rate stays right (wrong radius, honest gyro — a commanded circle becomes a spiral); `bias_xy` offsets the
reported pose, which is a shifted world rather than drift. Default `odom.geometry` is `{}`: correct
wheel constants, so nothing graded moved when this was introduced.

`view.layers` asks for `ghost` and `trails` — wrong wheel constants are only visible as the distance
between the robot and what its odometry believes. `1.05` radius with `0.97` lever is what two worn tyres
and a reprinted hub add up to.

## What you see

- `o` — the odometry ghost, pulling ahead along the direction of travel: 5 cm per metre.
- `t` — the trail the ghost leaves, which is the path the odometry thinks it drove.
- the readout line's `odom off` segment, the metres between the two.

## What it measured

Measured with the reference solution, seed 1:

- after 12 m of straight lane the ghost is **0.61 m** ahead of the robot, with the truth still on its line;
- one commanded revolution ends **28°** rotated — the radius alone reports 12.6 m for 12 m driven, the
  lever alone turns 11° over 360°, and the two together add up to 8.25 %;
- the graded run drops to **70/100**: T1 ok, T2 ok (closure 0.18 m on a 4 m square — that controller
  corrects its own drift), T3 **fails with 26 wall contacts**, T4 ok at 0.17 m.

T3 is the point. The LIDAR task navigates between the tables by odometry, and a robot that believes it
drove 8 % further than it did comes out in a wall. T4 stays green because the GPS approach does not
trust the wheel odometry.

## The exercise

Model error is not a noise knob: it scales with the distance driven, so every measurement taken over
odometry — T2's closure, T3's target, the odometry feed of every Kalman filter in Experiment 2 — shifts
with it. That is the argument for why this ships as a demo file and not as a default, and why grading
with it means recalibrating `config/tasks.json`.

Model: `docs/CONTRACT.md` §6.4. Tasks: `docs/CONTRACT.md` §8 and `config/tasks.json`.

See also: [docs/demos.md](../demos.md) (all six demos and the wheel-slip section),
[demos/open_odrift.md](open_odrift.md) (the same 5 % in a hall where it cannot end in a collision),
[demos/gps_shadow.md](gps_shadow.md) (the sensor side of the same disagreement).
