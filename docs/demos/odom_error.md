# odom_error — the wheel constants the odometry believes are wrong

`odom.geometry` lets the odometry integrator assume a geometry of its own while the chassis keeps the
correct one (`sensors.odom_geometry`). This is the commonest real fault: a wheel radius that is not what
the drawing says. With the true geometry handed to the integrator, which is how it used to be, the fault
could not be expressed at all.

## Start

Drive a straight line and watch `o`: the ghost pulls ahead by 5 cm per metre while the truth stays on its
line. This file runs in `production`, between the tables the graded tasks drive in:

```bash
ros2 launch mecanum_lab demo_odom_error.launch.py
```

A lane with nothing in it is one argument away: `world:=track` on the command above. The same error on the
reference solution, with no keyboard in the way:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_odom_error.json \
    controller:=student/solution.py seconds:=30
```

And what it does to the graded tasks:

```bash
./lab grade --config config/demo_odom_error.json --task alle --controller student/solution.py
```

Without ROS 2 these are `./lab sim --config config/demo_odom_error.json`, with `--world track` or
`--controller student/solution.py --seconds 30` on the end.

## What it changes

| key | value | what it does |
|---|---|---|
| `wheel_radius_scale` | `1.05` | every wheel speed converts to 5 % more distance, so the path scales — and so does the yaw rate, because the same radius sits in the turn equation |
| `lever_scale` | `0.97` | the assumed lever arm `a = lx + ly` is 3 % short, so the turn rate comes out 3 % too big |

Three more ways to be wrong, all three left at their default here and explained in
`docs/CONTRACT.md` §6.4: `wheel_base_scale` mis-measures the vehicle length, so the axes mix in the wrong
ratio; `scale_xy` is a wrong radius with an honest gyro, which turns a commanded circle into a spiral;
`bias_xy` offsets the reported pose, which is a shifted world rather than drift. Default `odom.geometry`
is `{}`, so nothing graded moved when this was introduced. Radius 1.05 with lever 0.97 is what two worn
tyres and a reprinted hub add up to.

`view.layers` asks for `ghost` and `trails`: wrong wheel constants are only visible as the distance
between the robot and what its odometry believes.

## In the window

* `o` — the odometry ghost, ahead along the direction of travel, 5 cm per metre.
* `t` — the trail of that belief: the **dashed** line in the robot's colour, while the solid line of the
  same colour is the path it really drove. Every metre between the two is 5 cm of wrong wheel.
* the `odom off` segment of the readout line, in metres.

## Measured

Reference solution, seed 1:

* after 12 m of straight lane the ghost is **0.61 m** ahead of the robot, with the truth still on its line;
* one commanded revolution ends **28°** rotated — the radius alone reports 12.6 m for 12 m driven, the
  lever alone turns 11° over 360°, the two together add up to 8.25 %;
* the graded run drops to **70/100**: T1 ok, T2 ok (closure 0.18 m on a 4 m square — that controller
  corrects its own drift), T3 **fails with 26 wall contacts**, T4 ok at 0.17 m.

T3 is the point. It navigates between the tables by odometry, and a robot that believes it drove 8 %
further than it did arrives in a wall. T4 stays green because the GPS approach does not trust wheel
odometry.

## The exercise

Model error is not a noise knob: it scales with the distance driven, so every measurement taken over
odometry — T2's closure, T3's target, the odometry feed of every filter in Experiment 2 — shifts with it.
Drive the lane with `o` and `t` on and measure the gap against 5 % of the distance you drove. Then say
which graded task would have caught it, and which one would have driven into the furniture.

Model: `docs/CONTRACT.md` §6.4. Tasks: `docs/CONTRACT.md` §8 and `config/tasks.json`.

Also: [docs/demos.md](../demos.md), [demos/open_odrift.md](open_odrift.md) (the same 5 % where it cannot
end in a collision), [demos/gps_shadow.md](gps_shadow.md) (the sensor side of the same disagreement).
