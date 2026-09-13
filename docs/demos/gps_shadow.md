# gps_shadow — the fix is worth less where you are standing

`gps.zones` degrades the GPS by place, not by time. The first rectangle that contains the robot
multiplies its σ, adds an offset, or takes the sky away. The default list is empty, so a graded run never
sees any of this.

## Start

```bash
ros2 launch mecanum_lab demo_gps_shadow.launch.py
```

The zones are rectangles in `production`'s metres, so this file brings that hall with it. From outside,
while the reference solution drives through them:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_gps_shadow.json \
    controller:=student/solution.py seconds:=30
```

As a measured exercise, with the series for the report:

```bash
./lab grade --config config/demo_gps_shadow.json --task kinematik \
            --controller student/solution.py --log messung.csv
```

Without ROS 2: `./lab sim --config config/demo_gps_shadow.json`. With `world:=arena` the zones sit in a
hall that has no tables to hide behind.

## What it changes

| zone | rectangle | what it does to a fix |
|---|---|---|
| under the high shelf | `8.0, 0.6, 19.4, 4.6` | `sigma_scale: 6`, `bias_xy: [0.8, -0.5]` — multipath pushes the fix away from the reflector |
| multipath in the corner | `0.6, 7.0, 6.0, 11.4` | `sigma_scale: 3`, `bias_xy: [-0.4, 0.3]` |
| loading dock, no sky | `16.6, 8.6, 19.4, 11.4` | `block: true` — no fix, and a receiver with no fix sends nothing |

The first zone that contains the robot wins; a broken entry is dropped rather than crashing the run.
`debug_truth: true` publishes `/<robot>/truth`, and `view.layers` asks for `gps` and `ghost`: without the
fix and the ghost a shadow is a hatched rectangle and nothing else.

## In the window

* `x` — the zones as hatched shadow, each labelled with what it does to a fix, blackout in the error colour.
* `g` — the fix, drawn where the receiver says the robot is. Outside the zones it sits under the dot; in
  the shelf zone it scatters.
* `o` — the odometry ghost, which ignores all of this and is what the shadow is measured against.
* the readout line: `q2 8 sats` on open floor, `q0 0 sats` in the dock.

Frame 30 of this config is `docs/img/readout-gps-shadow.png`; the command that rebuilds it is in
[docs/demos.md](../demos.md).

## Measured

400 fixes per spot, σ = 0.06 m being the lab default:

| spot | σ of a fix | median error | mean offset | fixes |
|---|---|---|---|---|
| open floor | 0.06 / 0.06 m | 0.07 m | (−0.01, 0.00) m | 400/400 |
| under the high shelf | 0.37 / 0.36 m | 1.03 m | (+0.83, −0.51) m | 400/400 |
| multipath in the corner | 0.18 / 0.18 m | 0.52 m | (−0.39, +0.30) m | 400/400 |
| loading dock | — | — | — | **no fix at all** |

## The exercise

One frame with `x`, `g` and `o` shows three ways a position can be wrong: noisy, biased, missing. It also
shows how much of the distance between fix and ghost the odometry invented. The last row is the part a
filter has to know: no fix and no message are different faults. Quality belongs to a place; a lost packet
is counted in `lost`. A filter that treats every message as equally good is tuned for a sensor that does
not exist.

Model: `docs/CONTRACT.md` §6.4. Graded tasks are calibrated on the plain GPS of each experiment, so
grading with this file means recalibrating the thresholds. The graded task that drives by GPS at all is T4.

Also: [docs/demos.md](../demos.md), [demos/odom_error.md](odom_error.md), [docs/window.md](../window.md).
