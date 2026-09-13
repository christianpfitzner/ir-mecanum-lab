# gps_shadow — the fix is worth nothing where you are standing

`gps.zones` degrades the GPS **by place**, not by time: the first rectangle that contains the robot
multiplies its σ, adds an offset, or takes the sky away entirely. The default is an empty list, so this
only happens in a run that asks for it.

```bash
./lab sim --world production --config config/demo_gps_shadow.json      # drive into it yourself
./lab run --world production --config config/demo_gps_shadow.json \
          --robot muster --controller student/solution.py --seconds 30  # watch it from outside
ros2 launch mecanum_lab demo_gps_shadow.launch.py                      # same through ROS, plus RViz
```

The window starts at the spawn pose; `--world arena` works too. To make it a measured exercise rather
than a drive, grade it: `./lab grade --world production --config config/demo_gps_shadow.json --task
kinematik --controller student/solution.py --log messung.csv`.

## What it changes

Everything is `gps.zones`, three rectangles in world metres (`sensors._zones`):

| zone | rectangle | what it does to a fix |
|---|---|---|
| under the high shelf | `8.0, 0.6, 19.4, 4.6` | `sigma_scale: 6`, `bias_xy: [0.8, -0.5]` — multi-path pushes the fix away from the reflector |
| multipath in the corner | `0.6, 7.0, 6.0, 11.4` | `sigma_scale: 3`, `bias_xy: [-0.4, 0.3]` |
| loading dock, no sky | `16.6, 8.6, 19.4, 11.4` | `block: true` — no fix at all, and a receiver with no fix sends nothing |

Beside them: `debug_truth: true`, which publishes `/<robot>/truth` so ghost, truth and fix can be
compared in one frame, and `view.layers` asking for `gps` and `ghost` — without the fix and the ghost the
shadow is a hatched rectangle and nothing else.

First zone that contains the robot wins. A broken entry is dropped rather than crashing the run.

## What you see

- `x` — the zones as hatched shadow, each labelled with what it does to a fix, the blackout in the
  error colour.
- `g` — the fix itself, drawn where the receiver says the robot is. Outside the zones it sits under the
  dot; in the shelf zone it scatters.
- `o` — the odometry ghost, which does not care about any of this and is the reference the shadow is
  measured against.
- the readout line: `q2 8 sats` on open floor, `q0 0 sats` in the dock.

Frame 30 of this config, rebuilt from the command in [docs/demos.md](../demos.md), is
`docs/img/readout-gps-shadow.png`.

## What it measured

400 fixes per spot, σ = 0.06 m being the lab default:

| spot | σ of a fix | median error | mean offset | fixes |
|---|---|---|---|---|
| open floor | 0.06 / 0.06 m | 0.07 m | (−0.01, 0.00) m | 400/400 |
| under the high shelf (σ×6, bias +0.8/−0.5) | 0.37 / 0.36 m | 1.03 m | (+0.83, −0.51) m | 400/400 |
| multipath in the corner (σ×3) | 0.18 / 0.18 m | 0.52 m | (−0.39, +0.30) m | 400/400 |
| loading dock (`"block": true`) | — | — | — | **no fix at all** |

## The exercise

One frame with `x`, `g` and `o` shows the three ways a position can be wrong: noisy, biased, or missing
— and how much of the distance between fix and ghost the odometry invented. The conclusion that matters
for a filter is the last row: **no fix and no message are different faults**, and quality 0 belongs to a
place, not to a packet. A filter that treats every message as equally good is tuned for a sensor that
does not exist.

Model and semantics: `docs/CONTRACT.md` §6.4. Graded tasks are calibrated on the plain GPS of each
experiment (`config/tasks.json`), which is why the file is a demo and why grading with it means
recalibrating the thresholds. The `T4` task `gps_anfahrt` is the graded one that drives by GPS at all.

See also: [docs/demos.md](../demos.md) (all six demos, the wheel-slip section, the screenshot recipe),
[demos/odom_error.md](odom_error.md) (the other way the position and the truth part company),
[docs/window.md](../window.md) (what hiding a layer does not change).
