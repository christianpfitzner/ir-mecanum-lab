# open_odrift — the empty hall where drift stays a number

`open` is 30 × 20 m of floor with a border wall and nothing else: no obstacle, no floor marking, no
goal, two spawns on the centre line. Every other arena hides a wrong wheel constant behind a collision.
Here the same wrong constant stays what it is: a number.

Hold `Up` and watch `o`: the ghost walks east along the lane while the dot stays where the robot really
is. One command, and the hall of this file is the empty one:

```bash
ros2 launch mecanum_lab demo_open_odrift.launch.py
```

The same drive recorded instead of watched — 60 s of the reference solution, with the truth on the bus so
the log holds both of them:

```bash
ros2 launch mecanum_lab lab.launch.py config:=config/demo_open_odrift.json \
    controller:=student/solution.py seconds:=60 truth:=true
```

Without the file, the same run is one wrong number on a command line, which is what `--set` is for:
`./lab sim --world open --set odom.geometry.wheel_radius_scale=1.05`. Without ROS 2 the two runs above are
`./lab sim --config config/demo_open_odrift.json` and the same arguments given to `./lab run`.

## What it changes

| key | value | why it is there |
|---|---|---|
| `world` | `open` | the hall with nothing in it |
| `gps.gap` | `[0.0, 1e9]` | the outage window of `docs/CONTRACT.md` §6.4, opened to the length of the exercise: with fixes on screen the ghost is only one candidate among three, and drift is the subject |
| `odom.geometry.wheel_radius_scale` | `1.05` | one wrong wheel constant, 5 % |

Nothing else is turned on: no slip, no odometry jitter, no GPS dropout, no thermal IMU drift. One wrong
number and nothing beside it, so the 5 % in the file is the 5 % on the screen. The IMU keeps its default
100 Hz, which makes the same drive usable for a second check: the integral of `gz` against the yaw the
odometry reports.

`view.layers` asks for `ghost` and `trails`, because the whole point is the ghost walking away from the
robot while both stand in an empty hall.

## What you see

- `o` — the ghost, ahead on the line of travel, with `odom off` in the readout naming the gap in metres;
- `t` — the trail of that belief;
- the dot, which is where the robot really is, and no fix anywhere near it.

## What it measured

20.00 m driven straight east at 0.5 m/s from the spawn, seed 1, 40.1 s of simulation time:

| `odom.geometry.wheel_radius_scale` | the odometry reports | ghost away from the robot | wall contacts |
|---|---|---|---|
| `1.0` (default) | 20.01 m | 0.01 m | 0 |
| `1.05` (this demo) | 21.01 m | **1.00 m** | 0 |

Halfway, after 12 m, the readout reads `odom off 0.60 m` — the same 5 cm per metre all the way. Both
runs end with **0 wall contacts**, because there is nothing to collide with.

That contrast is the page's whole argument: in a furnished hall the same class of error ends in a
collision instead of in a number. [demos/odom_error.md](odom_error.md) measures **26 wall contacts** for
task T3 with a 5 % radius and a 3 % lever arm. Here the error ends as a number a student can compare
with 5 % of 20 m.

## The exercise

Drive the 20 m, read the drift, and decide before looking whether the number is the 5 % you asked for.
Then say which of the graded tasks would have caught it: `quadrat` (T2) drives on odometry only and
corrects itself by closing the square, `korridor` (T3) does not, and in Experiment 2 every filter that
fuses odometry inherits exactly this bias — which is what `kf_fusion` measures during its GPS outage.

Model: `docs/CONTRACT.md` §6.4. The hall itself and the passage widths of all five arenas:
[docs/worlds.md](../worlds.md).

See also: [docs/demos.md](../demos.md) (all six demos and the wheel-slip section),
[demos/odom_error.md](odom_error.md), [demos/poi_exploration.md](poi_exploration.md) (the same hall with
one thing in it to find).
