# Mecanum lab — two experiments in one 2D simulator

A Python simulator for the lab course: **Experiment 1** teaches mecanum kinematics (the
student drives the tasks), **Experiment 2** teaches state estimation (the simulator drives,
the student estimates with a Kalman filter). Both run headless, without ROS 2 and without
numpy — ROS 2 is an optional layer, not a requirement.

## Install in one command

```bash
./install.sh              # checks python/pygame/pytest/ROS, offers to install what is missing
./install.sh --check      # report only, install nothing
./install.sh --mit-tests  # plus a short self test (simulation + unit tests, headless)
```

`./install.sh` writes nothing into your home directory and needs no sudo. Without an
internet connection: `sudo apt install python3-pygame` (or `./install.sh --user`).

## First steps — Experiment 1 (kinematics)

```bash
./lab run --robot alice --controller student/controller_template.py   # window, drive yourself
./lab grade --task v1 --controller student/solution.py               # grade all tasks
```

`./lab run` is the one-process form: the simulator, your node as a thread and the keyboard, all three
on the same `/cmd_vel`. A node publishes every tick and the keys publish only while one is held, so
your node drives and the keys interrupt it — and the readout line says which of the two was last
(`cmd topic 0.04 s`, `cmd keys 0.02 s`, `cmd none` once `cmd_timeout` passed without a frame).

**On `kinematik` the robot stands still, and that is correct.** For task `""` or `kinematik` the runner
behind `serve()` never calls `mission()`: it reads every `cmd_vel` that arrives, pushes it through your
`inverse_kinematics()` and publishes the four wheel speeds — because T1 is graded by the grader sending
commands blind and measuring what the wheels do. So hold an arrow key to watch the conversion: a key *is*
a `cmd_vel`, and it is your IK that turns it into wheels. Missions start at T2: `--task quadrat` runs
`mission()` once and the robot drives the square by itself.

## How it works

![The chassis on the left, the message loop on the right. Left: the four wheels on their axle points,
each with its roller axis at 45°, and the three body velocities. Right: one `cmd_vel` becoming four wheel
speeds becoming a pose becoming the sensor messages becoming an estimate.](docs/img/howitworks.png)

Both halves are drawn by `tools/labmap.py` out of the modules the exercise is graded with — the wheel
mounts from `render.wheel_mounts()`, the roller axes from `physics.inverse_kinematics()`, the topic names
from `types.topic()` — and the tool refuses to draw if a roller axis is not perpendicular to the velocity
its own wheel produces. The one rule the left half is about: a mecanum wheel pushes only across its
roller axis, so with all four rollers of one sign the sideways parts cancel and the robot goes forward,
and with the front pair against the rear pair the forward parts cancel and it strafes. What the loop does
with a number that is wrong on purpose — wheel slip at a wall, wheel constants in `odom.geometry` — is
`docs/demos.md`; signs and topic contracts: `docs/CONTRACT.md` §5 and §6.

## First steps — Experiment 2 (state estimation)

The simulation drives; your node only measures and reports its estimate on
`/<robot>/kf/pose` — position **and** its own 1σ, because the grade hangs on the uncertainty.

```bash
./lab run --task kf_gps --robot alice --controller student/kf_template.py --truth
./lab grade --task kf_alle --controller student/kf_template.py --log messung.csv
python3 tools/kfplot.py messung.csv       # numbers + ASCII plot, no matplotlib
```

Four tasks: `kf_gps` (CV model, GPS only) → `kf_fusion` (GPS + odometry + IMU through an
8 s GPS outage) → `kf_kovarianz` (the stated σ must match the error, NEES) → `kf_dynamik`
(fast, 200 Hz IMU). Handout: `docs/praktikum/kalman.tex` (`make kalman`).

## With ROS 2 (Kilted or newer)

`./install.sh` builds the ROS package as a second step — colcon is needed for that
(`sudo apt install ros-$ROS_DISTRO-dev-tools`, and `install.sh` says the same when it is missing) — so
every demo then starts by package name from any directory:

```bash
source /opt/ros/kilted/setup.bash && ./install.sh
source install/setup.bash                         # every new terminal, until ROS is the only one
ros2 launch mecanum_lab lab.launch.py             # sim + window, the keyboard drives 'muster'
ros2 launch mecanum_lab demo_wifi.launch.py       # one demo by name — six of them, table below
ros2 launch mecanum_lab kf.launch.py              # Experiment 2, every knob mapped onto --set
ros2 topic echo /muster/imu --once                # az at rest ≈ +9.81 — that is correct
ros2 launch mecanum_lab lab.launch.py --show-args # what a launch file takes
```

Nothing needs that build: every file also launches by path from this tree
(`ros2 launch launch/demo_wifi.launch.py`), and the plain commands stay:
`./lab sim --robots alice,bob`, `./lab spawn --name carlo`, `./lab rviz --robot alice`. To keep the
in-process bus even with ROS sourced: `MECANUM_ROS=stub ./lab sim --headless`.

By default a launch file starts the simulator and the window and **lets the keyboard drive**; a node
only drives when you ask for one (`controller:=student/solution.py`), because a node that publishes
every tick silences the keys — which looks exactly like a broken keyboard.

## The simulator window

| input | what it does |
|---|---|
| mouse wheel | zoom **to the cursor** · `+`/`-` zoom to the middle · `f` shows the whole world |
| drag with the left button | pan (the map follows the mouse) · right button centres again |
| `1`…`9` / `0` | follow one robot / show everything · window resizable, the camera keeps up |
| `w`/`s` drive · `a`/`d` turn | car-style keyboard driving, on unless `--no-teleop`; the keys set body speeds, this is not a game |
| `Up`/`Down` drive · `Left`/`Right` **strafe** | the same keys as the four degrees of freedom of a mecanum chassis instead of a car's two |
| `q`/`e` (or `,`/`.`) turn left/right | ±0.9 rad/s yaw. With teleop on, `q` **turns** instead of quitting — `ESC` or the window's close button ends the run |
| `SPACE` pause | `q` quits only when teleop is off |
| `m` | opens the layer menu (starts closed so it covers nothing): lidar scan, odometry trail, gps fix, estimate + σ ellipse, wheels, velocity vector, floor markings, goal, readout lines, gps shadow zones, odometry ghost + drift, radiation source + field, radio link + access point |
| `l t g k r v c z h x o p n` | switch a single layer — the same as clicking its row (`r` wheels, `c` floor markings, `x` GPS shadow, `o` odometry ghost, `p` radiation source and its field rings, `n` the radio link) |

The window starts **clean**: the four raw measurement layers (lidar scan, odometry trail, gps fix,
odometry ghost) are off, so the map stays visible — on the screen only, never on the bus, a
`ros2 topic echo` still sees all of it. `--view sensors` starts with everything on, and
`--layers scan,ghost,-hud` picks layers by hand. Layers, view profiles, and everything the readout line
carries (IMU, GPS quality, lost messages, LiDAR echoes): **[docs/window.md](docs/window.md)**.

## The demos

Each demo changes one block of the config and leaves the graded defaults alone, so starting one is a
`--config`, not an edit. Each has a launcher of its own — and
`ros2 launch launch/demo.launch.py demo:=wifi` is the same thing with the name as an argument.

| demo | launcher | what you see |
|---|---|---|
| `gps_shadow` | `demo_gps_shadow.launch.py` | hatched shadow (`x`), fixes that scatter, then none at all |
| `odom_error` | `demo_odom_error.launch.py` | the ghost (`o`) drifts ahead: 0.61 m per 12 m of straight lane |
| `open_odrift` | `demo_open_odrift.launch.py` | dead reckoning in an empty hall: the trail (`t`) where odometry thinks it has been |
| `sensor_reality` | `demo_sensor_reality.launch.py` | `sensor_state` in the readout: `lost_gps`, `q_gps`, `temp_imu` |
| `wifi` | `demo_wifi.launch.py` | the `link` panel (`n`), and a robot that loses its autonomy mid-lane |
| `poi_exploration` | `demo_poi_exploration.launch.py` | the `poi` panel (`p`) and a robot hunting by one number |

Wheel slip has no file of its own: it is `robot.slip`, and `./lab sim --world arena` with the robot
driven into the east wall shows the odometry counting metres that were never driven. The commands, the
measured numbers behind these rows and what each demo costs a graded run:
**[docs/demos.md](docs/demos.md)**.

## Where configuration lives

| Layer | File / option | Note |
|---|---|---|
| Defaults | `mecanum_lab/types.py` → `DEFAULT_CONFIG` | every sensor number of both experiments |
| Site | `config/default.json` | overrides the defaults |
| Task profile | `config/tasks.json` → `sim` | measured sensing per task (GPS rate, σ, outage, IMU) |
| Command line | `--set gps.sigma_xy=1.2 --set imu.rate=400 --set gps.gap='[14,8]'` | always wins |
| Launch file | `… launch.py --show-args` | every `--set` key is also a launch argument |

## The rest, by topic

| page | what is on it |
|---|---|
| [docs/demos.md](docs/demos.md) | the six demos in detail, and what makes a run go fast (`--speed`, `--fixed-step`) |
| [docs/window.md](docs/window.md) | layers, view profiles, and what the readout line is showing |
| [docs/worlds.md](docs/worlds.md) | the five arenas at one scale, task↔arena, the empty hall for drift work |
| [docs/steering.md](docs/steering.md) | the second drive train (Ackermann) and its two measured lessons |
| [docs/poi.md](docs/poi.md) | the radiation source `/poi`, its field, and why it goes through tables |
| [docs/wifi.md](docs/wifi.md) | the radio link `/link`, and what `autonomy` means when it drops |
| [docs/CONTRACT.md](docs/CONTRACT.md) | the interface of Experiment 1: topics, units, signs, geometry |
| [docs/CONTRACT-KF.md](docs/CONTRACT-KF.md) | the interface of Experiment 2 |
| `docs/praktikum/` | the handouts: `make anleitung`, `make kalman` |

## Useful commands

```bash
./lab docs                # topics, tasks, examples
./lab robots              # who is driving right now?
./lab grade --task kf_gps --controller student/kf_solution.py --json bericht.json
tools/check.sh            # teaching team's gate (tests, grading, budgets, hygiene)
python3 tools/fastgrade.py --task kf_alle --controller student/kf_solution.py --speed 25
```

## When something does not work

* **`pygame is missing`** → `./install.sh`, or run everything with `--headless`.
* **No display / SSH** → `--headless` (or `SDL_VIDEODRIVER=dummy`).
* **`ros2: command not found`** → source `/opt/ros/<distro>/setup.bash`; everything except
  `ros2 …` works without it too.
* **The robot ignores the keyboard** → a node is publishing `cmd_vel` every tick: start without
  `controller:=`, or `--no-teleop` to watch the node on purpose.
* **Grading reports `no measurement pairs`** → your node must publish `/<robot>/kf/pose`,
  and the sim must publish truth (`--truth`, automatic under `./lab grade`).
* **Estimate lags behind the measurements** → you used the wall clock. What counts are the
  message stamps (`fix.t`, `odom.t`), never `time.time()`.
* **Second task starts far away** → discard the previous task's measurements
  (`fix.t >= mission start`), see the handout, section "Three rules".
* **Lists** → `./lab docs`.

## Rules of this codebase

Stdlib + pygame (no numpy/scipy/yaml in the simulator or in the student files), **everything written
and everything named in English** — comments, docstrings, report text, handouts, identifiers, launch
arguments. Two tools check it, and both are a step of `tools/check.sh`: `tools/langcheck.py` reads the
prose, `tools/germanids.py` the names. What stays German is what the tool lets through — ROS field
names, the wheel names `VL/VR/HL/HR`, the task groups on the command line, and the two compatibility
maps that keep old shell histories working (`docs/CONTRACT.md` §6.11). Deterministic from `--seed`,
headless-capable.
