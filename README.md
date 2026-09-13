# Mecanum lab — two experiments in one 2D simulator

A Python simulator for the lab course: **Experiment 1** teaches mecanum kinematics (the
student drives the tasks), **Experiment 2** teaches state estimation (the simulator drives,
the student estimates with a Kalman filter). Everything here is started with ROS 2 — and every
run also works headless with pygame alone, without ROS 2 and without numpy, on a machine that
has none.

## Install in one command

```bash
./install.sh
```

Checks python/pygame/pytest/ROS, offers to install what is missing and builds the ROS package as a
second step — after that, every command below starts by package name. Nothing is written into your home
directory, no sudo is needed. To only look and change nothing, run `./install.sh --check`; to follow that
with a short self test (simulation + unit tests, headless), run `./install.sh --mit-tests`. Without an
internet connection: `sudo apt install python3-pygame` (or `./install.sh --user`).

## One run, two doors — and one command per block

Everything that drives is started with ROS 2. That is the standard form, the one the lab room types, and
the form every example in this repository is written in:

```bash
ros2 launch mecanum_lab lab.launch.py
```

`./lab` is the same simulator in one process — pygame and the standard library, no ROS, no build, no
daemon. It is the door for a machine without ROS 2 and the one the CI walks through; both doors read the
same config layers, so it is the same run. Where a block below is not a `ros2 …` command, that is why.

```bash
./lab sim
```

Two habits hold this documentation together. **One command per block**: what you see is what you type,
once per block, no `cd` in front of it and no second line waiting behind it. And **a launch file is named
by its package**, which works from any directory — before the build there is no package name, and until
then the same file is started by path (`ros2 launch launch/lab.launch.py`).

`ros2 run mecanum_lab mecanum-lab sim --world open` is `./lab sim --world open` under the package's own
name: the same CLI, the same options, no wrapper. It needs the `ros2 run` extension
(`sudo apt install ros-$ROS_DISTRO-ros2run`), which is not part of a ROS base install — `./install.sh --check`
says whether this machine has it, and nothing here depends on it.

## First steps — Experiment 1 (kinematics)

Your node, the simulator and the window — RViz comes with it when rviz2 is installed:

```bash
ros2 launch mecanum_lab lab.launch.py controller:=student/controller_template.py
```

Graded is a one-process run, because that is what the limits are calibrated on — grader, simulator and
your node on one clock and one bus:

```bash
./lab grade --task alle --controller student/solution.py
```

Without ROS 2 the first command is `./lab run --robot alice --controller student/controller_template.py`.
A launch file can grade too (`grade:=alle`), and it is not the same measurement: your node is a second
process then, and measured on one seed the reference solution comes out at **70/100** through that door
and **100/100** through this one — see ["When something does not work"](#when-something-does-not-work)
and `docs/CONTRACT.md` §9.2.

`./lab run` is the one-process form: the simulator, your node as a thread and the keyboard, all three on
the same `/cmd_vel` — your node drives, the keys interrupt it, and the readout line says who last did. Over
ROS 2 they are two processes on the same topic, which is the same argument with a network in the middle.
On `kinematik` the robot stands still until you press a key, and that is correct: T1 is graded by sending
commands blind and measuring the four wheel speeds, so a key *is* a `cmd_vel` and your IK is what turns it
into wheels. Why `serve()` and `mission()` divide the work that way, and what each line of the readout
means: **[docs/kinematics.md](docs/kinematics.md)**.


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

The workbench for one task, with the window and the truth on `/<robot>/truth`:

```bash
ros2 launch mecanum_lab kf.launch.py task:=kf_gps controller:=student/kf_template.py
```

All four tasks graded, with the measurement series the report is written from — grading is the
one-process command for both experiments, because the truth, the sensors and your filter have to be
measured on one clock:

```bash
./lab grade --task kf_alle --controller student/kf_template.py --log messung.csv
```

Numbers and an ASCII plot out of that file — a tool of this tree, so this one is a `python3` line:

```bash
python3 tools/kfplot.py messung.csv
```

Without ROS 2: `./lab run --task kf_gps --robot alice --controller student/kf_template.py --truth`, and
`./lab grade --task kf_alle --controller student/kf_template.py --log messung.csv`.

Four tasks: `kf_gps` (CV model, GPS only) → `kf_fusion` (GPS + odometry + IMU through an
8 s GPS outage) → `kf_kovarianz` (the stated σ must match the error, NEES) → `kf_dynamik`
(fast, 200 Hz IMU). Handout: `docs/praktikum/kalman.tex` (`make kalman`).

## Working with ROS 2 (Kilted or newer)

`./install.sh` builds the ROS package as a second step — colcon is needed for that
(`sudo apt install ros-$ROS_DISTRO-dev-tools`, and `install.sh` says the same when it is missing). In
every new terminal, the workspace overlay is the one line to type:

```bash
source install/setup.bash
```

Everything else is a `ros2 launch` of that package, and what a launch file takes, it lists itself:

```bash
ros2 launch mecanum_lab lab.launch.py --show-args
```

Two things worth knowing from a second terminal. The accelerometer at rest reads ≈ +9.81, and that is
correct:

```bash
ros2 topic echo /muster/imu --once
```

A further robot, with the name the simulator chose for it:

```bash
ros2 service call /sim/spawn_next std_srvs/srv/Trigger
```

Nothing needs that build: every file also launches by path from this tree
(`ros2 launch launch/demo_wifi.launch.py`). Without ROS 2 the same three things are
`./lab sim --robots alice,bob`, `./lab spawn --name carlo` and `./lab rviz --robot alice`; to keep the
in-process bus even with ROS sourced: `MECANUM_ROS=stub ./lab sim --headless`.

By default a launch file starts the simulator and the window and **lets the keyboard drive**; a node
only drives when you ask for one (`controller:=student/solution.py`), because a node that publishes
every tick silences the keys — which looks exactly like a broken keyboard.

## The simulator window

| input | what it does |
|---|---|
| mouse wheel | zoom **to the cursor** · `+`/`-` zoom to the middle · `f` shows the whole world |
| drag with the left button | pan (the map follows the mouse) · middle button centres again |
| right button | *place a robot here* — one row per robot that drives, at the pointer; the heading stays, `esc` or a second right-click closes it (nothing is asked for over a rack: there is no free floor to place one) |
| `1`…`9` / `0` | follow one robot / show everything · window resizable, the camera keeps up |
| `w`/`s` drive · `a`/`d` **strafe** | the two axes a mecanum chassis has, on the four letters everybody reaches for; the keys set body speeds, this is not a game |
| `Up`/`Down` drive · `Left`/`Right` **strafe** | the same two axes as arrows, for the hand that prefers them — a car has one of these two, this base has both |
| `q`/`e` (or `,`/`.`) turn left/right | ±0.9 rad/s yaw. With teleop on, `q` **turns** instead of quitting — `ESC` or the window's close button ends the run |
| `SHIFT` held while driving | both speeds doubled — 0.7 m/s and 1.8 rad/s instead of 0.35 and 0.9, the same curve, for the long lanes |
| `SPACE` pause | `q` quits only when teleop is off |
| `m` | opens the layer menu (starts closed so it covers nothing): lidar scan, odometry trail, gps fix, estimate + σ ellipse, wheels, velocity vector, goal, readout lines, gps shadow zones, odometry ghost + drift, radiation source + field, radiation dose map, radio link + access point, radio coverage map |
| `l t g k r v z h x o p i n c` | switch a single layer — the same as clicking its row (`r` wheels, `c` radio coverage map, `x` GPS shadow, `o` odometry ghost, `p` radiation source and its field rings, `i` the dose map of that field, `n` the radio link) |
| the pointer | shows the world coordinate under it (`12.40, 6.20 m`), inside the hall |

The window starts **clean**: the four raw measurement layers (lidar scan, odometry trail, gps fix,
odometry ghost) are off, so the map stays visible — on the screen only, never on the bus, a
`ros2 topic echo` still sees all of it. `--view sensors` starts with everything on, and
`--layers scan,ghost,-hud` picks layers by hand. Layers, view profiles, everything the readout line
carries (IMU, GPS quality, lost messages, LiDAR echoes), and why every label is drawn on a dark edge:
**[docs/window.md](docs/window.md)**.

## The demos

Six demos, each changing one block of the config and leaving the graded defaults alone, so starting one
is a `--config` and not an edit. Each has a launcher of its own, and
`ros2 launch mecanum_lab demo.launch.py demo:=wifi` is the same thing with the name as an argument.

| demo | what it turns on | page |
|---|---|---|
| `gps_shadow` | GPS that gets bad by place: shadow, bias, then no fix at all | [demos/gps_shadow.md](docs/demos/gps_shadow.md) |
| `odom_error` | odometry built on wheels a half-centimetre too large | [demos/odom_error.md](docs/demos/odom_error.md) |
| `open_odrift` | the same wrong radius in an empty hall, GPS off | [demos/open_odrift.md](docs/demos/open_odrift.md) |
| `sensor_reality` | latency, dropout, staleness, a chip that warms up | [demos/sensor_reality.md](docs/demos/sensor_reality.md) |
| `wifi` | commands that travel by radio, and a radio with a range | [demos/wifi.md](docs/demos/wifi.md) |
| `poi_exploration` | a radiation source somewhere in the hall, one number to find it | [demos/poi_exploration.md](docs/demos/poi_exploration.md) |

Every page has the commands, the keys, what the window shows and the numbers that were measured for it.
Wheel slip has no file of its own: it is `robot.slip`, and driving east into the wall in `arena` shows the
odometry counting metres that were never driven — `ros2 launch mecanum_lab lab.launch.py world:=arena`,
or `./lab sim --world arena` without ROS 2. What makes a run go fast:
**[docs/demos.md](docs/demos.md)**.

## Where configuration lives

| Layer | File / option | Note |
|---|---|---|
| Defaults | `mecanum_lab/types.py` → `DEFAULT_CONFIG` | every sensor number of both experiments |
| Site | `config/default.json` | overrides the defaults |
| Task profile | `config/tasks.json` → `sim` | measured sensing per task (GPS rate, σ, outage, IMU) |
| Command line | `--set gps.sigma_xy=1.2 --set imu.rate=400 --set gps.gap='[14,8]'` | always wins |
| Launch file | `… launch.py --show-args` | every `--set` key is also a launch argument |

A launch argument counts only when it was typed. `world:=`, `view:=` and the sensor knobs are empty by
default and are then not passed on at all, because a launch file that hands over its own default for a
knob nobody named outbids the `config:=` file — which is how a demo with `"world": "open"` in it came to
open `production`. Name it and it wins, leave it out and the config decides.

## The rest, by topic

| page | what is on it |
|---|---|
| [docs/demos.md](docs/demos.md) | the six demos — one page each — and what makes a run go fast (`--speed`, `--fixed-step`) |
| [docs/kinematics.md](docs/kinematics.md) | Experiment 1: `serve()` and `mission()`, and why the robot stands still on T1 |
| [docs/window.md](docs/window.md) | layers, view profiles, and what the readout line is showing |
| [docs/worlds.md](docs/worlds.md) | the five arenas at one scale, task↔arena, the empty hall for drift work |
| [docs/steering.md](docs/steering.md) | the second drive train (Ackermann) and its two measured lessons |
| [docs/poi.md](docs/poi.md) | the radiation source `/poi`, its field, and why it goes through tables |
| [docs/wifi.md](docs/wifi.md) | the radio link `/link`, and what `autonomy` means when it drops |
| [docs/CONTRACT.md](docs/CONTRACT.md) | the interface of Experiment 1: topics, units, signs, geometry |
| [docs/CONTRACT-KF.md](docs/CONTRACT-KF.md) | the interface of Experiment 2 |
| `docs/praktikum/` | the handouts: `make anleitung`, `make kalman` |

## Useful commands

What is on the bus, what the tasks are, what the examples are:

```bash
./lab docs
```

Who is driving right now:

```bash
ros2 topic echo /sim/robots --once
```

The teaching team's gate (tests, grading, budgets, hygiene):

```bash
tools/check.sh
```

Grading in one line — and at four times real time, which is the supervisor's pace, not the student's:

```bash
./lab grade --task alle --controller student/solution.py --json bericht.json
```

```bash
python3 tools/fastgrade.py --task kf_alle --controller student/kf_solution.py --speed 25
```

## When something does not work

* **`pygame is missing`** → `./install.sh`, or run everything with `--headless`.
* **No display / SSH** → `--headless` (or `SDL_VIDEODRIVER=dummy`).
* **`ros2: command not found`** → source `/opt/ros/<distro>/setup.bash`; everything except
  `ros2 …` works without it too.
* **The robot ignores the keyboard** → a node is publishing `cmd_vel` every tick: start without
  `controller:=`, or `--no-teleop` to watch the node on purpose.
* **A grade that does not repeat** → grade with `./lab grade`, not with `grade:=` in a launch file. The
  launch form runs your node as a second process on a real network, and the tasks that drive by odometry
  measure that: the reference solution is 100/100 through `./lab grade --task alle` and 70/100 through
  `grade:=alle` on the same seed, because T3 gives up on its own estimate after 4.6 m of the 12.9 m it
  needs. Every limit in `config/tasks.json` is calibrated on the one-process run (CONTRACT §9).
* **Grading reports `no measurement pairs`** → your node must publish `/<robot>/kf/pose`,
  and the sim must publish truth (`--truth`, `truth:=true`, on by default in `kf.launch.py`).
* **Estimate lags behind the measurements** → you used the wall clock. What counts are the
  message stamps (`fix.t`, `odom.t`), never `time.time()`.
* **Second task starts far away** → discard the previous task's measurements
  (`fix.t >= mission start`), see the handout, section "Three rules".
* **Lists** → `./lab docs`.

Two rules hold the tree together — stdlib + pygame only, and everything named and written in English —
both checked by `tools/check.sh`, both explained where they are enforced (`docs/CONTRACT.md` §1 and
§6.11, `tools/langcheck.py`, `tools/germanids.py`). Everything is deterministic from `--seed`.
