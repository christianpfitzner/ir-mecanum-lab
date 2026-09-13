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

`./lab run` is the one-process form: the simulator, your node as a thread and the keyboard, all three on
the same `/cmd_vel` — your node drives, the keys interrupt it, and the readout line says who last did.
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
ros2 service call /sim/spawn_next std_srvs/srv/Trigger   # another robot, name chosen for you
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
| `w`/`s` drive · `a`/`d` **strafe** | the two axes a mecanum chassis has, on the four letters everybody reaches for; the keys set body speeds, this is not a game |
| `Up`/`Down` drive · `Left`/`Right` **strafe** | the same two axes as arrows, for the hand that prefers them — a car has one of these two, this base has both |
| `q`/`e` (or `,`/`.`) turn left/right | ±0.9 rad/s yaw. With teleop on, `q` **turns** instead of quitting — `ESC` or the window's close button ends the run |
| `SHIFT` held while driving | both speeds doubled — 0.7 m/s and 1.8 rad/s instead of 0.35 and 0.9, the same curve, for the long lanes |
| `SPACE` pause | `q` quits only when teleop is off |
| `m` | opens the layer menu (starts closed so it covers nothing): lidar scan, odometry trail, gps fix, estimate + σ ellipse, wheels, velocity vector, goal, readout lines, gps shadow zones, odometry ghost + drift, radiation source + field, radio link + access point, radio coverage map |
| `l t g k r v z h x o p n c` | switch a single layer — the same as clicking its row (`r` wheels, `c` radio coverage map, `x` GPS shadow, `o` odometry ghost, `p` radiation source and its field rings, `n` the radio link) |
| the pointer | shows the world coordinate under it (`12.40, 6.20 m`), inside the hall |

The window starts **clean**: the four raw measurement layers (lidar scan, odometry trail, gps fix,
odometry ghost) are off, so the map stays visible — on the screen only, never on the bus, a
`ros2 topic echo` still sees all of it. `--view sensors` starts with everything on, and
`--layers scan,ghost,-hud` picks layers by hand. Layers, view profiles, and everything the readout line
carries (IMU, GPS quality, lost messages, LiDAR echoes): **[docs/window.md](docs/window.md)**.

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
Wheel slip has no file of its own: it is `robot.slip`, and `./lab sim --world arena` into the east wall
shows the odometry counting metres that were never driven. What makes a run go fast:
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

Two rules hold the tree together — stdlib + pygame only, and everything named and written in English —
both checked by `tools/check.sh`, both explained where they are enforced (`docs/CONTRACT.md` §1 and
§6.11, `tools/langcheck.py`, `tools/germanids.py`). Everything is deterministic from `--seed`.
