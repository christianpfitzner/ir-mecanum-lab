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

```bash
source /opt/ros/kilted/setup.bash
./lab sim --robots alice,bob                      # simulator as a real ROS node
./lab spawn --name carlo                          # add a robot to a running sim
ros2 launch launch/kf.launch.py                   # Experiment 2, everything configurable
ros2 launch launch/kf.launch.py bewerten:=kf_alle controller:=student/kf_solution.py
ros2 launch launch/sim.launch.py robots:=alice,bob  # Experiment 1
ros2 topic echo /alice/imu --once                 # az at rest ≈ +9.81 — that is correct
ros2 topic echo /tf --once                        # map → alice/odom → alice/base_link → laser
rviz2 -d rviz/kf.rviz --ros-args -p use_sim_time:=true   # the sim stamps TF in sim seconds
```

To keep the in-process bus even with ROS: `MECANUM_ROS=stub ./lab sim --headless`.

## The simulator window (what you hide is not what is published)

| input | what it does |
|---|---|
| mouse wheel | zoom **to the cursor** · `+`/`-` zoom to the middle · `f` shows the whole world |
| drag with the left button | pan (the map follows the mouse) · right button centres again |
| `1`…`9` / `0` | follow one robot / show everything · window resizable, the camera keeps up |
| `Up`/`Down` drive · `Left`/`Right` **strafe** | keyboard driving, on unless `--no-teleop`; the keys set body speeds, this is not a game |
| `q` or `,` turn right · `e` or `.` turn left | ±0.9 rad/s yaw. With teleop on, `q` **turns** instead of quitting — `ESC` or the window's close button ends the run |
| `SPACE` pause | `q` quits only when teleop is off |
| `m` | opens the layer menu (starts closed so it covers nothing): lidar scan, odometry trail, gps fix, estimate + σ, wheels, velocity, floor markings, goal, readout lines, gps shadow zones, odometry ghost |
| `l t g k w v d z h s o` | switch a single layer — the same as clicking its row (`s` GPS shadow, `o` odometry ghost) |

The menu switches **drawing only**. `/<robot>/scan`, `/odom`, `/gps`, `/imu` and your `kf/pose`
keep running at full rate; `ros2 topic hz /alice/scan` does not care what the window shows. That
is deliberate: hide the dots, keep the data, and see which layer belongs to which topic. The
per-robot readout line also carries the IMU (`ax`, `ay`, `gz`) — it is live in **every** run, not
only in Experiment 2: default `imu.rate` is 100 Hz with bias random walk, scale error, tilt
cross-coupling and vibration, and `az ≈ +9.81 m/s²` while standing still.

### GPS that gets bad by place: the shadow demo (not the default)

`gps.zones` degrades the fix **where the robot is**, not when: the first rectangle that contains
it multiplies `sigma_xy`, adds a bias (multi-path pushes the fix away from the reflector) or
suppresses the fix completely. It ships empty, because the graded tasks in `config/tasks.json`
are calibrated on the plain GPS of each experiment — a permanent shadow would silently move what
every filter is graded against. `config/demo_gps_shadow.json` is the demo that turns it on:

```bash
./lab sim --world production --config config/demo_gps_shadow.json     # drive into it yourself
./lab run --world production --config config/demo_gps_shadow.json \
          --robot muster --controller student/solution.py --seconds 30
```

Measured with 400 fixes per spot (σ=0.06 m is the lab default):

| spot | σ of a fix | median error | mean offset | fixes |
|---|---|---|---|---|
| open floor | 0.06 / 0.06 m | 0.07 m | (−0.01, 0.00) m | 400/400 |
| under the high shelf (σ×6, bias +0.8/−0.5) | 0.37 / 0.36 m | 1.03 m | (+0.83, −0.51) m | 400/400 |
| multipath in the corner (σ×3) | 0.18 / 0.18 m | 0.52 m | (−0.39, +0.30) m | 400/400 |
| loading dock (`"block": true`) | — | — | — | **no fix at all** |

The window draws the zones as hatched shadow (`s`), the blackout in the error colour, and labels
each with what it does to the fix. Together with the odometry ghost (`o`) and the rubber that
slipping wheels leave on the floor, one frame shows a student the three ways a position can be
wrong: noisy, biased, or missing — and how much of it the odometry invented (`overlays.py`).

### Wheel slip: odometry you can watch lying

Press the robot against a wall and the odometry keeps counting metres that were never driven.
That is the drift of Experiment 1 in its purest form — no sensor noise needed. The model is
physical: against an obstacle the body stands still while the wheels keep the speed the motor
demands, and odometry integrates wheel speeds (`sensors.py` integrates what the wheels report,
never the truth). `robot.slip` scales it: `1` = full slip (default), `0` = ideal static friction
with honest odometry. Measured in `arena`, 4 s of full throttle east into the wall, seed 5:

| `robot.slip` | body really moved | odometry counted | phantom distance |
|---|---|---|---|
| `1` (default) | 0.69 m | 1.55 m | **+0.86 m** |
| `0` | 0.69 m | 0.69 m | −0.003 m |

By hand: `./lab sim --world arena`, then `Up` into the east wall — the `odom x` in the readout
climbs while the robot's dot stays where it is. To record it: `./lab grade --task kinematik
--controller student/solution.py --log messung.csv` writes `ax_imu, ay_imu, gz_imu` next to the
other columns (`python3 tools/kfplot.py messung.csv --list` shows all of them).

## Task and arena belong together

`--task` takes task ids, groups or a comma list: `v1` and `alle` (Experiment 1),
`kf_alle`/`v2` (Experiment 2), `beide` (really all). Every task names its arena in
`config/tasks.json` (`"welt"`); `./lab` and `tools/fastgrade.py` follow that setting,
`--world` overrides it. Grading in the wrong arena gets you wall contacts
instead of points.

![The four arenas at one common scale: arena (24 × 16 m, open hall, the four state-estimation
tasks), production (20 × 12 m hall with six tables, the four kinematics and odometry tasks),
maze (13 × 11 m built on 1 m grid cells) and track (18 × 11 m ring around a central island).
Solid blocks are walls that collide, dashed lines are painted floor markings without
collision, dots are the start poses of robots 1–4 with their heading, the bullseye is the
goal.](docs/img/worlds.png)

*The four arenas, drawn by `python3 tools/worldpic.py` — one metre has the same thickness in
every panel, so the halls are comparable.* The numbers under the panels come from the same
sources the checks use: task titles from `config/tasks.json`, and the width of the tightest
passage on the widest start→goal path from `tools/worldcheck.py`. `arena` is deliberately open
(5.25 m at its narrowest) because state estimation wants free space and a GPS outage in a
corner; `maze` is deliberately tight (0.50 m free where the robot needs 0.46 m) because that
is what makes odometry hard. Add an arena or move a task and the figure follows when you
regenerate it — `tools/check.sh` draws it as a check, so a stale image cannot survive a build.

## Where configuration lives

| Layer | File / option | Note |
|---|---|---|
| Defaults | `mecanum_lab/types.py` → `DEFAULT_CONFIG` | every sensor number of both experiments |
| Site | `config/default.json` | overrides the defaults |
| Task profile | `config/tasks.json` → `sim` | measured sensing per task (GPS rate, σ, outage, IMU) |
| Command line | `--set gps.sigma_xy=1.2 --set imu.rate=400 --set gps.gap='[14,8]'` | always wins |
| Launch file | `ros2 launch launch/kf.launch.py --show-args` | 46 arguments, all mapped onto `--set` |

Arenas: `arena` (open, Experiment 2), `production`, `maze`, `track` — or your own
`worlds/name.txt` (`python3 tools/worldcheck.py --welt name` checks it, including that the
world is closed). One grid cell is 0.5 m; `"worlds": {"cell_by_world": {"maze": 1.0}}` in
`config/default.json` makes a single world coarser without touching the robot — that is why the
maze has room to drive in while the graded arenas stay as they are.

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
* **Grading reports `no measurement pairs`** → your node must publish `/<robot>/kf/pose`,
  and the sim must publish truth (`--truth`, automatic under `./lab grade`).
* **Estimate lags behind the measurements** → you used the wall clock. What counts are the
  message stamps (`fix.t`, `odom.t`), never `time.time()`.
* **Second task starts far away** → discard the previous task's measurements
  (`fix.t >= mission start`), see the handout, section "Three rules".
* **Lists** → `./lab docs`.

## Rules of this codebase

Stdlib + pygame (no numpy/scipy/yaml in the simulator or in the student files), **all written
prose and all UI text in English** — comments, docstrings, report text, handouts,
`config/tasks.json`. That is checked: `python3 tools/langcheck.py` (also a step in
`tools/check.sh`). Identifiers are English too, except the ones that are API and would break
handouts and student code: the launch arguments (`sekunden`, `aufgabe`, `bewerten`, `wahrheit`,
`protokoll`, `aufzeichnung`), ROS topic names, JSON keys and the wheel names `VL/VR/HL/HR`.
Deterministic from `--seed`, headless-capable.
Details: `docs/CONTRACT.md` (Experiment 1), `docs/CONTRACT-KF.md` (Experiment 2).
