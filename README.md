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
| `SPACE` pause · `q` quit | as before |
| `m` | opens the layer menu (starts closed so it covers nothing): lidar scan, odometry trail, gps fix, estimate + σ, wheels, velocity, floor markings, goal, readout lines |
| `l t g k w v d z h` | switch a single layer — the same as clicking its row |

The menu switches **drawing only**. `/<robot>/scan`, `/odom`, `/gps`, `/imu` and your `kf/pose`
keep running at full rate; `ros2 topic hz /alice/scan` does not care what the window shows. That
is deliberate: hide the dots, keep the data, and see which layer belongs to which topic.

## Task and arena belong together

`--task` takes task ids, groups or a comma list: `v1` and `alle` (Experiment 1),
`kf_alle`/`v2` (Experiment 2), `beide` (really all). Every task names its arena in
`config/tasks.json` (`"welt"`); `./lab` and `tools/fastgrade.py` follow that setting,
`--world` overrides it. Grading in the wrong arena gets you wall contacts
instead of points.

![The four arenas of the simulator: arena (24 × 16 m, mostly open, the four state-estimation
tasks), maze (6.5 × 5.5 m maze), production (20 × 12 m hall with tables, the four kinematics
and odometry tasks) and track (18 × 11 m ring). Each panel shows walls, floor markings, the
goal as a bullseye and every spawn as a coloured dot with its heading.](docs/img/worlds.png)

*The four arenas, drawn by `python3 tools/worldpic.py`.* The picture is generated from
`worlds/*.txt` and from the task titles in `config/tasks.json` — add an arena or move a task
and the figure follows when you regenerate it (`tools/check.sh` does that as a check, so a
stale image cannot survive a build).

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
tools/check.sh            # teaching team's gate (tests, grading, budgets, pictures)
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
