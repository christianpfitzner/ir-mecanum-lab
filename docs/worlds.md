# The five arenas, and the hall that is deliberately empty

The passage widths quoted below are the checker's, not the author's: `tools/worldcheck.py` measures
them, `tools/worldpic.py` prints them under the panels, and a test compares this page with the tool.

## An empty hall, and a source to find

`--world open` is 30 × 20 m of floor with a border wall and nothing else: no obstacle, no floor
marking, no goal, two spawns on the centre line. Odometry drift is the subject there — in every other
arena a wrong wheel constant shows up as a collision first, here it stays what it is: a number.
Measured on 20 m driven straight at 0.5 m/s (seed 1, 40.1 s):

| `odom.geometry.wheel_radius_scale` | odometry counted | ghost away from the robot | wall contacts |
|---|---|---|---|
| 1.0 (default) | 20.01 m | 0.01 m | 0 |
| 1.05 — `config/demo_open_odrift.json`, GPS switched off | 21.01 m | **1.00 m** | 0 |

Two commands, the first with the wrong wheel constant and GPS switched off — hold `Up` and watch the `o`
ghost walk away from the robot while the truth stays on the line:


```bash
ros2 launch mecanum_lab demo_open_odrift.launch.py --show-args
```

(That file asks for `"world": "open"` itself, so the hall needs no argument — and `--show-args` says so.)
The same hall with honest wheels, to see what the 1.00 m above are compared against:

```bash
ros2 launch mecanum_lab lab.launch.py world:=open
```

Without ROS 2 the same two runs are `./lab sim --world open --config config/demo_open_odrift.json` and
`./lab sim --world open`.

## Task and arena belong together

`--task` takes task ids, groups or a comma list: `v1` and `alle` (Experiment 1),
`kf_alle`/`v2` (Experiment 2), `beide` (really all). Every task names its arena in
`config/tasks.json` (`"world"`); `./lab` and `tools/fastgrade.py` follow that setting,
`--world` overrides it. Grading in the wrong arena gets you wall contacts
instead of points.

This is why **a run that grades announces the task it grades**, whatever door it came in through. Naming
only the grade — `./lab sim --grade alle`, or `ros2 launch … grade:=alle` before this was fixed — left the
announcement empty, and an empty announcement means the hall of `config/default.json`, which is `maze`.
That run is one command long to reproduce, `./lab grade --task alle --world maze --controller
student/solution.py`, and it answers **40/100**: T1 and T4 still pass, `quadrat` falls to 0/30 over three
wall contacts and `korridor` to 0/30 over thirty-one — the 12.9 m corridor of `production` is a zigzag
between 0.55 m walls there. Nothing in the report is lying; it is a correct measurement of the wrong hall.
`node.graded_task()` is the one place where that rule lives; `--world` on top of a grade still wins,
because an override is the point of an override.

![The five arenas at one common scale, in three columns: arena (24 × 16 m, open hall, the four
state-estimation tasks), maze (13 × 11 m built on 1 m grid cells), open (30 × 20 m, border walls
only — the hall for drift work), production (20 × 12 m hall with six tables, the four kinematics
and odometry tasks) and track (18 × 11 m ring around a central island). Solid blocks are walls that
collide, `-` and `|` are painted floor — free to drive over, drawn nowhere — dots are the start poses of
robots 1–4 with their heading, the bullseye is the goal.](docs/img/worlds.png)

*The five arenas, drawn by `python3 tools/worldpic.py` — one metre has the same thickness in
every panel, so the halls are comparable.* The numbers under the panels come from the same
sources the checks use: task titles from `config/tasks.json`, and the width of the tightest
passage on the widest start→goal path from `tools/worldcheck.py`. `arena` is deliberately open
(5.25 m at its narrowest) because state estimation wants free space and a GPS outage in a
corner; `maze` is deliberately tight (0.50 m free where the robot needs 0.46 m) because that
is what makes odometry hard. `open` has no goal and nothing in it, so there is no path to quote a
passage for and its panel states what the same check measures without one: 9.25 m of free space in
the widest spot, 0.46 m of it needed — the number is the hall, not a gap between two walls. Add an
arena or move a task and the figure follows when you regenerate it — `tools/check.sh` draws it as a
check, so a stale image cannot survive a build, and two tests compare these numbers with the tool
rather than with this file.


Arenas: `arena` (open, Experiment 2), `production`, `maze`, `track`, `open` (nothing but floor and
border, for drift work) — or your own
`worlds/name.txt` (`python3 tools/worldcheck.py --world name` checks it, including that the
world is closed). One grid cell is 0.5 m; `"worlds": {"cell_by_world": {"maze": 1.0}}` in
`config/default.json` makes a single world coarser without touching the robot — that is why the
maze has room to drive in while the graded arenas stay as they are.

Add an arena by putting `worlds/name.txt` in place and checking it with `python3 tools/worldcheck.py --world name`.
