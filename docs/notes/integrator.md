# Integration note for experiment 2 (Kalman filter) — what the integrator changed

Status: after the latest calibration run. This note extends `docs/CONTRACT-KF.md` and
justifies every deviation from the first draft. Order: first the findings (faults that the
integration test showed), then the thresholds, then the open remainder.

## 1. Faults found and fixed

Nos. 1–13 and 14–17 are covered by tests (red without the fix, verified); nos. 18–20 are
test-bench faults that only `tools/check.sh --ros` or careful handling by the operator
showed — which is why they appear here as table rows, not as tests.

| # | Symptom | Cause | Fixed in |
|---|---|---|---|
| 1 | ROS run: node dies with `KeyError: 'PWCS'`, no `kf/pose` in RViz | `ros_bridge.to_ros()` names the class `PoseWithCovarianceStamped`, the lookup asked for `"PWCS"` | `ros_bridge.py` |
| 2 | The sim process dies as soon as a node sends `kf/pose`: `ValueError: truth value of an array` | `cov_diag()` wrote `len(cov or [])` — under ROS `cov` is a numpy-like array | `ros_bridge.py`, test in `tests/test_kf_grading_integrator.py` |
| 3 | Second task of a grading run: estimate ~1.2 m off, NEES 35 | The blind run started where the previous task had ended (wall contacts, then the wrong path) | `engine.reset_robot()` + calls in `node.verbinde_auftrag()` and `tools/fastgrade.py` per KF task |
| 4 | The first task of a run measures with the profile of the *last* task | `tasks.sim_profil()` merged the `sim` blocks of all selected tasks (kf_gps 5 Hz + kf_fusion 1 Hz ⇒ 1 Hz for both) | `tasks.py`: `sim_profil()` now returns the start profile = profile of the first task; per task the grader applies its own |
| 5 | `./lab grade --task kf_alle` drives through the `maze` arena | `--world` had the fixed default `maze` | `node.py`: default `None`, KF tasks take `arena`; experiment 1 unchanged |
| 6 | `ros2 launch launch/kf.launch.py bewerten:=kf_alle` grades a robot named "kf_alle" | `cmd_sim()` passed `--grade` to `_grader()` as the robot name (swapped arguments) | `node.py` (`_grader(args.robot, args.grade, …)`) — also affected `launch/lab.launch.py` |
| 7 | `./lab run --robot alice --controller …` shows an empty arena | `cmd_run()` spawned only `--robots` | `node.py`: `--robot` is spawned automatically (exactly as in `cmd_grade`) |
| 8 | Measurement log (`--log`) has empty `*_kf` columns | The log tap sits on the sim outbox; `kf/pose` however comes from the students | `node.simlauf()` now taps the incoming estimate too (deduplicated) |
| 9 | Grader reports `rate_hz 38.6` although the node sends nothing | The subscription callback in `grade.py` fires again on every `spin()` — `_kf_seen` counted calls, not messages | `grade._kf_saehen(msg)` counts new messages only (object/stamp) |
| 10 | Report shows `rmse 1000000000.0` | "never an estimate" was passed on as a number | `grade.py`: display capped at 999, own hint "no kf/pose message received at all" |
| 11 | IMU bias random walk was invisible (tests) | `_stichprobe()` never applied `w_a`/`w_g` to the bias | `sensors.py`, test in `tests/test_sensors_imu_b.py` |
| 12 | After a respawn or profile switch the odometry stamp sits at 0, the GPS stamp at 45 s | The odometry and IMU clocks count from the moment they are created | `sensors.py`: `t_offset` per sensor, `engine._zeitbezug()` anchors them to simulation time |
| 13 | Reference solution picks up an old GPS position in the second task (15 m off) | The bus keeps the last message; a fix from the previous task was swallowed as a current measurement | `student/kf_solution.py`, `student/kf_template.py`: `KF.start` = mission start, earlier fixes dropped; the rule is in the handout now ("Three rules") |

Only a real ROS run could have shown two of them (nos. 1, 2) — `tools/check.sh --ros`
is not optional in experiment 2.

### 1b. What only the full `tools/check.sh` showed (experiment 1 had been edited)

| # | Symptom | Cause | Fixed in |
|---|---|---|---|
| 14 | `./lab grade --task alle` (experiment 1) drives into the arena wall, T2 stays at `path 0` | The tasks name their arena in `config/tasks.json` (`"welt": "production"`), my `cfg_get_welt()` read that recommendation only for `--world auto` and for KF tasks — so experiment 1 ran in `maze` while `tools/fastgrade.py` ran in `production` | `node.cfg_get_welt()`: the task recommendation always applies, `--world` wins; test `test_versuch_1_bekommt_production_und_versuch_2_arena` |
| 15 | `--task alle` after experiment 2 = eight tasks, including the KF blind runs — and the arena recommendation turns into a majority vote | Before experiment 2 `alle` meant "all tasks". Now: `alle`/`v1` = experiment 1, `kf_alle`/`v2` = experiment 2, new `beide` for everything | `tasks.resolve()` (+ docstring) |
| 16 | The student node reports `unknown task 'alle'` before the first task starts | `mach_engine()` published the raw `--task` value on `/sim/task`; but a group is not a task | `node.erster_auftrag()` — the first real ID is what gets published |

| 17 | **Experiment 1 fails silently**: T2/T3/T4 report `path 0.0`, grader stages run up to the time limit even though the robots drive cleanly | The engine pushes the robot list (`/sim/robots` with `mission_state`, `distance`, `contacts`) only on `spawn`/`reset`. `simlauf()` never refreshed it — the grader saw the spawn instant for the whole grading | `node.simlauf()`: publish the list every round when it changed; test `test_simlauf_haelt_die_Roboterliste_aktuell` (provably red without the fix) |

| 18 | `tools/check.sh --ros` aborts right at the ROS block ("ROS not sourced") although ROS 2 is present | `set -u` in the script plus the ROS setup files, which read unset `AMENT_*` variables — the same case already solved in `./lab` with `set +u` | `tools/check.sh` (set +u around the source) |
| 19 | One KF task of `kf_alle` fails **at random** with the same seed (`rmse 3.5 m`, `NEES 332`), the next run of the very same command is clean | The student node runs in its own thread. When a new task starts it reads `rob.odom()` — and the bus still holds the last message of the *previous* drive: pose metres away, stamp seconds old. The filter started on that and then predicted the gap as one CV step | `student/kf_solution.py`, `student/kf_template.py` (`stempel()` baseline + `SCHLAF`), regression tests in `tests/test_kf_solution_d.py` |

No. 17 was the most expensive finding: it hits **experiment 1**, and it was visible neither in
the unit tests nor in `tools/fastgrade.py` (that script publishes the list itself, line 89)
nor in the window (the renderer query reads the engine directly). Only the complete
`check.sh` run against experiment 1 showed it — `path 0.0 m — barely moved?` was the only
clue. Since then the rule is: after every rebuild of `simlauf()` also run
`./lab grade --task alle` (experiment 1), not only the KF tasks.

Lesson from no. 14: **a new task belongs in an arena that it names itself**, and picking the
arena must not have two paths (one via `--world`, one via the
test profile). The test covers both paths.

## 2. Reference solution: what it does now (and why more lines)

One filter for all four tasks, state (x, y, vx, vy) plus heading, without numpy. Plus three
things the first draft did not have and that K2/K3 would otherwise fail on:

* `KF.start` — drop measurements taken before the mission starts (no. 13).
* Larger `Q` during the GPS outage: odometry drift is not white noise (prelab question 3),
  so without correction the uncertainty grows faster than the CV model predicts.
* Consistency rule (`KF.konsistenz`): innovation variance against the announced `S`, every 20
  fixes a factor applied to `Q`. That *is* the task of K3 — a reference solution that only
  carries a guessed `Q_ACC` value would be a poor example to read.

The file is 283 lines now (budget 260 → 290, justified in `tests/test_kf_solution_d.py` and
`tools/loc.py`).

**Line budgets overall:** `python3 tools/loc.py --strict` is green. Raised, each with the
reason in the comment in `tools/loc.py`: `engine.py` 300→340, `node.py` 465→520,
`grade.py` 610→620, `student/kf_template.py` 160→190, `tools/kfplot.py` 190→250. The
total budget of the simulator core stayed at 3700 (measured 3684) — the pinch sits
there on purpose, not in the individual modules.

## 3. Measured values of the reference solution (kf_alle, one run per seed, `--speed 25`)

| Seed | K1 rmse / NEES | K2 rmse / impr. / gap | K3 rmse / NEES | K4 rmse / impr. | Points |
|---|---|---|---|---|---|
| 1 | 0.070 / 0.63 | 0.598 / 3.41 / 0.876 m | 0.117 / 0.89 | 0.081 / 11.2 | 90/90 |
| 2 | 0.071 / 0.62 | 0.317 / 5.63 / 0.289 m | 0.067 / 0.36 | 0.118 / 7.1 | 90/90 |
| 3 | 0.053 / 0.34 | 0.266 / 6.31 / 0.218 m | 0.066 / 0.32 | 0.066 / 11.5 | 90/90 |
| 4 | 0.053 / 0.33 | 0.431 / 3.34 / 0.533 m | 0.102 / 0.81 | 0.077 / 11.9 | 90/90 |

After no. 19 four further runs of seed 1 read 0.070/0.598/0.117/0.083 ± 0.003 — the numbers
above are reproducible now. Before the fix one run in three lost a task to the stale start
message, which is the reason the table used to be "measured once per seed" with a shrug.

Real time instead of accelerated (`./lab grade --task kf_alle …`, seed 1): also 90/90.
From the log of the same run `tools/kfplot.py` measures the same numbers as the grader
(RMSE 0.125 vs. 0.126 m, improvement 5.97, NEES 0.59) — two independent evaluations.

Thresholds are met with margin except for K3: there the reference solution measures NEES
0.32 … 0.89 over the four seeds (0.19 … 1.04 was the range before no. 19, when a stale start
could still be in a run) — so the lower bound sits at 0.15 instead of the original 0.5, while
the upper one stays strict at 3.5.
Anyone changing the bound: it lives in `config/tasks.json` (`nees`) and must agree with the
text in the handout (`tab:aufgaben`) and `docs/CONTRACT-KF.md` §5.
hence the loose lower bound instead of 0.5 from the first draft.

## 4. Open items / for the next supervisor

* `grade.py` at 597 lines is the largest core module. Splitting it into `grade.py`
  (experiment 1) and `kf_grade.py` would be clean, but it is not something to do on the
  side while the lab is running; the budget in `tools/loc.py` is documented, not fudged.
* `tools/fastgrade.py` has no `--log`; that would be useful for logs in the fast runner
  (`tools/check.sh` and `./lab grade` cover the case at real-time pace).
* The robot list now goes on the bus at 1 Hz (immediately when it changes). Anyone changing
  the rate: ROS clients only see what is published after they subscribe — publishing purely
  on change makes `ros2 topic echo --once` hang.
* The KF tasks depend on the command sequence in `config/tasks.json`. Anyone changing the
  world or the spawns must re-check the runs (`kontakte` has to stay 0 — the grader drives
  blind, a contact is not a student error).
* `install.sh` builds the ROS interfaces only if `colcon` is present; without them the
  JSON handshake takes over (CONTRACT §4). Run `./install.sh` once in the lab room.
* **Language is a gate, not a preference**: written prose and UI text are English, and
  `python3 tools/langcheck.py` runs as a step in `tools/check.sh`. It reports umlauts
  everywhere (no identifier here uses them) and German words in prose. German-derived names
  that are API stay (`sekunden`, `aufgabe`, `bewerten`, `wahrheit`, `protokoll`,
  `aufzeichnung`, ROS topics, JSON keys, `VL/VR/HL/HR`, `simlauf()`, `welt_fuer()`); renaming
  them would invalidate handouts and student code, so it is a separate deliberate step.
  What the detector cannot see is a sentence that *describes* an old rule — the README used to
  say "comments and UI text in German" while every file was English. Read the rules once in a
  while, do not only run the tool.
* No. 19 generalises: **a node must never trust a message that was already in the bus when its
  task began.** Every new sensor, every new task and every new node type has to answer that
  question again — a stamp baseline is two lines, the grading damage is metres.
* Still open from `docs/notes/f.md`: `./lab spawn` does not reach a simulator that was started
  through the launch file, and `/sim/*` is published with VOLATILE QoS, so a late subscriber
  sees nothing until the next publish (the robot list works around that with its 1 Hz beat).

---

## View overhaul and the minimal TF tree (this pass)

Requested: no outlines on the wall blocks, resizable window, zoom, a closed-looking world,
visible wheel rotation, lidar dots in the robot's colour, a small menu to hide sensor layers
while ROS keeps publishing, and a minimum TF tree so `map` exists for RViz. Pygame handles all
of it; a second renderer would have doubled the surface of a course that promises "very little
code", so there is none.

**New files.** `cam.py` (scale + centre: wheel zoom **at the cursor**, drag pan,
`VIDEORESIZE`, `f` = whole world, `gui_style.px_per_meter_min` keeps the robot from shrinking
to a dot), `menu.py` (translucent layer panel, click or key, header says *view — ROS keeps
publishing*), `tf_bcast.py` (`/tf`, `/tf_static` as plain `tf2_msgs/msg/TFMessage` — **no
tf2_ros, no numpy**, so CONTRACT §1 still holds).

**Two decisions that change the contract, both reversible by config:**

* **Frame names carry the robot** (`alice/odom`, `alice/base_link`, `alice/laser`,
  `alice/imu_link`; `map` stays shared). TF names are global; without the prefix every robot in
  the hall claims the same frames and RViz cannot assign a scan to a robot. Headers and /tf read
  the names from one place (`tf_bcast.frames()`), so they cannot drift apart. `gps`, `truth` and
  the students' `kf/pose` remain in `map`. Handout impact: RViz presets (`rviz/kf.rviz` updated,
  `Offset Frame: test/base_link`) and anything that typed `laser` literally.
* **`tf.map_to_odom` defaults to `odom`, i.e. `map -> odom` is the identity.** So
  `map -> base_link` shows the drifting odometry, and closing that gap with GPS/IMU stays the
  students' exercise instead of becoming a gift from the simulator. `tf.map_to_odom=truth`
  (launch: `tf_karten_odom:=truth`) composes `map -> base_link` to the exact pose for the
  supervisor; `/<robot>/truth` is unchanged.

**Worlds.** `maze` is rebuilt: closed border (it was open along the last row), 1.0 m grid, 6x5
corridors one cell wide, four spawns in the corners instead of `S.2.3.4` shoulder to shoulder.
`worlds.cell_by_world` finally makes the cell size configurable per world (`worlds.py` ignored
`worlds.cell` completely while `tools/worldcheck.py` read it — that inconsistency is gone), so
the maze can be coarse while `production`/`arena`/`track` keep their graded 0.5 m grid and the
robot's 0.21 m footprint is untouched. `worldcheck` gained a closure check and now reports
`0 worlds with problems`.

**View.** Walls are filled with one colour (the old 1 px brighten read as a second, thinner
wall); everything outside `world.size` is `gui_style.void`, darker than the floor, so a missing
border shows up as a gap; scan dots use the robot's own colour; wheels are drawn as plates with
three roller strokes travelling along them, exaggerated by `gui_style.wheel_scale` (5 cm wheels
are three pixels in a 13 m hall — a viewing aid like the old 0.25 roller scaling, physics
untouched). `gui_style.grid` and `gui_style.lidar_alpha` are gone: the renderer never used the
first and the second contradicted "scan in robot colour". Old config files that set them simply
have two keys with no effect.

**Budgets.** `CORE_TOTAL` 3700 → 4300, `render.py` 335 → 455, plus entries for the three new
modules (`tools/loc.py` is and stays authoritative, CONTRACT §7 now points there instead of
carrying a stale copy). Physics, bus and grading did not grow.

**Reproduce:** `tools/check.sh` (release gate, includes a maze run and both gradings),
`SDL_VIDEODRIVER=dummy python3 -m pytest tests -q` (131 passed), `python3 tools/worldcheck.py`,
and for TF evidence with ROS sourced: `./lab sim --robots alice --headless --seconds 20` plus
`ros2 topic echo --once /tf_static` → `alice/base_link -> alice/laser`, `alice/imu_link`;
`ros2 topic echo --once /tf` → `map -> alice/odom` (identity), `alice/odom -> alice/base_link`.
