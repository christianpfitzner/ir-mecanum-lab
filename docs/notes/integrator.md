# Integration note for experiment 2 (Kalman filter) — what the integrator changed

Status: after the latest calibration run. This note extends `docs/CONTRACT-KF.md` and
justifies every deviation from the first draft. Order: first the findings (faults that the
integration test showed), then the thresholds, then the open remainder.

## The English sweep (this pass)

Every identifier is now English (`tools/germanids.py` guards names the way `langcheck.py` guards
prose), and so are the arguments on the command line and in the launch files. The old German
spellings still work where a printed handout uses them: `launch/kf.launch.py` and
`launch/sim.launch.py` translate the argument and print
`deprecated launch argument 'sekunden', use 'seconds'`, `tasks.py` maps the keys of an old
`config/tasks.json` through `_LEGACY_KEYS` with one warning per key, and `./lab --log-intervall`
behaves the same way. CONTRACT §6.11 holds both tables; new code always uses the English name.

Two things fell out of the sweep besides the renames. A grading report and the CSV log now use
English field names with **no** alias — students read those two with their own scripts, and a
renamed column is a visible, immediate error rather than a silently missing value. And
`ros2 launch launch/kf.launch.py` without arguments no longer dies: the old `LogInfo` line read the
`welt` configuration before it was declared, so ROS raised `SubstitutionFailure` before a single
node had started.

## 1. Faults found and fixed

Nos. 1–13 and 14–17 are covered by tests (red without the fix, verified); nos. 18–20 are
test-bench faults that only `tools/check.sh --ros` or careful handling by the operator
showed — which is why they appear here as table rows, not as tests.

| # | Symptom | Cause | Fixed in |
|---|---|---|---|
| 1 | ROS run: node dies with `KeyError: 'PWCS'`, no `kf/pose` in RViz | `ros_bridge.to_ros()` names the class `PoseWithCovarianceStamped`, the lookup asked for `"PWCS"` | `ros_bridge.py` |
| 2 | The sim process dies as soon as a node sends `kf/pose`: `ValueError: truth value of an array` | `cov_diag()` wrote `len(cov or [])` — under ROS `cov` is a numpy-like array | `ros_bridge.py`, test in `tests/test_kf_grading_integrator.py` |
| 3 | Second task of a grading run: estimate ~1.2 m off, NEES 35 | The blind run started where the previous task had ended (wall contacts, then the wrong path) | `engine.reset_robot()` + calls in `node.wire_task()` and `tools/fastgrade.py` per KF task |
| 4 | The first task of a run measures with the profile of the *last* task | `tasks.sim_profile()` merged the `sim` blocks of all selected tasks (kf_gps 5 Hz + kf_fusion 1 Hz ⇒ 1 Hz for both) | `tasks.py`: `sim_profile()` now returns the start profile = profile of the first task; per task the grader applies its own |
| 5 | `./lab grade --task kf_alle` drives through the `maze` arena | `--world` had the fixed default `maze` | `node.py`: default `None`, KF tasks take `arena`; experiment 1 unchanged |
| 6 | `ros2 launch launch/kf.launch.py grade:=kf_alle` grades a robot named "kf_alle" | `cmd_sim()` passed `--grade` to `_grader()` as the robot name (swapped arguments) | `node.py` (`_grader(args.robot, args.grade, …)`) — also affected `launch/lab.launch.py` |
| 7 | `./lab run --robot alice --controller …` shows an empty arena | `cmd_run()` spawned only `--robots` | `node.py`: `--robot` is spawned automatically (exactly as in `cmd_grade`) |
| 8 | Measurement log (`--log`) has empty `*_kf` columns | The log tap sits on the sim outbox; `kf/pose` however comes from the students | `node.run_loop()` now taps the incoming estimate too (deduplicated) |
| 9 | Grader reports `rate_hz 38.6` although the node sends nothing | The subscription callback in `grade.py` fires again on every `spin()` — `_kf_seen` counted calls, not messages | `grade._kf_saehen(msg)` counts new messages only (object/stamp) |
| 10 | Report shows `rmse 1000000000.0` | "never an estimate" was passed on as a number | `grade.py`: display capped at 999, own hint "no kf/pose message received at all" |
| 11 | IMU bias random walk was invisible (tests) | `_one_sample()` never applied `w_a`/`w_g` to the bias | `sensors.py`, test in `tests/test_sensors_imu_b.py` |
| 12 | After a respawn or profile switch the odometry stamp sits at 0, the GPS stamp at 45 s | The odometry and IMU clocks count from the moment they are created | `sensors.py`: `t_offset` per sensor, `engine._clock_to_sim()` anchors them to simulation time |
| 13 | Reference solution picks up an old GPS position in the second task (15 m off) | The bus keeps the last message; a fix from the previous task was swallowed as a current measurement | `student/kf_solution.py`, `student/kf_template.py`: `KF.start` = mission start, earlier fixes dropped; the rule is in the handout now ("Three rules") |

Only a real ROS run could have shown two of them (nos. 1, 2) — `tools/check.sh --ros`
is not optional in experiment 2.

### 1b. What only the full `tools/check.sh` showed (experiment 1 had been edited)

| # | Symptom | Cause | Fixed in |
|---|---|---|---|
| 14 | `./lab grade --task alle` (experiment 1) drives into the arena wall, T2 stays at `path 0` | The tasks name their arena in `config/tasks.json` (`"world": "production"`), my `cfg_get_world()` read that recommendation only for `--world auto` and for KF tasks — so experiment 1 ran in `maze` while `tools/fastgrade.py` ran in `production` | `node.cfg_get_world()`: the task recommendation always applies, `--world` wins; test `test_task_1_gets_production_and_task_2_arena` |
| 15 | `--task alle` after experiment 2 = eight tasks, including the KF blind runs — and the arena recommendation turns into a majority vote | Before experiment 2 `alle` meant "all tasks". Now: `alle`/`v1` = experiment 1, `kf_alle`/`v2` = experiment 2, new `beide` for everything | `tasks.resolve()` (+ docstring) |
| 16 | The student node reports `unknown task 'alle'` before the first task starts | `make_engine()` published the raw `--task` value on `/sim/task`; but a group is not a task | `node.first_task()` — the first real ID is what gets published |

| 17 | **Experiment 1 fails silently**: T2/T3/T4 report `path 0.0`, grader stages run up to the time limit even though the robots drive cleanly | The engine pushes the robot list (`/sim/robots` with `mission_state`, `distance`, `contacts`) only on `spawn`/`reset`. `run_loop()` never refreshed it — the grader saw the spawn instant for the whole grading | `node.run_loop()`: publish the list every round when it changed; test `test_simlauf_haelt_die_Roboterliste_aktuell` (provably red without the fix) |

| 18 | `tools/check.sh --ros` aborts right at the ROS block ("ROS not sourced") although ROS 2 is present | `set -u` in the script plus the ROS setup files, which read unset `AMENT_*` variables — the same case already solved in `./lab` with `set +u` | `tools/check.sh` (set +u around the source) |
| 19 | One KF task of `kf_alle` fails **at random** with the same seed (`rmse 3.5 m`, `NEES 332`), the next run of the very same command is clean | The student node runs in its own thread. When a new task starts it reads `rob.odom()` — and the bus still holds the last message of the *previous* drive: pose metres away, stamp seconds old. The filter started on that and then predicted the gap as one CV step | `student/kf_solution.py`, `student/kf_template.py` (`stamp()` baseline + `SCHLAF`), regression tests in `tests/test_kf_solution_d.py` |

No. 17 was the most expensive finding: it hits **experiment 1**, and it was visible neither in
the unit tests nor in `tools/fastgrade.py` (that script publishes the list itself, line 89)
nor in the window (the renderer query reads the engine directly). Only the complete
`check.sh` run against experiment 1 showed it — `path 0.0 m — barely moved?` was the only
clue. Since then the rule is: after every rebuild of `run_loop()` also run
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
  world or the spawns must re-check the runs (`contacts` has to stay 0 — the grader drives
  blind, a contact is not a student error).
* `install.sh` builds the ROS interfaces only if `colcon` is present; without them the
  JSON handshake takes over (CONTRACT §4). Run `./install.sh` once in the lab room.
* **Language is a gate, not a preference**: everything written and everything named is English,
  and two tools run as steps in `tools/check.sh`. `python3 tools/langcheck.py` reads the prose:
  it reports umlauts everywhere (no identifier here uses them) and German words in prose.
  `python3 tools/germanids.py` reads the names: it parses every Python file and reports German
  identifiers, German data keys and non-ASCII identifiers, and fails on anything outside its
  allowlist (ROS names, `VL/VR/HL/HR`, the CLI task groups, and the German values of
  `tasks._LEGACY_KEYS` and the launch `DEPRECATED` tables, which it reads from the code itself).
  What neither detector can see is a sentence that *describes* an old rule — the README used to
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

## 5. Repo hygiene (`.gitignore`, and why the PDFs stay)

62 `.pyc` files and twelve LaTeX by-products (`.aux`, `.fls`, `.fdb_latexmk`, `.log`, `.out`,
`.toc`) were tracked, so every clone carried the previous author's byte-code and every
`compileall` run showed up as a diff. `.gitignore` now covers byte-code and test caches,
`setup.py`/colcon output (`build/`, `dist/`, `*.egg-info/`, `/install/`, `/log/`), LaTeX
by-products, the output the tools write when asked (`--json bericht.json`, `--log messung.csv`,
`kfplot -o bild.png`), virtualenvs and editor litter. `tools/check.sh` fails if a byte-code or
LaTeX by-product appears in the index again.

**The two handout PDFs stay in git on purpose** — a clone must be printable without LaTeX; do
not add `*.pdf` to `.gitignore`. `.gitignore` deliberately does not ignore `docs/img/`: that
figure is a generated artefact that is meant to be shipped (worldpic draws it, the check proves
it still draws).

**Reproduce:** `git ls-files | grep -E '\.(py[cod]|aux|fls|log|toc|out)$'` must print nothing,
and `git status` must stay clean after `python3 -m compileall -q mecanum_lab && python3 -m pytest
tests -q`.

## 6. Wheel slip, keyboard turning, and the IMU in the readout (after the arena picture)

Someone driving by hand could not see odometry drift: at an obstacle the old chassis set the
wheel speeds to **zero**, so the odometry — which integrates wheel speeds, as it must — stayed
honest exactly where a real mecanum robot lies. `physics.py` now models that one situation: the
body velocity goes to 0, the wheels keep `robot.slip` × the commanded speed (default 1, `0` is
the old ideal static friction). Measured with seed 5, 4 s of full throttle into the east wall of
`arena`: the body moves 0.69 m, the odometry counts 1.55 m — **0.86 m of phantom distance**, and
0.003 m with `robot.slip=0`. Tests: `test_slip_at_an_obstacle_fools_the_odometry_a` (both
settings) and `test_slip_value_comes_from_the_config_a` (the knob really reaches
`Geometry`). Grading is unaffected: the reference solution drives without contacts, so it never
enters the slip branch.

Turning by keyboard existed but was undiscoverable, and one of its two keys killed the window:
`q` is both "yaw left" in `teleop_keys()` and "quit" in the renderer. Yaw is now `q`/`,` and `e`/`.`
(±0.9 rad/s, `e` = counter-clockwise = left), the renderer only quits on `q` when teleop is off
(`ESC` always works), and while teleop is on the HUD header prints the driving keys instead of
the view keys.

The IMU was already published at `imu.rate` (100 Hz by default, in every mode — 99.3 Hz measured
over 3 s) but was invisible without ROS. The per-robot readout line now shows `ax`, `ay`, `gz`
(`--set imu.rate=0` prints "no imu"), which is also what `messung.csv` records as
`ax_imu, ay_imu, gz_imu`.

Budget: `render.py` 455 → 470 (readout field, teleop help line, `teleop` state), core total
4197/4300. `tools/loc.py` and CONTRACT §7 carry the new number.

## 7. GPS shadow zones, three overlays, and four findings from reading the code

`gps.zones` (default `[]`) degrades the fix by **place**: first rectangle containing the robot
multiplies `sigma_xy`, adds a bias (multipath) or blocks the fix. Measured with 400 fixes per
spot: open floor σ 0.06 m, under the shelf σ 0.37 m and mean offset (+0.83, −0.51) m, in the
multipath corner σ 0.18 m/−0.39 m, in the dock **no fix at all**. It ships empty and the demo
lives in `config/demo_gps_shadow.json`, because the graded tasks are calibrated on the plain GPS
of their experiment. `overlays.py` (new) draws the shadow, the odometry ghost with the drift in
metres (`o`) and the rubber that slipping wheels leave behind; two new view layers, no physics.

**Four things the code review turned up, all fixed and tested:**

1. `tasks.world_for` chose the arena with `max(set(hints), key=hints.count)`. Set order follows
   the string hash, and CPython randomises that per process — measured: `--task beide` picked
   `arena` under `PYTHONHASHSEED=0,6,7` and `production` under `1…5`. The same seed graded a
   different hall. Now the tie goes to the first task, as the docstring always promised.
2. `types.Scan` promised "clockwise from front-left" while the lidar, the helper
   `lateral_distance` and CONTRACT §5 use beam 0 forward, counter-clockwise. That is the central
   sign convention of experiment 1 and it was written down backwards in the file students read.
3. The GPS cross was drawn inside `_schaetzung`, which the frame loop calls only for the estimate
   layer — so the menu row and key `g` did nothing as soon as `k` was off. Now its own method.
4. `./lab teleop` — documented in CONTRACT §6.9 and in the handout's first exercise — did not
   exist; students got `lab: error: unrecognized arguments: teleop`. It is now `sim` with one
   robot and the keyboard on.

Two more were found while making the picture: `spawn()` never reset the odometer, so a robot put
down at (7.5, 3.5) reported its own pose as (0, 0) plus travel (the drift overlay showed 22 m of
error while it drove a straight line), and `Robot.pose` kept the dataclass default `(0,0)` until
the first physics step — enough to streak a trail across the whole hall. Both are set at spawn now
and covered by tests.

**Still open (from the same review, none of them fixed here):** the simulation clock comes from
`time.monotonic()` and the 0.5 s accumulator clamp throws whole seconds away silently, and the
grader is fed that wall-clock `dt` (CONTRACT §1 says: never wall clock); `OdometrySensor` receives
the *true* `Geometry`, so a wrong wheel radius or lever arm — the biggest real odometry error
source — cannot be expressed; `kf.gps_delay` and `lidar.max_walls` are config keys nothing reads;
`_merge` deletes a key when an override is `None`, which is how `{"world": None}` wipes the
`world` from `config/default.json`.

## 8. The grader now runs on the simulation clock

`run_loop` used to hand the grader the wall-clock delta, so `Grader.tick()` — whose docstring says
"advance one **simulation** step" — advanced on the wall: pausing the window kept the student's
task clock running, and a scheduling hiccup shifted the whole drive plan relative to the
simulator. It now receives `eng.t - sim_t`, the time the simulator actually advanced, so
grading stays consistent with what the robot experienced, and pausing pauses both.

That also exposed the last piece of randomness in KF grading: K3's NEES floor (now 0.05, with the
measured spread written into `config/tasks.json` and CONTRACT-KF §5). The remaining fix — a
fixed-step grading run that does not pace on the wall clock at all — is still open, and would need
every KF threshold re-measured, because `rate_hz` is measured against sim time.

## 9. The four loose ends from §7 are closed — and one of them turned out to be a wall

**The pace of a run is a switch now.** `run_loop` took `min(wall clock delta, 0.25)` as its dt and
`SimEngine.step()` swallowed everything above a 0.5 s accumulator; both lost whole seconds without
a word. They count what they dropped (`SimEngine.dropped`, and the same figure inside the loop) and
say it once: `sim time fell behind the wall clock by 1.7 s — use --speed or --fixed-step`. The new
`--speed N` (simulation seconds per wall second) and `--fixed-step` (exactly 1/`rate`, no sleeping)
are on `run`/`sim`/`grade`; `tools/check.sh` grades experiment 1 a second time at `--speed 4`.
Measured, three runs per pace, seed 1 (tables in CONTRACT §9.1 and CONTRACT-KF §5.1): T4's target
error 0.094…0.147 m realtime, 0.158…0.250 m at 4×, 0.087…0.260 m at fixed-step, against a limit
of 0.45 m — **no threshold had to move**, 100/100 and 90/90 everywhere it can be graded. The loss
warning itself stayed silent in every one of those runs, even with 20 busy processes pinned to the
simulation's core: what varied here was timing granularity, not a stall over the per-round make-up
limit, so the warning is pinned by a test with a scripted clock rather than by a host. Two findings
worth keeping: `--speed 4` measures *tighter* than real time (K2 rmse 0.253…0.506 → 0.273…0.372),
and T4's spread does not disappear at fixed-step, because only the simulation left the wall clock —
the student node still samples it in its own thread.

**Experiment 2 cannot be graded at `--fixed-step`, and that is not a threshold problem.** A node
reports `kf/pose` at most once per loop iteration, so its rate in *simulation* seconds is
(iterations per wall second) ÷ (sim seconds per wall second): 2.6…4.0 Hz at fixed-step (~36× here)
against `rate_min` 5 and 10 Hz, and the estimate degrades with it (K4 rmse 0.517…0.778 vs 0.35).
Measured further: `--speed 6` already gives 9.1…9.6 Hz, `--speed 8` gives 7.0 Hz — K4 falls over at
both. Loosening `rate_min` would delete the only check that a node publishes continuously instead of
once per GPS fix, so the thresholds stay and the *pace* is what gets restricted: experiment 2 up to
about 4× realtime, `--fixed-step` never. §8's "would need every KF threshold re-measured" is
therefore answered — re-measured, and the answer is that this pace is not a grading pace.

**`odom.geometry`: the odometry integrator finally believes its own wheel constants.** Before,
`SimEngine` handed `OdometrySensor` the chassis `Geometry`, so a wrong radius or lever arm — the
commonest real odometry error, and the one T2 exists to teach — could not be expressed.
`sensors.odom_geometry()` builds the separate one (`wheel_radius_scale` scales the whole path and,
because the radius sits in the turn equation too, the yaw rate; `lever_scale` scales a = lx+ly, so a
commanded circle ends rotated; `wheel_base_scale` mis-measures lx only; `scale_xy` and `bias_xy` are
applied by the sensor). Empty by default, tested to stay identical, demo in
`config/demo_odom_error.json`: 0.61 m of ghost drift on 12 m of straight lane, one revolution
finished 28° rotated, and the graded run down at 70/100 with T3 collecting 26 wall contacts.

**The dead keys are alive.** `gps.delay_ticks` holds each fix in a ring buffer and delivers it N
emissions later, keeping the stamp it was *generated* with (`SimEngine._push(..., stamp=)` exists for
that, and `reset()` empties the buffer) — `kf.gps_delay` is the same delay in seconds, because that
is the name the handout gives the students and a recommendation nothing simulates is a wish.
`lidar.max_walls` caps the segments one scan may use, in world order; the default 400 is above the
wall count of every world, which a test now pins.

**`_merge` no longer deletes on `None`.** The CLI hands the config layers one tree built from its
arguments, and an argument that was not given is `None` — so `{"world": None}` erased the `world` of
`config/default.json`, and a task profile had to write `"gap": null` to switch an outage off. `None`
now means "nothing overridden" and the off-spelling is the empty list, which is what
`config/tasks.json` says for the three tasks after `kf_fusion`. `cfg["gui"]` is read too: a config
file with `"gui": false` gets no window, `--headless` stays the stronger voice.

## 10. A sensor now says how good the measurement was (this pass)

**`quality`/`sats`, and the difference between "no fix" and "no radio".** Every message says what it
is worth: `Gps.quality` 2 good, 1 degraded, 0 no fix, plus the anchors it came from. Both come from
`GpsSensor.sky(pose)`, which needs the position and nothing else — no random number — so the window
can ask for it every frame and a test can write down the answer without a seed. That indirection is
the point: while a robot sits in a blackout there is no fresh message to read the quality out of, and
`engine.gps_health()` therefore reports the receiver's opinion rather than the last fix's. Measured on
one straight drive through `config/demo_sensor_reality.json`: q2/8 anchors on open floor, q1/3
between the racks from x = 8.0 m, q0/0 in the dock from x = 16.6 m, with 27 dropped messages in
between. One consequence worth writing down: quality 0 never rides on a message that this simulator
sends. A receiver without a solution has no position to send, so `fix()` answers `None` for it — as it
always did for `block` zones and for the gap window — and the two faults are told apart by the
readout (`q0 0 sats` for the place, `lost 7` for the radio), not by a third message kind.

**Dropouts are a property of the seed.** `gps.dropout` tosses its coin **before** anything is
measured, so which emissions are missing does not depend on where the robot happened to be, and the
pattern is reproducible with `--seed` and in a test (same seed: the same 120 holes; seed 7 → 27 of
120, seed 8 → 30). `gps.latency` moves the stamp and never the value; the delayed series is literally
the head of the undelayed one, which is how the test says it. Measured 0.25 s of wire: a fix arrives
0.45 s after it was taken (the wire plus the wait for the next emission slot), never older than 1.0 s.
One finding while wiring this: counting "emissions that brought nothing" is *not* the number of lost
messages, because an asynchronous wire also has slots with nothing due — the counter lives in the
receiver and counts dropouts per robot, which is the number the readout is allowed to call `lost`.

**A LIDAR sees echoes, not walls.** `_ray_rect()` answers `(distance, face)` now, because whether a
surface returns anything depends on its orientation and the slab test is the only place that knows
which pair of faces the ray crossed; `Lidar._reflects()` then compares |cos| of the incidence with
`lidar.reflectivity_min`. Measured 0.5 m off a 30 m wall: the wall is seen 7.17 m down its length
with the default and 1.93 m at 0.25, 20 of the 360 beams come back empty, and the beam pointing
straight at the wall at 0.500 m is bit for bit unchanged. What is *not* modelled is the "shorter
reading" half of a grazing echo — that needs an intensity per surface, i.e. a second knob that only
means something together with the first; the absent reading is the one that makes real lidars miss
painted posts, so that is what the switch does. `Scan.missing` counts the empty beams (`inf` in
`ranges` is not enough: a clipped reading looks exactly like a wall at `range_max`).

**`odom.jitter` is a stamp, not a rate.** Jittering the publish period was tried first and measured:
at the default 50 Hz the accumulator self-corrected (4 late messages out of 500 — no jitter to see),
and at 20 Hz publishing whenever the accumulator reaches a *variable* period biased the rate upward by
11 % (886 messages instead of 799). Both are worse than what the knob is for. It now stamps each
message late by up to `jitter` of a period, drawn one-sided with `Noise.late` — never early, so stamps
cannot overtake and no filter has to cope with a negative `dt`. Measured: 13.3…26.8 ms between stamps
instead of exactly 20.0 ms, σ 2.8 ms, message count and values identical.

**The IMU warms up and its bias follows.** First-order lag towards `temp_start` + `temp_motor` at
full drive, and the bias moves with the temperature (`temp_walk`, `temp_walk_gyro`) — the curve every
datasheet draws as bias against temperature, and an offset that averaging does not remove. Measured
over 25 s at 0.5 m/s: 24.00 → 29.52 °C, `az` bias +0.024 m/s², `gz` bias +0.00069 rad/s; while
standing the chip stays at 24.00 °C and `az` stays +9.81. The default is a cold chip, because the
graded KF thresholds of experiment 2 are calibrated on a bias that stays where it started. The
temperature itself has no field in `sensor_msgs/Imu`, so over ROS only its effect is visible; the
readout and the `temp_imu` column carry the number.

**Off has to mean off, and it is measured.** `Noise.chance(0.0)` and `Noise.late(0.0)` return without
touching the generator, and every other new term multiplies by 1 or adds 0.0 — so the default config
produces the streams of before, message for message: 2337 measurements of one fixed 12 s drive
compared at full float precision against a copy of the modules from before this pass, identical for
the default config and with `demo_gps_shadow.json` and `demo_odom_error.json` loaded. The digest is a
test now (`tests/test_sensor_reality.py`), so the next person who reads a knob that is switched off
fails loudly instead of moving a graded threshold by a millimetre. Line counts and the reason for
each: CONTRACT §7 and the addendum in `tools/loc.py`; `render.py` came out of this 3 lines shorter.

## 11. A second drive train: `--variant steering` (this pass)

**A new module, not a bigger `physics.py`.** 290 lines of bicycle geometry, rate-limited rack,
Ackermann axle, four rolling wheel speeds and a second integrator — put next to the mecanum equations
instead of inside them, because `physics.py` is what every graded task of experiment 1 runs on and a
student reading `fk(ik(v)) == v` should not have to skip a car to find it. The subclass reuses
`physics.Chassis._collide`, so a car that touches a shelf behaves exactly like the robot the students
know, `steering.slip` included (measured, 30 s of 0.6 m/s into a wall: body 4.38 m, wheels still at
12.0 rad/s, odometry 17.91 m — and with `steering.slip=0` the wheels stand still and the odometry is
honest to 1 cm).

**The engine stopped solving kinematics; the chassis does.** `_drive()` called
`physics.inverse_kinematics(chassis.geometry, …)` for the `cmd_vel` path, which quietly made the
engine a mecanum engine. It calls `chassis.set_twist()` now, and the odometry is handed
`chassis.odo_feed()` instead of `chassis.wheels`: on the car that pair is `(wheel speeds, commanded
rack angle)`, because four wheel speeds cannot carry a steering angle and only the chassis knows which
quantity its integrator needs. `spawn()` and `_make_odometer()` each got one branch. The claim that
nothing else changed is checked two ways: `tests/test_steering_w4.py` drives a `""` robot and a
`"steering"` robot in one engine and asserts the first is still `physics.Chassis` with
`sensors.OdometrySensor`, and `tools/check.sh` still grades 100/100, 90/90 and 30/30.

**What `vy` does, and why that is a warning.** `set_twist` takes `v = vx` and
`delta = atan2(omega * L, max(|v|, 0.05))`; the `vy` that cannot be fulfilled is logged once per robot
(`steering robot cannot strafe, vy=0.25 dropped`) and the car keeps driving. Not an exception and not
a silent drop: a student who sends one strafe component must learn in one sentence that this car has
one degree of freedom, and must not spend the lab period debugging a robot that stands still because
of it. The test counts the log records: one, not two hundred. `set_wheels` — the lab 1 exercise, four
speeds — cannot steer either; it takes their mean as `v` and leaves the rack straight, which is the
legible answer to a student who points the old inverse kinematics at this car.

**The numbers, measured as geometry and not as `v/omega`.** The radius test drives a half turn in a
world without walls and takes the chord between the two poses: demanded 14°/22°/30° give
4.011/2.475/1.732 m against `L/tan(delta)`, ratio 1.0000 each, and 45° still drives `R_min` = 1.600 m.
The rack step response is 16 steps of 1.2° (= 60°/s at 50 Hz) and clamps at the stop. The Ackermann
pair is asserted against `atan(L/(R − W/2))`/`atan(L/(R + W/2))` and the four wheel speeds against the
state, with the identity the model rests on: the mean of the rear pair *is* `v`. The body-frame
lateral component of every one of 600 random steps is below 1e-9 m, and the reported `vy` is exactly
`0.0` — that is the "no sideways motion ever" test, and it would catch a future "improvement" that
lets the car slide.

**The odometry of this car replays what it was told.** `steering.Odometry` integrates `(v, delta)`
with the `delta` the rack was *commanded* to take, rate-limited by the believed rate, because there is
no steering encoder to contradict it. `odom.steer_max_scale` is that belief. Over 20 s at full lock,
the other noise off: 1.0 → 0.0° yaw error and 0.00 m; 1.1 → **+44.9°** and 1.12 m; 0.9 → **−42.0°**
and 1.34 m. Both directions are in the test, next to the honest case, because "it drifts" without the
reference number proves nothing. This is the same teaching shape as `odom.geometry` from §10 — the
integrator believes a model, and the model can be wrong — but here it is a yaw error that no amount of
driving averages out, which is why the handout says it too (`anleitung.tex`, "A second drive train:
the steering car").

**`--variant` was a dead flag.** In `node.py` it was parsed, printed in `--help` and never passed to
`spawn()`: `./lab run --variant slow` has been starting a stock robot the whole time. Three lines
(mine, in a file this package does not own — `make_engine()` and `cmd_run()` now forward it, and the
help text names both drive trains). Worth knowing because the flag is now load-bearing for a variant
that visibly cannot do what the mecanum robot can.

**Two fields the students read, and where they had to go.** `wheels` in `/sim/robots` stays a list of
four speeds — that is what every existing reader of it assumes — so the rack angles arrived as the new
`steer_deg` (two numbers, degrees, empty list on a mecanum robot). The car's geometry travels on
`/sim/config` as the `steering` block, for the same reason the sensor profile does: a node that plans
corners from `R = L/tan(delta_max)` has to ask the simulation which car it is driving, not its own
local copy of the config. Both are `String` topics, so the ROS bridge forwards them unchanged and
needed no change. The view: `overlays.steer_readout()` builds the HUD text (`steer +26.6/+22.9 deg
R=1.90 m`, in wheel-label order, because "inner/outer" swaps sides with the steering direction), and
the wheels themselves are turned by `render._wheel()` from `chassis.wheel_headings` — the mount point
stays on the body, only the stroke turns.

**A node that waits for no task.** `serve()` had two shapes: mission, or `cmd_vel` → inverse
kinematics forever. A drive demo for a car has no kinematics to hand over, so a module that defines
`drive(rob)` gets that called once, with the same `running`/`done`/`failed:<type>` handling a mission
gets, and the pass-through stays switched off afterwards — replaying the demo's last `cmd_vel` through
a kinematics would push a parked car into the shelf in front of it. Every module without `drive` (both
templates, both reference solutions) takes the old path in the same order.

**The example, and the four things it got wrong before it drove.** `student/steering_example.py` is
one idea — point the wheels at the next waypoint with `omega = v · 2 sin(alpha) / distance` — in
214 lines. Measured on the command from the brief: mission `done` after **45.2 s** of the 60 s budget,
**0** wall contacts, 20.3 m driven, final gap to the shelf **0.959 m**, odometry within 5 mm of the
truth. Four findings from getting there, all of them in the file as comments: the lookahead has to be
the *actual* distance to the waypoint (a fixed 0.5 m under-commanded every corner by a decimetre);
the commanded curvature has to be clamped at `tan(delta_max)/L`, otherwise the controller asks freely
and the rack simply sits on its end stop while the log says nothing; the route has to start on the
spawn's own line, because a car that cannot turn on the spot needs "getting onto the route" as a
maneuver; and every plan has to be mapped through the start pose, since the odometry starts at the
spawn and not at (0, 0). The 60 s of the brief's command is enough but not generous — the lap is
20 m at ≤0.75 m/s plus a parking maneuver that approaches by LIDAR.


## 12. An empty hall, and a source the robot can only hear (this pass)

**Two deliverables, one argument.** `worlds/open.txt` is 30 × 20 m of floor with a border wall, two
spawns and nothing else; `mecanum_lab/pois.py` gives a world a radiation source and the robot a counter
for it. The pairing is the point: in a hall with nothing in it, a wrong wheel constant and a sensor that
disagrees with the LIDAR are both *numbers on a screen* instead of a collision that ends the run.

**The world is a grid file, the source is a config key.** The package brief named
`config/worlds/open.json`; this repository has no JSON world format — `worlds/<name>.txt` is an ASCII
grid and `worlds.py` is the only parser, so a second format for one hall would have been a second way to
be a hall. The hall is therefore `worlds/open.txt` (60 × 40 cells, 4 merged border rectangles), and the
sources are the config key `pois`, which is also where they belong: the grid file has no comments and no
room for `activity` or `range`, and validation needs the walls, which the engine has and the parser does
not. `pois` empty (the default) means `SimEngine._make_pois()` returns `None`, so no sensor loop runs at
all and `tests/test_sensor_reality.py` is still byte for byte the pre-W5 stream.

**`kind`, not `type`.** The source entry spells its field `kind` because `config/tasks.json` already
uses that word for the same concept (§6.11 renamed `art` → `kind`); two spellings for one idea is the
mistake that file exists to undo. Everything else of the shipped scenario is as specified: (12.0, 6.0),
`activity 1.0`, `range 4.0`, 5 Hz, distance off.

**The empty hall has no "tightest passage", and the figure says so.** The suggested caption number was
20.0 m. That is the height of the hall, not a measured width: `worldcheck.py` scores a path by the
clearance of its cells (distance to the nearest wall), and `open` has no goal, so it has no start→goal
path at all. Instead of inventing a number for the panel, the checker now reports what it can measure
when there is no path — the widest free spot in the file, same clearance figure — which for `open` is
**9.25 m** (the centre of a 29 × 19 m interior). `tools/worldpic.py` quotes that under the panel and the
README says plainly that the number is the hall and not a gap. A test compares the README's numbers with
`worldpic.clearance()` rather than with this note. Five panels are three columns now, so the figure grew
from 976 × 886 px to 1632 × 966 px.

**GPS off means a gap, not a rate of zero.** `gps.rate: 0` looks like the natural way to switch the
sensor off, but `engine._due()` clamps the period to 1/0.001 s, so a drive longer than ~16 minutes would
still deliver one fix. `gps.gap: [0.0, 1e9]` is the outage window of §6.4 opened to the width of the
exercise and produces exactly zero messages (measured: 0 GPS messages, 4014 IMU messages and 802 LIDAR
messages in the 40 s drift drive — the other sensors keep working, only the sky is gone).

**The numbers of the two demos.** Drift: 20.00 m driven straight at 0.5 m/s in `open`, seed 1 — with
`odom.geometry.wheel_radius_scale: 1.05` the odometry counts 21.01 m and the ghost is 1.00 m ahead of
the robot (0.60 m after 12 m); with 1.0 it is 0.01 m away. Both runs end with 0 wall contacts, because
there is nothing to touch. Field: the shipped source measures 0.800 at 0.5 m, 0.200 at 2 m, 0.0588 at
the 4 m range and 0.000 past it; with 400 counts per unit the relative σ of one reading is 5.6 % /
11 % / 21 % at those three distances (Poisson: σ counts = √counts). Along the spawn's own line the
reading is 0.000 until x = 8.1 m, peaks near 1.0 at x = 12 m (the line misses the source by 0.25 m),
and the two crossings of 0.50 are 1.90 m apart in the noise-free field — 2·d0, which is how a student
gets a distance out of a series. Through a wall: in `production` at (10.0, 3.0), the LIDAR beam pointed
at the source reports the table at 0.60 m while the counter reports 0.072 ± 0.014.

**A shared noise stream is a coupling, and it is tested as one.** The counter draws from the run's one
`sensors.Noise`, so planting a source shifts the odom/scan/gps/imu values of that run. That is the
correct behaviour for a seeded sensor and the reason nothing in `config/tasks.json` may name a source
until the thresholds are re-measured — `tests/test_world_poi_w5.py` asserts the shift, and
`test_the_poi_keys_change_nothing_while_no_source_is_planted` in the golden file asserts that the keys
themselves are inert while empty.

**Four files outside this package's list, each at its seam.** `render.py` +3 (the `p` key, its boolean,
the call — new drawing belongs to `overlays.py`, which grew +71), `menu.py` +1 (the row),
`ros_bridge.py` +10 (without a `KIND_MSG`/`to_ros` entry a ROS run would raise on the first `/poi`
message; it uses the JSON-on-a-String pattern of `kf/info`, no new interface), `node.py` +0 lines (two
printed topic lists gained the word `poi`, otherwise `./lab docs` would list a topic set that no longer
exists). None of them changes a graded path, a threshold or a message that existed before.

**Left open on purpose.** `logbook.py` has no `intensity_poi` column yet (one entry in `COLUMNS`, the
next person who touches the CSV should add it — until then the readout line and `ros2 topic echo` are
where the reading is), `robot_io.RobotIO` has no `poi()` accessor so a student node reads
`rob.bus.last("poi", rob.name)[0]`, and there is no `student/` gradient-climbing example: `student/*`
is not this package's to write, and the three exercises of `config/demo_poi_exploration.json` are
written so that one of them can become that example.
