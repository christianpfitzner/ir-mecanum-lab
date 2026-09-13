# CONTRACT-KF.md — engineering contract for experiment 2 "state estimation" (not for students)

Extends `docs/CONTRACT.md` (experiment 1) and **does not contradict it**: everything that
exists stays, experiment 2 builds on top of it. Anyone changing a file that experiment 1
also uses keeps backward compatibility — `./lab grade --task alle` (experiment 1) has to
stay green unchanged.

---

## 1. Learning goal and grading idea

| | Experiment 1 | Experiment 2 |
|---|---|---|
| Subject | kinematics + control | state estimation (Kalman filter) |
| Control path | students → `wheel_speeds` | **grader** → `cmd_vel` (pass-through) |
| Sensing path | odometry/LIDAR/GPS → students | sensors → filter → `kf/pose` → grader |
| Verdict | behavior (arrived?) | **accuracy** (RMSE against `truth`) |

The grader drives the robot along the command trajectory defined in
`config/tasks.json`, records `truth` (exact) and the student estimate `kf/pose` at the
same time, and judges RMSE, the improvement ratio over the raw sensor, the maximum error
during a GPS outage and the consistency of the stated standard deviation (NEES). The
student does **not** steer.

For this to work the robot stays in `pass-through` mode: the experiment 2 node never sends
`wheel_speeds` and defines no `inverse_kinematics`.

## 2. New topics

| Topic | Type | Direction | Content |
|---|---|---|---|
| `/<robot>/imu` | `sensor_msgs/msg/Imu` | sim → all | 100 Hz (configurable), body-frame axes, `linear_acceleration.z ≈ +g` |
| `/<robot>/kf/pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | students → sim/grader | own estimate incl. 1σ (diagonal) |
| `/<robot>/kf/info` | `std_msgs/msg/String` | students → all | JSON diagnostics (run time, rates, Q/R) — the evaluator ignores it |
| `/<robot>/truth` | `geometry_msgs/msg/PoseStamped` | sim → all | as in experiment 1, now with an adjustable rate (`truth.rate`), only with `debug_truth` |
| `/sim/config` | `std_msgs/msg/String` | sim → all | JSON of the sensor configuration (the grader checks the test profile) |

`types.MSG_SPECS` carries the new keys `imu`, `kf`, `kfinfo`; `topic("kf","alice")`
→ `/alice/kf/pose`.

## 3. IMU model (`sensors.ImuSensor`) — intent and values

Realistic MEMS model, deliberately keeping the effects that hurt in practice:

1. **White noise** as a *density* (`*_noise` in unit/√Hz) → spread per sample
   `density · √(rate/2)`.
2. **Bias** — one fixed start value per robot and axis (drawn from `Noise`, so it is
   seed-fixed), plus **bias random walk** (`*_bias_walk` in unit/√s). Bias dominates
   everything: an acceleration bias of 0.05 m/s² gives 2.5 m after 10 s of double integration.
3. **Scale error** (`*_scale`, relative, fixed per robot) — noise cannot average it away.
4. **Tilt + vibration**: a small OU-driven roll/pitch (σ `tilt_sigma`, correlation time
   `tilt_tau`) lets gravity leak into the horizontal axes (`g·sin φ`), plus chassis
   vibration (`vibration` at `vibration_hz`).
5. **Settling**: the first `startup` seconds run with a `startup_bias` offset — like an IMU
   driver that has not settled in yet.
6. **Temperature**: the chip is a first-order lag (`temp_tau`) towards `temp_start + temp_motor`
   at `TEMP_FULL_SPEED` (0.8 m/s of driving earns the whole `temp_motor`, more driving earns no
   more of it), and the bias follows that temperature — `temp_walk` m/s² per °C on the
   accelerations, `temp_walk_gyro` rad/s per °C on the rates (`Imu.temp` carries the °C). This is
   the curve a datasheet draws as bias against temperature: it follows the **load**, so it grows
   while the robot drives and walks back while it stands, and averaging does not remove it because
   it is an offset and not noise. Measured over 25 s of straight drive at 0.5 m/s with
   `config/demo_sensor_reality.json`'s `imu` block (14 °C, 25 s, 3.5 mm/s² and 0.12 mrad/s per °C),
   seed 1: 24.00 → 29.5 °C, the mean `az` of the same 25 s 9.7480 m/s² against 9.7368 m/s² with the
   defaults, the mean `gz` −0.005125 against −0.005511 rad/s; at 0.8 m/s and above the chip ends
   120 s of drive at 37.88 °C — 4.8 time constants of a 25 s lag, so 0.11 °C of the 38.00 °C
   asymptote is still missing.
   **Off by default** (`temp_motor`/`temp_walk`/`temp_walk_gyro` = 0.0): no graded task of either
   experiment names a `temp_*` key in its `sim` block, so every threshold of §5 is measured on a
   chip that stays where it started.

`az` holds the specific force (+g when flat), `gx/gy` pick up the tilt as well — exactly
like on a real 6-DOF IMU. The temperature does not change that convention, and it does not touch a
robot that has not driven: measured standing still for 120 s (seed 1), the default configuration and
the demo configuration above give the *same* chip (24.00 °C) and the same mean `az` to four decimals
(9.7381 m/s²) — the −0.07 m/s² that separates that from +9.81 is `accel_bias`, which the temperature
neither adds nor removes. On the wire the °C is not a field of `sensor_msgs/msg/Imu`, so it travels on
`/<robot>/sensor/info` and in the `temp_imu` column of the log (CONTRACT §6.4).

**Determinism:** every random draw per robot comes from the engine's single `Noise` stream.

## 4. Configuration (experiment 2)

New in `DEFAULT_CONFIG` (`types.py`), overridable as before through
`config/default.json` → `--config file.json` → `--set section.key=value`:

```
truth:  {rate: 20.0}
imu:    {rate, gyro_noise, gyro_bias, gyro_bias_walk, gyro_scale,
         accel_noise, accel_bias, accel_bias_walk, accel_scale,
         tilt_sigma, tilt_tau, vibration, vibration_hz, gravity,
         startup, startup_bias,
         temp_start, temp_motor, temp_tau, temp_walk, temp_walk_gyro}   # §3, all four off by default
gps:    {rate, sigma_xy, sigma_theta, bias_xy, delay_ticks,
         gap: [t0, duration], bias_step: [t0, duration, dx, dy]}
kf:     {rate, q_acc, q_turn, gps_delay}   # recommendation to the students; the one value the
                                           # simulator reads is gps_delay, see below
odom:   {rate, sigma_wheel, sigma_xy, sigma_theta, bias_omega, jitter,
         geometry: {wheel_radius_scale, wheel_base_scale, lever_scale, scale_xy, bias_xy}}
```

`gps.delay_ticks` delivers every fix N emissions late out of a ring buffer, and `kf.gps_delay`
— the delay the students are told their filter must survive — is the same delay in **seconds**,
which the engine converts with `gps.rate` (`gps.delay_ticks` wins if both are given). The fix is
still *generated* when the GPS measured it (its own noise, its own place in the `gap` window) and
keeps that stamp on delivery, so the receiver can see `now - fix.t` and predict over the gap
instead of steering to where the robot was. `odom.geometry` is CONTRACT §6.4's model error of the
believed wheel constants; `{}` (the default) is the true geometry, so nothing measured here moved.

`--set` (new in `node.py`) takes `path.sub.path=value`, the value parsed as JSON
(number/bool/string/list). That makes **everything** reachable from the launch file without
writing JSON files.

**Test profile:** every KF task in `config/tasks.json` carries a `"sim"` block with the
sensor profile it is graded against. `make_engine()` (node.py) merges
`DEFAULT < config/default.json <- --config <- tasksim <- --set`. A grading run is therefore
independent of what `config/default.json` happens to say.

## 5. Task plan for experiment 2 (`config/tasks.json`, `"experiment": 2`)

Both columns are the task's own JSON: the profile is its `sim` block, the thresholds are the limit
keys it names (every number below is from `config/tasks.json`, none from an older table).

| ID | P | Core | Profile = `sim` block | Thresholds = the limit keys of the task |
|---|---|---|---|---|
| `kf_gps` | 30 | KF with CV model, GPS only | GPS σ_xy 0.5 m at 5 Hz, `gap []`, IMU 100 Hz, truth 20 Hz, `debug_truth` | `rmse_max 0.42`, `improvement_min 1.6`, `max_error_max 1.25`, `rate_min 5.0`, `contacts_max 0` |
| `kf_fusion` | 30 | GPS + odometry + IMU, GPS outage | GPS σ_xy 0.8 m at 1 Hz, `gap [15, 8]` → outage 15…23 s after task start, IMU 100 Hz with `gyro_bias 0.002`, odometry `sigma_wheel 0.05` / `sigma_xy 0.002` / `sigma_theta 0.0015`, truth 20 Hz | `rmse_max 0.8`, `improvement_min 2.5`, `rate_min 5.0`, `outage.error_max 1.8` with `outage.duration_min 6.0`, `contacts_max 0` |
| `kf_kovarianz` | 20 | consistent 1σ (NEES) | the `kf_gps` profile again: σ_xy 0.5 m at 5 Hz, `gap []`, IMU 100 Hz | `nees [0.05 … 3.5]` — the one task whose band is the exercise — plus `rmse_max 0.42`, `improvement_min 1.0`, `max_error_max 1.25`, `rate_min 5.0`, `contacts_max 0` |
| `kf_dynamik` | 10 | fast + faithful following error | GPS σ_xy 0.6 m at 5 Hz, `gap []`, IMU 200 Hz, truth 20 Hz | `rmse_max 0.35`, `improvement_min 1.8`, `max_error_max 0.9`, `rate_min 10.0`, `contacts_max 0` |

The grader drives the task's `drive` list itself: 45.6 s of commands for K1 and K3, 37 s for K2, 31 s
for K4, each measured only after `warmup 4.0` s and bounded by `timeout` 60 / 56 / 60 / 45 s. A limit a
task does not name is not applied — `_apply()` and `_check_kf()` skip a missing bound — so K3's
`improvement_min 1.0` and `max_error_max 1.25` are real checks rather than decoration: without those
two keys a task worth 20 points would be graded by the NEES band, the rate and the contacts alone.

`--task kf_alle` selects every task with `"experiment": 2` (`tasks.resolve` can do that, and
so does `kf_gps,kf_fusion`). World for all of them: `arena` (open floor, perimeter walls).

Grading formulas (`grade.py`), measured only after `warmup` seconds:

```
rmse(estimate)    = sqrt(mean(dx²+dy²))            against truth, nearest neighbor ≤ 0.25 s
improvement     = rmse(gps) / rmse(kf)
nees              = mean((dx²/sx² + dy²/sy²) / 2)   # 2 degrees of freedom, expected value 1
outage          = max error inside the GPS outage window
```

Two constraints that come with experiment 2 (both in `engine.py`/`node.py`):

* **The grader drives blind.** For a task with `"kind": "kf"` the robot is dropped at the
  spawn pose before the start (`Engine.reset_robot`) — the command sequence in the task's `drive`
  list is
  not fed back, so it has to begin where the world parks the robot. Simulation time keeps
  running; `gps.gap` stays **task-relative** (`set_task`). `--world` may then be left out:
  KF tasks fetch their own arena (`arena`) themselves.
* **The GPS outage and the sensor clocks run on message stamps, not on the wall clock.**
  Odometry and IMU count from the moment they are created; the engine attaches them to
  simulation time with `_clock_to_sim()`, so their stamp does not start at 0 again after a
  respawn or profile switch while GPS keeps reporting simulation time.
  `tools/fastgrade.py` runs the same simulation without the real-time pace (`--speed 8` measured
  2.4 simulation seconds per wall second here, because the reference node competes for the CPU) —
  a wall-clock-timed test would see neither the outage nor a wrong `dt`.

### 5.1 Measured spread per pace (3 runs each, seed 1, reference solution)

Same runs as CONTRACT §9.1, same host. `rate_hz` is the grader's count of `kf/pose` messages per
**simulation** second, everything else is what §5 lists as the check:

| Metric | `--speed 1` | `--speed 4` | `--fixed-step` | limit |
|---|---|---|---|---|
| K1 `rmse` | 0.068 | 0.108…0.145 | 0.406…0.540 | ≤0.42 |
| K1 `improvement` | 10.26 | 4.93…6.57 | 1.28…1.71 | ≥1.6 |
| K1 `max_error` | 0.180 | 0.223…0.349 | 1.065…1.122 | ≤1.25 |
| K1 `rate_hz` | 38.6 | 12.9…13.2 | 2.6…2.7 | ≥5 |
| K2 `rmse` | 0.253…0.506 | 0.273…0.372 | 0.748…0.791 | ≤0.80 |
| K2 `improvement` | 4.05…6.50 | 5.56…6.91 | 2.08…2.20 | ≥2.5 |
| K2 `nees` (not graded) | 1.45…5.62 | 0.90…1.46 | 1.12…1.21 | — |
| K2 `outage_max` | 0.256…0.394 | 0.109…0.485 | 0.417…0.527 | ≤1.8 |
| K3 `rmse` | 0.055…0.114 | 0.085…0.117 | 0.368…0.465 | ≤0.42 |
| K3 `nees` | 0.23…0.74 | 0.23…0.46 | 0.26…0.43 | 0.05…3.5 |
| K4 `rmse` | 0.090…0.091 | 0.174…0.225 | 0.517…0.778 | ≤0.35 |
| K4 `max_error` | 0.161…0.184 | 0.373…0.489 | 1.577…1.785 | ≤0.90 |
| K4 `rate_hz` | 57.7 | 13.9…14.1 | 3.6…4.0 | ≥10 |
| **points** | **90/90** | **90/90** | 0/90 | — |

What that says, in order:

* Nothing moved at `--speed 1` and `--speed 4`: **no threshold of experiment 2 was retuned.**
* `--speed 4` measures *tighter* than real time (K2 rmse 0.253…0.506 → 0.273…0.372, K2 NEES
  1.45…5.62 → 0.90…1.46). The realtime pace was the noisy one, and a K2 run that lands on NEES 5
  is a scheduling accident rather than a filter judgement.
* `--fixed-step` is **not a usable pace for experiment 2**, and that is a property of the check, not
  of the thresholds. A node reports `kf/pose` at most as often as it loops, so its rate in
  *simulation* seconds is (loop iterations per wall second) / (sim seconds per wall second):
  fixed-step runs ~36x realtime here, which leaves 2.6…4.0 Hz against `rate_min` 5 and 10 Hz — and
  with the report rate gone the estimate itself degrades (K4 rmse 0.517…0.778 against 0.35).
  Loosening `rate_min` to make the row green would delete the only check that a node publishes
  continuously instead of once per GPS fix, so it stays; `--fixed-step` is for experiment 1, for
  the simulator and for fast reruns of a filter that reports by message stamps.
* The ceiling a supervisor has to respect is `speed · rate_min < the node's loop rate in wall
  seconds`. Measured on this host with the reference filter (which reports every 0.02 s of sim time):
  `--speed 6` already leaves it at 9.1…9.6 Hz and `--speed 8` at 7.0 Hz, both under K4's
  `rate_min = 10` (80/90 — a scheduling accident, not a wrong filter). So experiment 2 is gradeable
  up to about four times realtime here, and `--fixed-step` is out of the question.
* `tools/fastgrade.py` switches the sleeps of the bus off for the node too, so it keeps the report
  rate even at `--speed 8`: re-measured on this tree, three runs of `python3 tools/fastgrade.py
  --task kf_alle --controller student/kf_solution.py --speed 8` give **90/90 every time** — K1 0.07 m,
  K2 0.599 m, K3 0.117…0.118 m with NEES 0.9, K4 0.081…0.083 m, the rates 38.6 / 38.6 / 32.6 / 57.7 Hz
  (the rates this bullet always quoted), and 170.4 s of simulation in 70.3…70.6 s of wall clock — the
  2.4 simulation seconds per wall second of §5. `--speed 20` gives 90/90 too. **The collapse this
  bullet reported when §5.1 was written — K3 rmse 0.904, NEES 37.0, 70/90 — does not reproduce here.**
  The change next to it is `engine._build_sensors()`, which rebuilds the instruments on a profile
  switch: before that, a switch kept the instruments of the task before it, and K3 is the task whose
  profile differs from its predecessor. That attribution is not tested by this package — what is
  measured is that K3 passes at `--speed 8` now. The guard in the node ("a step over `SLEEP` = 2 s was
  a sleep, not a prediction", `student/kf_solution.py`) is still why a fast pace does not throw the
  filter away, and a KF grade is still only ever worth what the node's clock was worth — which is why
  this section is a table of runs rather than a proof.

**Why K2 grades relatively and K3 with a loose lower bound** (calibrated over seeds 1–4,
reference solution): at 1 Hz GPS with σ = 0.8 m the *absolute* error is noise-limited — the
same implementation measures 0.27…0.60 m RMSE depending on the noise realization. An
absolute limit of 0.45 m would become a lottery, so `improvement_min` (stable: 3.4…6.2)
and the outage limit do the checking. On NEES the same solution scatters between 0.27 and
1.04; the lower bound (0.05 since the paced runs below) only punishes wildly inflated
covariance, the upper one
(3.5) any smug filter.

## 6. File ownership for experiment 2

The list is what is in the tree today, one line per file, with the role experiment 2 uses it for:

```
mecanum_lab/types.py           Imu (incl. `temp`), Kf, SensorInfo, MSG_SPECS, config blocks [Int.]
mecanum_lab/sensors.py         ImuSensor (§3 incl. the thermal term), GpsSensor (gap, zones) [Int.]
mecanum_lab/engine.py          IMU tick, truth rate, set_kf, /sim/config, sensor_info()      [Int.]
mecanum_lab/ros_bridge.py      Imu, PoseWithCovarianceStamped, the JSON-on-a-String topics   [Int.]
mecanum_lab/robot_io.py        imu() kf() send_kf() truth() sensor_profile() config()        [Int.]
mecanum_lab/tasks.py           experiment selection, the `sim` profile of a task             [Int.]
mecanum_lab/grade.py           kind "kf_drive": RMSE, improvement, max error, NEES, outage   [Int.]
mecanum_lab/node.py            --set, --truth, --log, profile merge, the pace of §5.1        [Int.]
mecanum_lab/logbook.py         the CSV a report is written from: x_kf, sx_kf, temp_imu, q_gps [Int.]
mecanum_lab/render.py          truth/gps/kf overlay, the σ ellipse of the estimate           [Int.]
config/tasks.json              four KF tasks, their thresholds and their profiles            [Int.]
worlds/arena.txt               open floor with perimeter walls                               [Int.]
launch/kf.launch.py            main launch: every setting as a declared argument             [Int.]
student/kf_template.py         submission template; grades as an empty estimate              [D]
student/kf_solution.py         reference solution (passes kf_alle)                           [D]
tools/kfplot.py                CSV -> numbers + ASCII chart (matplotlib optional)            [F]
tools/fastgrade.py             the same Grader without the wall-clock pace, see §5.1         [F]
docs/praktikum/kalman.tex      handout for experiment 2                                      [E]
install.sh, README.md          one-command install and quickstart                            [E]
rviz/kf.rviz                   view for odom + gps + kf/pose                                 [F]
tests/test_sensors_imu_b.py        noise density, bias, double integration, outage           [A]
tests/test_kf_grading_integrator.py  the verdict over scripted truth/gps/kf/pose             [Int.]
tests/test_kf_solution_d.py          reference solution, API rules, its line budget          [D]
tests/test_grading_speed.py          the pace of §5.1, with a scripted clock                 [Int.]
tests/test_sensor_knobs.py           the gps delay the students are told to survive          [Int.]
tests/test_config_layering.py        the layer merge of §4 (`None` = nothing overridden)     [Int.]
tests/test_sensor_info.py            where the °C of §3 lives on the wire                    [Int.]
```

The line counts are the output of `python3 tools/loc.py` on this tree, shown as `lines / budget`;
`BUDGET` in that file stays the authoritative list, and the justification for every number in it is the
addendum of the package that grew the file (the comment above `BUDGET`, tally in CONTRACT §7):

```
types.py 560 / 565        engine.py 644 / 650        grade.py 669 / 675       node.py 743 / 755
sensors.py 556 / 560      ros_bridge.py 510 / 515    robot_io.py 286 / 290    tasks.py 211 / 215
render.py 483 / 485       logbook.py 115 / 118       tools/kfplot.py 305 / 310
launch/kf.launch.py 193 / 195   student/kf_template.py 231 / 235   student/kf_solution.py 302 / 310
```

None of these grew because experiment 2 may pad files: the IMU model, the layered config, the
measurement log and the pace of §5.1 are new constraints, and most of the added lines are the sentence
saying what a term does to a drive. Two of the numbers are held from the code rather than from this
table: `student/kf_solution.py` ≤ 310 by `test_reference_solution_stays_in_the_line_budget` (the file is
the reading model for a submitted node, so the budget is a test), and every budget by
`tools/loc.py --strict`. What students have to read — `robot_io.py`, the topics in `types.py` — is still
the small end of the list.

## 7. Constraints (still in force)

Stdlib + pygame, `rclpy` optional, everything runnable headless and without ROS,
determinism through the seed, English as the comment and handout language, no `print` in
the simulation path. The reference solution must **not** use numpy — it is the reading
model for the students' submitted code.

## 8. Working instructions for the agents (experiment 2)

### 8.1 What the student node may and must do

The node runs in the runner `robot_io.serve(modul)`; the simulation **drives** (pass-through
of `cmd_vel` from the grader). Therefore: define **no** `inverse_kinematics` and **never**
call `send_wheels()`/`publish_cmd_vel()` — otherwise the node competes with the grader.

`mission(rob, task)` is called by the runner exactly once per task and should loop until the
task changes:

```python
while rob.running() and rob.task() == task:
    rob.spin(0.005)
    ...
```

For the *grader* the task ends when the command sequence runs out, not through `done`.
`mission()` may therefore run until the task switches; the runner reports `done` afterwards.

**Time is simulation time.** `mess.t` (Odom, Gps, Scan, Imu, Kf, truth) is seconds on the
simulation clock; the runner stamps every outgoing `Kf` message with the current simulation
time. No `time.time()`, no `time.sleep()` as a clock source — `tools/fastgrade.py` decouples
simulation time from wall time, and a wall-clock-timed filter then runs in the wrong
direction. The filter clocks itself off the stamps of the incoming measurements.

**New task = new measurements.** For KF tasks the simulation places the robot at the spawn
pose for every task (`Engine.reset_robot`), but it does not withdraw a measurement that was
published long ago: the bus still holds the last message, and that one belongs to the
*previous* task. A filter that starts from it sits meters off for seconds. Every solution
therefore needs a mission start (`KF.start`, set on the first odometry stamp of the task) and
the follow-up check `fix.t >= f.start`. The handout lists it under "Three rules that save the
run" — it is the case that costs the most points during the tutoring sessions.

Two consequences of that rule, both in the reference solution and in the template:

* **The start itself must be fresh.** At the moment `mission()` begins, note the newest stamp
  the bus already knows (`stamp(rob.odom(), rob.imu(), rob.gps())`) and do not build the
  filter until a stamp lies above it. Comparing stamps with stamps is enough — comparing with
  the wall clock is not, and is exactly what made a KF task fail at random (`integrator.md`
  no. 19).
* **A node that slept does not extrapolate.** If the next measurement is more than `SLEEP`
  seconds ahead of the filter's own time, the node was not running: start the filter again on
  that measurement instead of one giant CV step (which throws the state ahead by seconds of
  velocity and blows up P).

### 8.2 API (use only these methods)

| Call | returns |
|---|---|
| `rob.gps()`, `rob.odom()`, `rob.imu()`, `rob.scan()` | last measurement or None (dataclass from `types.py`) |
| `rob.age("gps")` | age of the last measurement in s (1e9 = never) |
| `rob.sensor_profile()` | dict of the active sensor setup (`gps`, `imu`, `odom`, `truth`, `rate`, `debug_truth`, `seed`) |
| `rob.config("gps.sigma_xy", 0.5)` | a single sensor value, no dict wrangling |
| `rob.task()` | active task title, e.g. `kf_fusion` |
| `rob.send_kf(x, y, theta, sx, sy, sth, info=None)` | estimate incl. 1σ on `/<robot>/kf/pose` |
| `rob.truth()` | exact pose — **only** a self-check, never inside the filter |
| `rob.running()`, `rob.spin(dt)`, `rob.sleep(s)` | lifecycle |

`types.Imu`: `t, ax, ay, az, gx, gy, gz, roll, pitch, temp` (`az` ≈ +g at rest; `temp` is the °C of
§3, and on the ROS wire it rides on `/<robot>/sensor/info`, because `sensor_msgs/msg/Imu` has no such
field). `types.Odom`: `t, x, y, theta, vx, vy, omega`. `types.Gps`: `t, x, y, theta`.

### 8.3 Grading (grade.py, per task in `config/tasks.json`)

After `warmup` seconds every tick measures the last truth, the last estimate and the last
raw sensor; reported are `rmse`, `rmse_<sensor>`, `improvement`, `max_error`, `rate_hz`, `nees`,
`nees_over` (the share of single samples whose per-sample NEES is above the fixed 5.99 in `_eval_kf`;
printed, never graded), `outage_max`/`outage_duration`, `samples`, `time` and `contacts` — the keys
`_eval_kf` fills and `format_report` prints. Passed = all thresholds met. The thresholds live in the
JSON — do **not** retune them in code, make the reference solution better instead.

Valid for the reference solution: `python3 tools/fastgrade.py --task kf_alle
--controller student/kf_solution.py --speed 20` has to exit 0, and
`./lab grade --task kf_alle --controller student/kf_solution.py` as well.

### 8.4 Constraints for all files

Stdlib + pygame, **no numpy** (the reference solution is a reading model), English as the
comment language, no banner comments, keep the LOC budgets from §6. Tests are named
`tests/test_<module>_<agent>.py` and run without ROS, without a window, without network
(`SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q`).
