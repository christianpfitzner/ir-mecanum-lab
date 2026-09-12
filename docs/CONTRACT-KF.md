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

`az` holds the specific force (+g when flat), `gx/gy` pick up the tilt as well — exactly
like on a real 6-DOF IMU.

**Determinism:** every random draw per robot comes from the engine's single `Noise` stream.

## 4. Configuration (experiment 2)

New in `DEFAULT_CONFIG` (`types.py`), overridable as before through
`config/default.json` → `--config file.json` → `--set section.key=value`:

```
truth:  {rate: 20.0}
imu:    {rate, gyro_noise, gyro_bias, gyro_bias_walk, gyro_scale,
         accel_noise, accel_bias, accel_bias_walk, accel_scale,
         tilt_sigma, tilt_tau, vibration, vibration_hz, gravity,
         startup, startup_bias}
gps:    {rate, sigma_xy, sigma_theta, bias_xy, gap: [t0, duration], bias_step: [t0, duration, dx, dy]}
kf:     {rate, q_acc, q_turn, gps_delay}    # pure recommendation to the students,
                                            # the simulation never uses this block
```

`--set` (new in `node.py`) takes `path.sub.path=value`, the value parsed as JSON
(number/bool/string/list). That makes **everything** reachable from the launch file without
writing JSON files.

**Test profile:** every KF task in `config/tasks.json` carries a `"sim"` block with the
sensor profile it is graded against. `make_engine()` (node.py) merges
`DEFAULT < config/default.json <- --config <- tasksim <- --set`. A grading run is therefore
independent of what `config/default.json` happens to say.

## 5. Task plan for experiment 2 (`config/tasks.json`, `"experiment": 2`)

| ID | P | Core | Profile | Thresholds |
|---|---|---|---|---|
| `kf_gps` | 30 | KF with CV model, GPS only | σ=0.5 m, 5 Hz | RMSE ≤ 0.42 m, improvement ≥ 1.6, max error ≤ 1.25 m, rate ≥ 5 Hz |
| `kf_fusion` | 30 | GPS + odometry + IMU, GPS outage | σ=0.8 m, 1 Hz, outage 15…23 s after task start, gyro bias 0.002 rad/s | RMSE ≤ 0.80 m, improvement ≥ 2.5, outage error ≤ 1.8 m |
| `kf_kovarianz` | 20 | consistent 1σ (NEES) | σ=0.5 m, 5 Hz | mean NEES in [0.05 … 3.5], RMSE ≤ 0.42 m |
| `kf_dynamik` | 10 | fast + faithful following error | σ=0.6 m, 5 Hz, IMU 200 Hz | RMSE ≤ 0.35 m, improvement ≥ 1.8, max error ≤ 0.9 m, rate ≥ 10 Hz |

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
  spawn pose before the start (`Engine.reset_robot`) — the command sequence in `fahrt` is
  not fed back, so it has to begin where the world parks the robot. Simulation time keeps
  running; `gps.gap` stays **task-relative** (`set_task`). `--world` may then be left out:
  KF tasks fetch their own arena (`arena`) themselves.
* **The GPS outage and the sensor clocks run on message stamps, not on the wall clock.**
  Odometry and IMU count from the moment they are created; the engine attaches them to
  simulation time with `_clock_to_sim()`, so their stamp does not start at 0 again after a
  respawn or profile switch while GPS keeps reporting simulation time.
  `tools/fastgrade.py` runs 25 simulation seconds per second — a wall-clock-timed test
  would see neither the outage nor a wrong `dt`.

**Why K2 grades relatively and K3 with a loose lower bound** (calibrated over seeds 1–4,
reference solution): at 1 Hz GPS with σ = 0.8 m the *absolute* error is noise-limited — the
same implementation measures 0.27…0.60 m RMSE depending on the noise realization. An
absolute limit of 0.45 m would become a lottery, so `improvement_min` (stable: 3.4…6.2)
and the outage limit do the checking. On NEES the same solution scatters between 0.27 and
1.04; the lower bound (0.05 since the paced runs below) only punishes wildly inflated
covariance, the upper one
(3.5) any smug filter.

## 6. File ownership for experiment 2

```
mecanum_lab/types.py        Imu, Kf, MSG_SPECS, config blocks             [Integrator]
mecanum_lab/sensors.py      ImuSensor, GpsSensor (gap/bias_step)          [Integrator]
mecanum_lab/engine.py       IMU tick, truth rate, set_kf, /sim/config     [Integrator]
mecanum_lab/ros_bridge.py   Imu and PoseWithCovarianceStamped conversion  [Integrator]
mecanum_lab/robot_io.py     imu() truth() kf() send_kf()                  [Integrator]
mecanum_lab/tasks.py        experiment selection, sim-profil()            [Integrator]
mecanum_lab/grade.py        KF kind "kf_drive" + measurement              [Integrator]
mecanum_lab/node.py         --set, --truth, --log, profile merge, tap     [Integrator]
mecanum_lab/render.py       truth/gps/kf overlay + covariance ellipse     [Integrator]
config/tasks.json           four KF tasks                                 [Integrator]
worlds/arena.txt            open floor                                    [Integrator]
launch/kf.launch.py         main launch: everything as an argument        [Integrator]
student/kf_template.py      submission template with TODOs                [D]
student/kf_solution.py      reference solution (passes kf_alle)           [D]
docs/praktikum/kalman.tex   handout for experiment 2                      [E]
tools/kfplot.py             CSV -> matplotlib or ASCII chart              [F]
install.sh, README.md       one-command install, quickstart               [F]
rviz/kf.rviz                view for odom + gps + kf/pose                 [F]
docs/praktikum/kalman.tex   handout for experiment 2                      [E]
install.sh + README.md      three-line install                            [E]
tests/test_sensors_imu_b.py noise model, determinism, outage               [A]
tests/test_kf_grade_e.py    grading with a built-in ideal filter          [D]
```

Budgets (new additions, `tools/loc.py`): `sensors.py` 260, `grade.py` 300, `node.py` 240,
`types.py` 290, `engine.py` 210, `ros_bridge.py` 290, `robot_io.py` 215, `render.py` 230,
`launch/kf.launch.py` 130, `student/kf_template.py` 150, `student/kf_solution.py` 260,
`tools/kfplot.py` 120.

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
* **A node that slept does not extrapolate.** If the next measurement is more than `SCHLAF`
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

`types.Imu`: `t, ax, ay, az, gx, gy, gz, roll, pitch` (`az` ≈ +g at rest).
`types.Odom`: `t, x, y, theta, vx, vy, omega`. `types.Gps`: `t, x, y, theta`.

### 8.3 Grading (grade.py, per task in `config/tasks.json`)

After `warmup` seconds every tick measures the last truth, the last estimate and the last
raw sensor; reported are `rmse`, `rmse_<sensor>`, `improvement`, `max_error`, `rate_hz`,
`nees`, `outage_max`/`outage_duration`. Passed = all thresholds met. The thresholds live in the
JSON — do **not** retune them in code, make the reference solution better instead.

Valid for the reference solution: `python3 tools/fastgrade.py --task kf_alle
--controller student/kf_solution.py --speed 20` has to exit 0, and
`./lab grade --task kf_alle --controller student/kf_solution.py` as well.

### 8.4 Constraints for all files

Stdlib + pygame, **no numpy** (the reference solution is a reading model), English as the
comment language, no banner comments, keep the LOC budgets from §6. Tests are named
`tests/test_<module>_<agent>.py` and run without ROS, without a window, without network
(`SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q`).
