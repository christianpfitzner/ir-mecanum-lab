# CONTRACT.md — Internal engineering contract (not for students)

This file is the **single source of truth** on naming, interfaces and file
ownership. Every module follows it; deviations only after talking to the
integrator (who also does the integration and the testing).

Project goal: a 2D mecanum simulator in Pygame with ROS 2 bindings for a
university lab course. Constraints: **very little code**, **very few
dependencies**, **runs immediately**, readable and editable by students.

---

## 1. Principles (hard requirements)

| # | Rule |
|---|-------|
| 1 | **Dependencies:** Python stdlib + `pygame`. `rclpy` is *optional* (it runs without ROS too). No numpy, no yaml, no scipy, no ROS message generation as a must. |
| 2 | **Python:** 3.10+, stdlib types only (`dataclasses`, `math`, `json`, `random`, `argparse`, `threading`). No type ceremony without value — where an annotation costs readability, a comment wins. |
| 3 | **Everything written and everything named is English**: comments, docstrings, log and report text, identifiers, launch arguments, the keys of `config/tasks.json`, handouts, contracts. Two tools enforce it as steps of `tools/check.sh`: `tools/langcheck.py` reads the prose (umlauts, German function words), `tools/germanids.py` reads the names — definitions, arguments, attributes, module-level constants and data strings; local variables are counted and reported on every run and fail it only with `--locals` (§6.11). What `germanids.py` allows is the exception list: ROS topic, service and message field names, the wheel labels `VL/VR/HL/HR`, the task groups on the command line (`alle`, `beide`, `kf_alle`, `v1`, `v2`) and the German *values* of the two compatibility maps — `tasks._LEGACY_KEYS` and the `DEPRECATED` tables in `launch/`. Those maps exist so that files and shell histories from before the migration keep working; see §6.11. |
| 4 | **No class that only forwards.** If a function is under 4 lines and used once, inline it. |
| 5 | **Keep the LOC budgets** (see §7). At the end of every file: no blank-line junk, no banner comments. |
| 6 | **Determinism:** world physics depends only on `dt` and the seeds, never on wall clock time or thread order. |
| 7 | Everything runs under `SDL_VIDEODRIVER=dummy` (headless, CI) and without ROS. |
| 8 | No `print` for debugging in the simulation path — `logging` through `mecanum_lab.log`. |

## 2. Directory & file ownership

```
mecanum-lab/
├── lab                       bash, ONLY ENTRY POINT for students        [B]
├── README.md                 Quickstart                               [E]
├── requirements.txt          pygame                                   [B]
├── config/
│   ├── default.json          simulation configuration                   [MINE]
│   └── tasks.json            grading thresholds                         [D]
├── worlds/*.txt              environments as ASCII grids                [A]
│                             cell size per world: worlds.cell_by_world   (types.py)
├── mecanum_lab/
│   ├── __init__.py           log helper, version                        [MINE]
│   ├── types.py              data types, topics, config                 [MINE]
│   ├── stub.py               in-process bus (runs without ROS)           [MINE]
│   ├── engine.py             SimEngine: robots, step, sensors           [MINE]
│   ├── worlds.py             ASCII grid -> World                        [A]
│   ├── physics.py            Mecanum IK/FK, chassis, collision          [A]
│   ├── sensors.py            odometry / LIDAR / GPS + noise             [A]
│   ├── render.py             pygame renderer: layers, walls, robots     [C]
│   ├── cam.py                camera: zoom to cursor, pan, resize        [C]
│   ├── menu.py               layer menu — view only, never the bus      [C]
│   ├── ros_bridge.py         rclpy backend + stub bus + services        [B]
│   ├── tf_bcast.py           /tf and /tf_static (map → odom → base_link) [B]
│   ├── node.py               CLI: sim/run/teleop/spawn/grade/robots     [B]
│   ├── robot_io.py            client for student nodes  [D]
│   ├── tasks.py              task definitions           [D]
│   └── grade.py              grading run                [D]
├── student/
│   ├── controller_template.py  submission template with TODOs           [D]
│   └── solution.py             reference solution                       [D]
├── launch/*.launch.py        ROS 2 launch files                         [B]
├── interfaces/mecanum_lab_interfaces/   own spawn service               [B]
├── tests/                    pytest, no ROS dependency            [A,B,D]
└── docs/praktikum/anleitung.tex  handout (LaTeX)                        [E]
```

**Rule:** every file belongs to exactly one agent. Freedom from `git` conflicts beats
elegance. If agent X needs a change in a file owned by someone else: write it into
`docs/notes/<agent>.md` — one file per agent, never shared — and **do not edit
it yourself**. Same place for: LOC tally, deviations from this contract, test commands.

## 3. Topics, frames, types

Naming scheme: `/<robot>/<topic>` for robot edges, `/sim/<...>` for the simulation.

**Frames carry the robot name**, because TF names are global and 25 robots drive in one hall:
`map` (the world of `worlds/<name>.txt`, no prefix) → `<robot>/odom` → `<robot>/base_link` →
`<robot>/laser`, `<robot>/imu_link`. The same names appear in the message headers — one source
for both: `tf_bcast.frames()` in `tf_bcast.py`, which `ros_bridge.to_ros()` also calls.
`gps`, `truth` and the students' `kf/pose` stay in `map`.

| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | sim → everyone | the tree above, published by `tf_bcast.attach()`; **no tf2_ros dependency** — plain rclpy, so §1 stays true. `tf.map_to_odom: odom` (default) keeps `map → odom` the identity, so `map → base_link` is the drifting odometry: closing that gap is the students' job in lab 2, not a gift from the sim. `tf.map_to_odom: truth` is the tutor's view; `/<robot>/truth` remains the ground truth topic. |

| Topic | Type (ROS 2) | Direction | Content |
|---|---|---|---|
| `/<robot>/cmd_vel` | `geometry_msgs/msg/Twist` | student node → sim | body velocity (vx, vy, omega); vy positive to the **left** |
| `/<robot>/wheel_speeds` | `std_msgs/msg/Float64MultiArray` (4) | students → sim | wheel speeds [FL, FR, RL, RR] in rad/s — **the actual exercise** |
| `/<robot>/odom` | `nav_msgs/msg/Odometry` | sim → everyone | dead reckoning from *measured* wheel speeds, noisy |
| `/<robot>/scan` | `sensor_msgs/msg/LaserScan` | sim → everyone | 360 rays, 0…2π, range `range_max` |
| `/<robot>/gps` | `geometry_msgs/msg/PoseStamped` | sim → everyone | noisy global position ("UWB/MoCap") |
| `/<robot>/gps_cov` | `geometry_msgs/msg/PoseWithCovarianceStamped` | sim → everyone | the same fix, one emission later on the same stamp, with its covariance on the diagonal (`σ_xy²`, `σ_xy²`, …, `σ_theta²`) — subscribe to this instead of `/gps` and nothing is lost, see §6.4 |
| `/<robot>/truth` | `geometry_msgs/msg/PoseStamped` | sim → everyone | exact pose, only with `debug_truth: true` |
| `/<robot>/poi` | `std_msgs/msg/String` | sim → everyone | JSON `{t, intensity}` of the radiation counter (§6.13) — a String topic because no standard message carries a stamped scalar, same pattern as `/sim/robots`; which source is loudest, and how far away, stay simulation truth |
| `/<robot>/link` | `std_msgs/msg/String` | sim → everyone | JSON `{t, quality, rssi_dbm, ap, up, dropped, latency_ms}` of the radio the commands travel on (§6.14); only when `wifi.enabled` |
| `/<robot>/sensor/info` | `std_msgs/msg/String` | sim → everyone | JSON `{t, quality, sats, lost, latency_ms, temp, scan_gaps}` — what the instruments say about themselves, at `gps.rate`; the three measurement messages have no field for any of it (§6.4) |
| `/<robot>/mission_state` | `std_msgs/msg/String` | students → sim/grader | `"idle"`, `"running"`, `"done"`, `"failed:<reason>"` |
| `/sim/robots` | `std_msgs/msg/String` | sim → everyone | JSON: list of robots including color/marker/mode, plus `steer_deg` — the two rack angles in degrees — on a steering robot (§5.1), empty on a mecanum one |
| `/sim/task` | `std_msgs/msg/String` | grader → sim → everyone | active task (`kinematik`, `quadrat`, …) |
| `/sim/spawn_robot` | *service*, see §4 | student/supervisor → sim | add a robot **with a given name** |
| `/sim/despawn_robot` | *service*, see §4 | same | remove a named robot |
| `/sim/spawn_next` | `std_srvs/srv/Trigger` | same | add a robot, the simulator picks the free name (§4) |
| `/sim/despawn_last` | `std_srvs/srv/Trigger` | same | remove the robot that joined last (§4) |
| `/sim/reset` | `std_srvs/srv/Trigger` | same | reset the world |
| `/clock` | `rosgraph_msgs/msg/Clock` | sim → everyone | simulation time (seconds, `use_sim_time`) |

The `wheel_speeds` order is sacred: **index 0=front left (FL), 1=front right (FR), 2=rear left (RL), 3=rear right (RR)**.

**Fallback mode (mandatory, otherwise there is no quick start):** if a robot has
*never* sent `wheel_speeds` and `cmd_vel` arrives, the sim takes the twist
directly ("pass-through"). As soon as the first `wheel_speeds` message arrives, the
robot stays in `wheels` mode forever. The mode is in `/sim/robots` and in the HUD.

## 4. Spawn service (ROS 2 has no built-in string service)

Adding a robot to a running simulation is a service call with one string in it, and `std_srvs` has
never had a service type that carries a string (`Trigger`, `SetBool`, `Empty` — in Kilted even
`SetString` is gone). Three transports, therefore, and a fourth that needs no argument at all. The
calling side (`robot_io`, `tools/lab`) notices nothing about which of them is in use:

1. **Preferred:** own interface `mecanum_lab_interfaces/srv/SpawnRobot`
   (lives in `interfaces/`, `colcon build --packages-select mecanum_lab_interfaces`):
   ```
   string name            # unique, [a-z0-9_]{2,24}
   string variant         # "" = automatic (slow|fast|agile)
   ---
   bool   success
   string message         # what happened, in a sentence ("spawned 'robot1' (stock, red) at ...")
   uint8  index
   string color           # "red", "blue", ...
   string marker          # "triangle", "square", ...
   float64 x
   float64 y
   float64 theta
   ```
2. **Fallback without the interface build:** topic handshake
   `/sim/spawn_robot/request` (`std_msgs/String`, JSON `{"name": "...", "variant": ""}`)
   → reply on `/sim/spawn_robot/result` (JSON in the same format). Timeout 2 s.
3. **Always there, no name in the request:** `/sim/spawn_next` and `/sim/despawn_last`, both
   `std_srvs/srv/Trigger`. The simulator chooses the name (`robot1`, `robot2`, … the first that is
   free) and answers in `message`, which is the only channel a Trigger has:

   ```bash
   ros2 service call /sim/spawn_next std_srvs/srv/Trigger
   # response: "spawned 'robot1' (stock, red) at (7.25, 4.75, 0 deg)"
   ros2 service call /sim/despawn_last std_srvs/srv/Trigger
   # response: "'robot1' removed"        # the robot with the highest index, i.e. the last that joined
   ```

   This is what a supervisor button, a shell history line, or a machine without
   `mecanum_lab_interfaces` built calls. `./lab spawn --name carlo` stays the named form.
4. **Stub mode (no ROS):** direct method call `SimEngine.spawn(name, variant)`; an empty `name` there
   means the same anonymous request as `/sim/spawn_next` (`SimEngine.next_name()`).

Name rules: `^[a-z][a-z0-9_]{1,23}$`; duplicate → `success=false`, `message="Name 'carl' is already
taken."`; `spawn_limit` reached → `message="Robot limit (12) reached."`; both are `SpawnError`, which
the handler turns into `success=false` rather than a stack trace on the caller's terminal.

## 5. Physics conventions (agents A and D must compute identically)

Right-handed system, x forward, y to the **left**, theta counter-clockwise (CCW)
positive. Wheels: `FL` (front left), `FR` (front right), `RL`, `RR`.
Mecanum in the **X layout** (roller axes point forward-inward), as in the
common kits.

```
a = lx + ly                      # lever arm for yaw
w_FL = ( vx - vy - a*omega) / r      # inverse kinematics
w_FR = ( vx + vy + a*omega) / r
w_RL = ( vx + vy - a*omega) / r
w_RR = ( vx - vy + a*omega) / r
```

Forward kinematics (pseudo-inverse, slip is ignored) is the exact inverse:

```
vx    =  r/4 * (w_FL + w_FR + w_RL + w_RR)
vy    =  r/4 * (-w_FL + w_FR + w_RL - w_RR)
omega =  r/(4*a) * (-w_FL + w_FR - w_RL + w_RR)
```

`types.GEOM` provides `lx, ly, r` — all implementations use the **same** signs.
Unit test `tests/test_kinematics.py` checks `fk(ik(v)) == v` for random values
*and* the signs above explicitly (y arrow points left!).

### 5.1 The steering variant (`--variant steering`, `mecanum_lab/steering.py`, §6.12)

Same frame, same axis directions, same wheel order and labels — but one driven axle and one steered
axle instead of four mecanum wheels. The body velocity therefore has **one** degree of freedom
(metres per second along the body x-axis) and the turn rate is not free: it follows from the angle
the rack took. `vy` has no solution on this car; `cmd_vel` drops it with one warning per robot and
keeps driving.

```
omega = v * tan(delta) / L                    # L = wheel_base, delta = rack angle, + = left
R     = L / tan(delta)                        # the radius this angle drives
R_min = L / tan(delta_max)                    # 1.60 m by default: no tighter corner exists
delta = atan2(omega * L, max(|v|, 0.05))      # cmd_vel input, solved backwards
tan(delta_inner) = L / (R - W/2)              # W = track: the two front wheels cannot share one angle
tan(delta_outer) = L / (R + W/2)              # both axles turn about one point on the rear axle line
```

`R` is `inf` straight ahead (`tan 0 = 0`), which is why the code works with `1/R` throughout.
The numbers are measured in `tests/test_steering_w4.py` as the **chord of a half turn** in a
wall-free world — not as `v/omega`, which would compare the model with itself: demanded
14°/22°/30° drive 4.011/2.475/1.732 m against `L/tan(delta)`, ratio 1.0000 for each, and demanding
45° still drives `R_min` = 1.600 m. The rack is rate-limited (`steer_rate_deg_s`, 60°/s = 1.2° per
1/50 s step), so a step command arrives as a ramp of 16 steps; that is the second reason a steering
car brakes before a corner. `w = v/r` still holds per wheel: the rear pair rolls at `v(1 − W/2R)`
and `v(1 + W/2R)`, so their mean is exactly `v`, and the four speeds go out on `/<robot>/wheel_speeds`
with the old labels (§3).

## 6. Interfaces between modules (implement exactly like this)

### 6.1 `mecanum_lab/types.py` [MINE, already written — read only]
`Pose(x,y,theta)`, `Twist(vx,vy,omega)`, `Rect(x0,y0,x1,y1)`, `World(name,cell,walls,spawns,goal,size)`,
`RobotSpec(name,index,color,variant,pose)`, `Robot(...)`, `Odom`, `Scan`, `Gps`,
`PALETTE`, `MARKERS`, `CONFIG` (dict), `load_config()`, `topic(kind, robot=None)`,
`sanitize_name()`, `log`.

### 6.2 `mecanum_lab/worlds.py` [A]
```python
def load_world(name: str, path: str | None = None) -> World      # cached; path: worlds/<name>.txt
def parse_grid(text: str, cell: float = 0.5, name: str = "?") -> World
def list_worlds() -> list[str]
```
ASCII grid: each character block is `cell` meters across.
`#` wall, `.`/` ` free, `S` spawn pose 1, `2`..`6` further spawn poses, `G` goal,
`-`/`|` painted floor — free to drive over, and drawn nowhere (see below).
Robot centre sits in the cell centre; theta from the order: 1=`0°`, 2=`90°`, 3=`180°`, 4=`270°`, then 0° again.
Neighbouring wall cells merge into large rectangles (2-pass, reduces ray tests).

Nothing that is not drawn lives in a grid file: the world has no comments and no sidecar format, so
**a world key that is data comes from the config** — `pois` (the sources of §6.13) and
`worlds.cell_by_world` are both config keys, and a demo file that plants a source therefore names
the hall it belongs to (`config/demo_poi_exploration.json` sets `world` as well).

`worlds/open.txt` is the hall of the drift exercises: 60 × 40 cells of 0.5 m = 30 × 20 m, border
walls only, two spawns on the centre line, no obstacle, no marking, no goal. A goal-less world is
legal (the checker only says so) and `tools/worldcheck.py` then has no start→goal path to measure; it
reports the widest free spot it found instead — the same per-cell clearance number, just without a
path to walk. That is the number `tools/worldpic.py` puts under the panel and the README quotes.

### 6.3 `mecanum_lab/physics.py` [A]
```python
@dataclass
class Geometry: lx, ly, r, max_speed, max_accel, tau, footprint_r
def make_geometry(cfg: dict, variant: str = "stock") -> Geometry   # variant: stock|slow|fast|agile
def inverse_kinematics(g, vx, vy, omega) -> list[float]            # [FL,FR,RL,RR] rad/s
def forward_kinematics(g, w) -> tuple[float, float, float]         # vx, vy, omega
class Chassis:
    def __init__(self, g: Geometry, pose: Pose, seed: int | None = None)
    wheels: list[float]        # measured actual wheel speed rad/s
    pose: Pose                 # exact (truth) pose
    twist: Twist               # body velocity from the wheels
    contacts: int              # number of wall contacts (increasing)
    def set_wheels(self, w: Sequence[float]) -> None    # clamped to max_speed
    def step(self, dt: float, walls: list[Rect]) -> None  # motor inertia, FK, integrate, collision
```
Motor: first-order lag with `tau` plus an angular-acceleration limit
`max_accel`. Collision: circle `footprint_r` against rectangles, the position is pushed
out along the smallest penetration depth, `contacts += 1` (only on a new contact). **Slip is
modelled at the obstacle only**: the body velocity becomes 0 while the wheels keep
`robot.slip` × the commanded speed (default `1` = full slip, `0` = ideal static friction). That
is the one place where odometry may lie — it integrates wheel speeds — and it is there on purpose
so that T1's drift is visible without turning any noise up. Everywhere else four wheel speeds give
the body velocity uniquely.

Four members exist for the second drive train, which subclasses `Chassis` instead of duplicating the
collision and the motor lag: `geom` (the geometry, so the view draws the car it simulates),
`wheel_headings` (always `[0,0,0,0]` here — a mecanum wheel never turns), `reset_motion()` (speeds
and targets to zero) and `set_twist(vx, vy, omega)` / `odo_feed()` (the two ends of the `cmd_vel`
path: in through the kinematics, out to the integrator). All four are behaviour-neutral: a mecanum
robot computes what it computed before, line for line, and `steering.py` overrides them (§6.12).

### 6.4 `mecanum_lab/sensors.py` [A]
```python
class Noise:  __init__(self, seed); gauss(sigma); uniform(a); chance(p); late(frac)
class OdometrySensor: __init__(self, g, noise, cfg); reset(pose); update(self, wheels, dt) -> Odom
class Lidar: __init__(self, world, noise, cfg); scan(self, pose) -> Scan
class GpsSensor: __init__(self, noise, cfg); reset(); fix(self, pose, t=0.0, robot="") -> Gps
def odom_geometry(true_geometry, scales) -> physics.Geometry   # what the odometer believes
```
`GpsSensor.sky(pose)` also belongs there: `(quality, satellites in view)` for a position, without a
random number — see the block below.
**Naming rule (integrator, after a name collision was found):** the sensor classes are
`OdometrySensor`, `Lidar`, `GpsSensor`; `types.Odom/Scan/Gps` are the messages. One
module must not carry both names — `sensors.Gps` would have shadowed the import of
`types.Gps` and produced `TypeError: unexpected keyword 't'`.
Odometry **computes only from the wheel speeds it is given** (measured values,
noise included), never from the truth pose → drift arises naturally and is
the point of Experiment 1. LIDAR: ray/AABB slab test, hits clamped to `range_max`,
no hit = `inf` → reported in `LaserScan.ranges` as `range_max` (`inf` only
internally). Hits against other robots: no (Experiment 1).
`gps.zones` (default `[]`) degrades the fix **by place** instead of by time: the first rectangle
that contains the robot multiplies `sigma_xy` by `sigma_scale`, adds `bias_xy` to the existing
bias, or returns no fix at all when `block` is set. `_zones()` drops malformed entries rather
than crashing, and with the default `[]` the noise calls are exactly the ones of the old sensor —
so graded results are unchanged. `config/demo_gps_shadow.json` is the demo that turns it on.

Two more knobs that were in the config but read by nobody:

* `sensors.odom_geometry(true_geometry, odom.geometry)` builds the geometry the **odometry
  integrator believes**, which is the point: handing it the chassis geometry made the commonest
  real odometry error unexpressible. `wheel_radius_scale` scales the whole path (and the yaw rate,
  the radius sits in both equations), `lever_scale` scales a = lx+ly so every turn comes out too
  big and a commanded circle ends rotated, `wheel_base_scale` mis-measures lx only; `scale_xy`
  (velocity too fast/slow, yaw right) and `bias_xy` (wrong odom origin) are not geometry and are
  applied by `OdometrySensor` itself. Empty config -> the true object, so every graded number is
  what it was. Demo: `config/demo_odom_error.json`.
* `lidar.max_walls` is how many segments one scan may use (taken in world order): what is past the
  cap is not there for the robot. Default 400 > the wall count of every world in `worlds/`.
* `gps.delay_ticks` delivers each fix N emissions late out of a ring buffer, and `kf.gps_delay`
  (seconds, the student-facing name) is the same thing through `gps.rate`. The message keeps the
  stamp it was **generated** with — `SimEngine._push(..., stamp=)` exists for that — so a filter can
  see `now - fix.t` and predict over the delay instead of feeding itself a stale position.

**What a sensor says about itself** — the numbers a measurement comes with in the real lab. Every
knob is off by default, and `config/demo_sensor_reality.json` is the drive that turns them on.
Measured with a 25 s straight drive at 0.5 m/s in `production` (seed 1), one number per knob:

* `Gps.quality` (2 good, 1 degraded, 0 no fix) and `Gps.sats` come from `sky(pose)`, which needs
  only the position: a zone leaves `sats` anchors in view, or one per unit of `sigma_scale`
  inflation when it doesn't say, and `block` leaves none. Quality 0 is a receiver without a
  solution, and `fix()` answers `None` for it — there is no position to send. `None` stays *no
  message* (gap window, blackout, dropped packet), which is a different fault: the window therefore
  shows `q0 0 sats` for the place and `lost 7` for the radio, from the receiver and not from the
  last message. Measured on one straight drive: q2/8 sats on open floor, q1/3 between the racks
  (x = 8.0…11.5 m), q0/0 in the dock (x = 16.6…19.3 m).
* `gps.dropout` is the probability per message that the transport loses it, and the coin is tossed
  **before** anything is measured: the holes of a run belong to the `--seed` and to the number of
  emissions, never to where the robot happened to stand. Losses are counted per robot
  (`GpsSensor.drops`, cleared with the sensor). Measured with 0.15: 12 of 122 emissions, 9.8 %.
* `gps.latency` (seconds, ±50 % jittered, `LATENCY_JITTER`) is the same mechanism as `delay_ticks`
  in seconds and asynchronous: a fix is due on the wire and one fix leaves per emission. It moves
  the stamp and never the value. Measured with 0.25: a fix arrives 0.45 s after it was measured
  (0.25 s of wire plus the wait for the next slot), at most 1.0 s, and the delayed series is the
  head of the undelayed one.
* `imu.temp_*` — the chip is a first-order lag (`temp_tau`) towards `temp_start` + `temp_motor` at
  `TEMP_FULL_SPEED`, and the bias follows its temperature (`temp_walk` for the accelerations,
  `temp_walk_gyro` for the rates). That is the curve a datasheet draws as bias against temperature:
  it follows the load, so it grows while the robot drives and walks back while it stands, and
  averaging does not remove it because it is an offset and not noise. Measured over 25 s of
  driving: 24.00 → 29.52 °C, `az` bias +0.024 m/s², `gz` bias +0.00069 rad/s; with the defaults the
  chip stays at 24.00 °C and `az` stays at +9.81 while standing. The temperature itself is not a
  field of `sensor_msgs/Imu`, so on the wire it travels on `/sensor/info` (below); in the window and in
  the `temp_imu` column of the log it is the number. (The rest of the IMU model: CONTRACT-KF §3.)
* `lidar.reflectivity_min` is a sensitivity threshold in |cos| of the incidence angle on the face a
  ray entered, so `_ray_rect()` answers `(distance, face)` and `Lidar._reflects()` decides whether
  anything comes back at all. Measured 0.5 m off a 30 m wall: that wall is seen 7.17 m down its
  length with the default 0.0 and 1.93 m at 0.25, 20 of 360 beams report nothing, and the 0.500 m
  beam pointing straight at it is bit for bit the same. This is why real lidars miss painted posts.
* `Scan.missing` counts the beams that came back `inf`, so a clipped reading cannot be mistaken for
  a wall; `range_max` is the field that says where the sensor gives up — spelled that way after
  `sensor_msgs/msg/LaserScan`, whose `range_max` it becomes one-to-one (§6.7). The alternative
  `max_range` was considered and rejected: the ROS message already has a name for this.
* **What the wire cannot carry, `/sensor/info` carries.** `LaserScan` has no field for the count, so
  `ros_bridge.py` turns `inf` into `range_max` and there the count is `sum(r >= range_max)`;
  `quality` and `sats` have no place in `geometry_msgs/msg/PoseStamped`, and the chip temperature has
  none in `sensor_msgs/Imu`. Those five numbers used to exist only in the window and in the log, which
  is the one place a second terminal cannot reach, so `engine.sensor_info()` publishes them as one
  JSON object on `/<robot>/sensor/info` at `gps.rate` — `types.SensorInfo`, `t, quality, sats, lost,
  latency_ms, temp, scan_gaps`, the same JSON-on-a-String pattern as `/poi` and `/link` (§6.13, §6.14)
  and for the same reason: a custom interface would put a colcon build in front of someone who only
  wants to read a quality. `ros2 topic echo /alice/sensor/info` therefore answers what the readout
  line answers. Named for the instruments and not for the GPS, because the temperature is the IMU's
  and the gaps are the LIDAR's.
* A GPS fix carries the σ **it was drawn with**: `types.Gps.sigma_xy` and `sigma_theta` travel in the
  message, because they are not constants of the device. A `gps.zones` shadow multiplies `sigma_xy` for
  the fixes it produces, and with `delay_ticks` a fix measured in the open is delivered after the robot
  has crossed into the shadow — a filter that looks the σ up in the settings (or in `/sensor/info`, which
  answers the *setting*) then trusts a wide reading too little and a narrow one too much. The measurement
  stream itself did not change with these two fields: `tests/test_sensor_reality.py` keeps its digest over
  every value, stamp and random draw of a 12 s drive, and the σ fields are excluded from it exactly as
  `quality` and `sats` are.
* `quality 0` is a property of a **place**, not of a message: `GpsSensor.sky()` says what the sky at a
  position is worth, and a receiver standing where it is worth 0 sends nothing at all — `fix()` answers
  `None` and no `/gps` message exists to carry the number. Where 0 does appear is the answer `/sensor/info`
  gives, the `q_gps` column of the log and the readout line, all three of which ask the receiver rather
  than the last message (that is `engine.gps_health()`, and `types.Gps` documents what a message can
  and cannot contain).
* `odom.jitter` stamps each odom message late by up to that fraction of its period (`Noise.late`,
  one-sided, so stamps never overtake each other). The values are integrated every physics step
  either way, so the message rate and the numbers stay: measured σ of the distance between stamps
  2.8 ms around the 20 ms period, message count identical.

Off means off, and that is measured rather than asserted: `Noise.chance(0.0)` and `Noise.late(0.0)`
return without drawing, and every other new term multiplies by 1 or 0 or adds 0.0. With
`DEFAULT_CONFIG` the sensors produce the pre-W3 streams message for message — the 2337 measurements
of one fixed 12 s drive, sha256 `5e3ff35e3bdd5c8367da9d460971702e3850fabe8986c485f5d9636c2789e4fc`,
identical with `config/demo_gps_shadow.json` and `config/demo_odom_error.json` loaded as well
(`tests/test_sensor_reality.py`).

### 6.5 `mecanum_lab/engine.py` [MINE, already written — read only]
```python
SimEngine(world, cfg=None, seed=None)
  .spawn(name, variant="", pose=None) -> Robot        # raises SpawnError
  .despawn(name) -> bool ; .reset() ; .set_task(name) ; .task
  .set_cmd_vel(name, Twist) ; .set_wheel_speeds(name, list4) ; .set_mission_state(name, str)
  .step(dt) -> None      # splits into fixed physics steps, sensor rates internal
  .sub_step -> float     # 1/rate, what a --fixed-step run steps by
  .dropped -> float      # s of sim time the 0.5 s accumulator threw away (warns once)
  .gps_health(name) -> (quality, sats, lost)   # what the receiver knows, also while it is silent
  .drain() -> list[(kind, robot|None, payload)]        # for the bridge to process
  .robots: dict[str, Robot] ; .world ; .t ; .robots_info() ; .world_json()
```
The engine calls physics/sensors **exactly like this** (agent A has to match it):
`physics.make_geometry(cfg["robot"], variant)`,
`physics.Chassis(geom, pose, seed=…)` with **public attributes**
`geom, pose, wheels, twist, contacts` plus `set_wheels(list)`, `step(dt, walls)`;
`sensors.Noise(seed)`, `sensors.OdometrySensor(sensors.odom_geometry(geom, cfg["odom.geometry"]),
noise, cfg["odom"])` with `reset(pose)` and `update(wheels, dt) -> Odom` (called *every* physics
step) — the integrator gets the **believed** geometry, not the chassis one,
`sensors.Lidar(world, noise, cfg["lidar"]).scan(pose) -> Scan`,
`sensors.GpsSensor(noise, cfg["gps"]).fix(pose, t, name) -> Gps`.

Two small wiring jobs belong to the engine because only it knows both sides: the odom stamp is
published with `stamp = t - Noise.late(odom.jitter)/odom.rate` (a report can be late, never early,
so the stamps stay in order — §6.4), and `gps_health()` answers quality, satellites and lost messages
for a robot from the receiver and its counters instead of from the last message, which is the only
way to show "no fix here" and "the radio lost it" apart while neither produces a message.

`spawn()` has one branch for the second drive train, and only one: `steering.is_steering(variant)`
builds `steering.make_chassis(cfg["steering"], variant, pose)` and `steering.build_odometer(...)`
where the lines above build geometry, chassis and integrator. Everything after that is shared,
because both cars answer the same five calls (`set_wheels`, `set_twist`, `step`, `odo_feed`,
`reset_motion`) and both integrators take one argument whose meaning the chassis decides: four wheel
speeds on the mecanum side, the pair `(wheels, commanded_rack_angle)` on the car (§6.12). That is why
`_drive()` hands `cmd_vel` to `chassis.set_twist()` instead of computing inverse kinematics itself,
and why `robots_info()` carries `steer_deg` next to `wheels` — two rack angles are not wheel speeds.
`config_json()` publishes the `steering` block for the same reason: the corner a student plans comes
from the car that is really driving.


### 6.6 `mecanum_lab/render.py` [C]
```python
class Renderer:
    def __init__(self, engine, cfg: dict)
    def draw(self) -> None                 # one frame, HUD included
    def poll(self) -> dict                 # pressed keys/events, see below
    def close(self) -> None
    @property
    def ok(self) -> bool                   # False once the window was closed
```
`poll()` returns among other things `{"quit": bool, "pause": bool, "toggle_lidar": bool,
"toggle_trail": bool, "camera": int|None, "key": str, "menu": str, "resized": (w,h)|None}`.
Keyboard teleop is **not** implemented in render.py (node.py's job, so the renderer stays ROS-free).
Colors come from `Robot.spec.color` (0..1 float RGB), markers from `spec.marker`.
The renderer only reads `engine.robots[*].{pose,scan,odom,wheels,...}` and `engine.world`.

Two helpers belong to the view, not to the simulation: `cam.py` keeps scale and centre (wheel
zoom **at the cursor**, drag pan, resizable window, `f` shows the whole world) and `menu.py`
draws the layer panel. The panel switches drawing only — `show_scan`, `show_trails`, `show_gps`,
`show_kf`, `show_wheels`, `show_velocity`, `show_goal`, `show_hud`, and `show_coverage` (the radio
map of §6.14). Hiding a
layer must never change what `node.run_loop()` publishes; the lidar dots and the scan on
`/<robot>/scan` are deliberately independent, and the scan dots use the robot's own colour so a
crowded hall is still readable.

### 6.7 `mecanum_lab/ros_bridge.py` [B]
```python
class Bus:                       # shared surface for rclpy and stub (stub.py [MINE])
    def pub(self, kind: str, robot: str | None = None) -> Callable[[payload], None]
    def publish(self, topic_name: str, payload) -> None
    def sub(self, kind: str, robot: str | None, cb) -> None       # cb(payload)
    def sub_topic(self, topic_name: str, cb) -> None              # for JSON/debug topics
    def last(self, kind, robot=None) -> tuple[payload | None, float]   # value + age in s
    def service(self, name: str, handler) -> None                 # handler(dict) -> dict
    def call(self, name: str, req: dict, timeout: float = 2.0) -> dict
    def spin(self, timeout: float = 0.01) -> None
    def ok(self) -> bool
def make_bus(kind="auto", node_name="mecanum_sim") -> Bus        # auto: rclpy -> stub.py
```
`stub.StubBus` is given — `ros_bridge.RclpyBus` must offer this surface **verbatim**,
otherwise `robot_io` is not portable. The `kind` values are the keys of
`types.MSG_SPECS`. `payload` is *always* a data object from `types.py` (`Twist`,
`Scan`, `Odom`, `Gps`, `str`, `list[float]`, `Pose` for `truth`); conversion into
real ROS messages happens **only** in `ros_bridge.py`. Build **no** second stub bus in
ros_bridge — use `from . import stub`.

### 6.8 `mecanum_lab/robot_io.py` [D]
```python
class RobotIO:
    def __init__(self, name: str, role="controller")     # connects to the bus (rclpy or stub)
    def spin(self, timeout=0.01) ; def running(self) -> bool ; def sleep(self, sec)
    def cmd_vel(self) -> Twist          # last twist, otherwise None
    def odom(self) -> Odom | None ; def gps(self) -> Gps | None ; def scan(self) -> Scan | None
    def age(self, kind) -> float        # age of the last measurement in s (for watchdogs)
    def send_wheels(self, w: Sequence[float]) -> None
    def publish_cmd_vel(self, vx, vy, omega) -> None
    def set_mission_state(self, s: str) -> None
    def task(self) -> str
    def spawn(self, name, variant="") -> dict ; def robots(self) -> list[dict]
def run_loop(node: RobotIO, on_tick, hz: float = 50) -> None
```
`serve(module)` runs one of three things per tick: `module.mission(rob, task)` when the task is not
the kinematics one, `module.drive(rob)` **once** when the module defines a `drive` and the task is the
kinematics one (no task at all, or `kinematik`), and otherwise the
`cmd_vel -> inverse_kinematics -> send_wheels` pass-through. The middle
one is for a module that waits for no task — a drive demo for the second drive train (§6.12) has no
kinematics to hand over, so running the node *is* the request. It sets `running`, then `done` (or
`failed:<type>`) like a mission does, and after it the pass-through stays switched off: replaying the
demo's last `cmd_vel` through a kinematics forever would drive a parked car into the shelf in front of
it. A module without `drive` — every template, both reference solutions — takes the same path it
took before, in the same order.

### 6.9 CLI `python -m mecanum_lab.node <command>` [B]
```
run     sim + one/many controllers in ONE process (stub, no ROS)   -> quick start
        --world maze --robot alice --controller student/controller_template.py [--headless]
        [--variant steering]     # the Ackermann car instead of a mecanum robot (§5.1, §6.12)
sim     sim alone (rclpy if ROS is present, otherwise stub + a message)
        --world maze --gui --robots alice,bob --task kinematik
controller  one student node only (needs ROS or a running stub bus) --robot alice
teleop  keyboard/twist remote control on cmd_vel          --robot alice
spawn   --name alice ; despawn --name alice ; robots ; reset ; task --name quadrat
grade   grading run  --robot alice [--task kinematik|quadrat|korridor|alle] [--json path]
docs    shows the topics of all robots
```

Pace (every command that steps the simulation): `--speed N` takes N **simulation seconds per wall
second** (default 1 = real time, as in the lab course), `--fixed-step` steps exactly 1/`rate` per
round and never sleeps — as fast as the CPU allows and independent of what else the machine is
doing. Both keep the fixed physics step, so the seed and the measurement series stay the same;
`--fixed-step` additionally sets `MECANUM_FAST=1`, because a student node that paces its own loop
on the wall clock could not keep up otherwise. A run that falls behind the wall clock says so once
with the number of seconds it lost (`run_loop` and `SimEngine.step()` each count their own).

Window or not is decided by two voices, and the flag is the stronger one: `--headless` always ends
the window, otherwise the config key `gui` counts — a file with `"gui": false` gets no window, which
is what it has always promised and never did.

`./lab <command>` (bash) = `python3 -m mecanum_lab.node <command>` with the correct
`PYTHONPATH` and `source /opt/ros/$ROS_DISTRO/setup.bash`, if present.

### 6.10 `mecanum_lab/overlays.py` [integrator]
```python
def zones(rend) -> None                    # gps.zones as hatched shadow, red where blocked
def odom_ghost(rend, robot) -> None        # where odometry thinks the robot is + Δ in m
def skid_marks(rend, robot, dt) -> None    # rubber on the floor while the wheels slip
def sensor_readout(rend, robot) -> list    # [(text, color)]: the gps, lidar and imu segments
def steer_readout(rend, robot) -> list     # the two rack angles of a steered car + the radius
```
`sensor_readout()` returns text for `render._hud()` to place — it draws nothing itself. It is the
place where the sensor's own opinion becomes visible without a second terminal: `gps x=+7.31 y=+6.02
q2 8 sats lost 3`, `lidar 17 beams no echo`, `imu ax=… gz=… 29.5 °C`. Quality and satellite count
come from `engine.gps_health()` (a blackout has no fresh message to read), the temperature from the
last `Imu` message, because `sensor_msgs/Imu` has no temperature field to carry it.

Drawing only: reads `engine.sensor_profile()` and the robot's own messages, never writes to the
bus and never touches physics. State (the fading skid marks) lives on the renderer, not in module
globals, so two windows in one process stay apart. `render.py` calls these functions and owns the
layers (`s` shadow, `o` ghost). `import render` happens inside the functions because
`render` imports this module — no import cycle at load time.

`steer_readout()` is the fourth overlay function and the readout of the second drive train: two rack
angles in degrees plus the radius they imply, `steer +26.6/+22.9 deg  R=1.90 m`. Nothing else on the
HUD says that the car in front of you is turning, and `R = L/tan(delta)` is the one number a driver
of that car has to keep in mind. The wheels themselves are drawn by `render._wheel()`, which gets
their angles from `chassis.wheel_headings` — the mount point stays on the body, only the stroke turns.

### 6.11 English names, deprecated aliases (`tools/germanids.py`) [MINE]

Identifiers, launch arguments and JSON keys are English. The migration renamed them in one go;
where a name was printed in a handout or typed daily by a student, the old spelling still works as
a **deprecated alias** that prints one line on stdout and is otherwise inert. `tools/germanids.py`
fails when a German name appears outside these tables — for the names it reads: definitions, arguments,
attributes, the constants a file assigns at module level, and the short strings that are data. Local
variables are counted on every run and their size is printed in the same summary line (today: 294 spots in 56 identifiers, most in the student example files); `--locals` turns that count into failures, which
is a rename package with the grading runs of both experiments hanging off it. A green run is therefore
not a clean tree — the number in the summary line is the difference, stated where a release gate reads
it.

| launch argument (today) | deprecated alias | | launch argument (today) | deprecated alias |
|---|---|---|---|---|
| `task` | `aufgabe` | | `log_interval` | `protokoll_intervall` |
| `grade` | `bewerten` | | `recording` | `aufzeichnung` |
| `seconds` | `sekunden` | | `log_level` | `log_stufe` |
| `world` | `welt` | | `gps_gap` | `gps_luecke` |
| `truth` | `wahrheit` | | `imu_scale` | `imu_skala` |
| `imu_tilt` | `imu_neigung` | | `imu_tilt_tau` | `imu_neigung_tau` |
| `imu_startup` | `imu_einschwingen` | | `imu_startup_bias` | `imubias_start` |
| `sim_rate` | `physik_rate` | | `kf_q_turn` | `kf_q_gier` |
| `tf_map_to_odom` | `tf_karten_odom` | | `log` | `protokoll` |

`launch/sim.launch.py` and `launch/lab.launch.py` carry only `sekunden`; `launch/kf.launch.py`
carries the whole table (`DEPRECATED` at the top of the file). The English name always wins when
both are given. `ros2 launch launch/kf.launch.py --show-args` lists the English names.

| key in `config/tasks.json` (today) | deprecated key | | key (today) | deprecated key |
|---|---|---|---|---|
| `title` | `titel` | | `points` | `punkte` |
| `world` | `welt` | | `order` | `reihenfolge` |
| `deliverable` | `abgabe` | | `checks` | `prueft` |
| `hints` | `hilfen` | | `phases` | `phasen` |
| `expect` | `erwarte` | | `target` | `ziel` |
| `target_index` | `ziel_index` | | `target_max` | `ziel_max` |
| `source` | `messung` | | `experiment` | `versuch` |
| `kind` | `art` | | `warmup` | `einlauf` |
| `estimate` | `schaetzung` | | `repeat` | `wiederhole` |
| `drive` | `fahrt` | | `duration` | `dauer` |
| `sine` | `sinus` | | `frequency` | `frequenz` |
| `outage` | `luecke` | | `duration_min` | `dauer_min` |
| `error_max` | `fehler_max` | | `improvement_min` | `verbesserung_min` |
| `max_error_max` | `max_fehler_max` | | `dx_abs_max` | `dx_betrag_max` |
| `dy_abs_max` | `dy_betrag_max` | | `yaw_abs_max` | `winkel_betrag_max` |
| `yaw_min` | `winkel_min` | | `yaw_max_deg` | `winkel_max_deg` |
| `lateral_min` | `abstand_min` | | `straight_omega_max` | `geradeaus_omega_max` |
| `time_max` | `zeit_max` | | `path_min` / `path_max` | `weg_min` / `weg_max` |
| `closure_max` | `abschluss_max` | | `contacts_max` | `kontakte_max` |
| `hold_time` | `haltezeit` | | `side` | `seite` |
| `pass_from` | `bestanden_ab` | | | |

`tasks._LEGACY_KEYS` reads them (one `log.warning` per key, today's spelling wins if both are
present) — a threshold never changes value with its name, only its spelling. The grading report
(`--json`) and the measurement log use the English field names of §8 without aliases, because
both are read by scripts the students wrote themselves and a renamed column shows up immediately.

Two further renames keep their old option as an alias, each with its own notice:
`./lab --interval` (was `--log-intervall`), `tools/worldcheck.py --open-max` (was `--offen-max`),
`tools/fastgrade.py --wallclock-max` (was `--wanduhr-max`).

The wheel labels stay `VL/VR/HL/HR` — they are hardware labels printed on the robot and in the
handout. `physics.WHEELS_EN` gives the English reading of the same four indices
(`FL, FR, RL, RR`) and the handout prints both spellings side by side.


### 6.12 `mecanum_lab/steering.py` [integrator] — the second drive train

```python
def is_steering(variant: str) -> bool                     # "steering", "steering-big", ...
def make_geometry(cfg: dict, variant: str) -> SteeringGeometry   # degrees in, radians out
def ackermann(g, delta) -> list            # [front left, front right] in rad, from §5.1
def wheel_speeds(g, v, delta) -> list      # [VL, VR, HL, HR] rad/s, the rolling wheels
def make_chassis(cfg, variant, pose, seed=None) -> Chassis      # what SimEngine.spawn() calls
def build_odometer(true_g, noise, cfg) -> Odometry              # what SimEngine._make_odometer() calls
class SteeringGeometry:   # wheel_base, track, steer_max, steer_rate, v_max, max_accel, tau,
    .lx .ly .r_min        #                      r, footprint_r, slip, name  (all from the config)
class Chassis(physics.Chassis):                                  # state: one speed, one rack angle
    v, delta, steer       # m/s, rad, the two front wheel angles
    def set_wheels(w)  # mean of the four -> v, rack untouched; def set_twist(vx, vy, omega)
    def odo_feed() -> (wheels, commanded_rack_angle) ; def reset_motion() ; def step(dt, walls)
    .wheel_headings -> [delta_L, delta_R, 0, 0]                  # what the view turns each wheel by
class Odometry(sensors.OdometrySensor):                          # integrates its own (v, delta)
    def update(feed, dt) -> Odom
```

A subclass of `physics.Chassis`, not a second file of mechanics: `step()` reuses the collision and
the slip rule of the mecanum chassis (`_collide`), which is the whole point — a car that drives into a
shelf behaves like the robot the students already know, `steering.slip` included. `physics.py` grew
the four hooks for it (§6.3) and stayed behaviour-neutral.

The rate-limited rack is `approach(delta, clamp(command, steer_max), steer_rate * dt)`; the drive is a
first-order lag with `tau` plus `max_accel`, clamped to `v_max`, all from `DEFAULT_CONFIG["steering"]`
so that a variant entry (`steering-big` in `steering.variants`) can change one car without touching the
other. `set_twist` is where `vy` dies: `delta = atan2(omega * L, max(|v|, 0.05))`, one `log.warning`
per robot, and the drive keeps going — a steering car that refused to move would turn a student's one
stray `vy` into an unexplainable standstill. `set_wheels` (four speeds, the lab 1 exercise) takes their
mean as `v` and leaves the rack alone, so a mecanum inverse kinematics aimed at this car produces a
straight line at the average speed instead of an unexplained motion.

`Odometry` is why this file exists at all: four wheel speeds do not carry a steering angle, so the
integrator takes the pair from `odo_feed()` and replays the rack angle it was *told* to take, with the
`steer_max_scale` it believes. There is no encoder that could contradict it — which is the lesson:
measured over 20 s at full lock with the other noise off, scale 1.0 gives 0.0° and 0.00 m of error,
1.1 gives **+44.9°** and 1.12 m, 0.9 gives −42.0° and 1.34 m. `student/steering_example.py` is the
controller that goes with it (rounded rectangle, then LIDAR parking), and the reason the example is a
`drive()` demo: §6.8.

### 6.13 `mecanum_lab/pois.py` [integrator] — Points of Interest, and the counter that finds them

```python
def load_sources(value, world, d0_default: float = 1.0) -> list[Source]   # raises ValueError
class Source:      # name, kind, x, y, activity, range_m, d0
    def distance(self, x, y) -> float
    def intensity(self, x, y) -> float     # activity / (1 + (d/d0)²) inside range, 0 outside
class PoiSensor:
    def __init__(self, sources, noise, cfg=None)     # cfg = the `poi` block
    def read(self, pose) -> Poi | None               # None: this world has no source
```

The second *field* sensor of the lab and the only one that is not about position: the robot measures
how much of something arrives at its antenna. `types.Poi` is the message — a stamp and one number,
nothing else — published on `/<robot>/poi` at `poi.rate` (5 Hz) by one `self._due(...)` block of
`engine._substep()`. `overlays.poi_readout()` puts the same number into the readout line and asks
`SimEngine.loudest_poi()` for the name beside it: the window is the tutor's view, the topic is the
robot's, and only one of the two may hand out answers.

**The field.** `intensity = activity / (1 + (d/d0)²)` up to the source's `range`, 0 beyond it. `d0`
(which a source may carry itself, otherwise `poi.d0`) is where the counter reads half the activity, so
it sets the shape and not the level: half at `d0`, a tenth at `3·d0`, a hundredth at `9·d0`. A ratio of
two readings is therefore a distance estimate — that is the exercise, and it is why the metres are not
published at all: no switch puts the answer on the same topic as the question. The truth view is the
field rings of layer `p` under `debug_truth`, which is drawn by the window and never sent anywhere.

Measured for the source of `config/demo_poi_exploration.json` (`activity 1.0`, `range 4.0`, `d0 1.0`),
4000 readings per spot with the shipped `poi.counts = 400`:

| distance | field | measured mean | σ of one reading | relative |
|---|---|---|---|---|
| 0.5 m | 0.800 | 0.800 | 0.045 | 5.6 % |
| 2 m | 0.200 | 0.200 | 0.022 | 11 % |
| 4 m (= `range`) | 0.0588 | 0.0588 | 0.012 | 21 % |
| 4.5 m | 0 | 0.000 | 0.000 | silence |

**The noise is a counter's, not a dial's.** `counts = intensity · poi.counts` per reading with a Poisson
sigma of `sqrt(counts)`, so the *absolute* sigma falls with the distance while the relative one grows as
`1/sqrt(intensity)`: nearly exact next to the source, a hint at the edge of its range. That is what a
gradient controller has to survive. `poi.counts: 0` switches the counter model off and reports the
field itself (the maths exercise, which is what the tests read). Out of range nothing is counted and
nothing is drawn from the random stream — a 0.0 reading is exactly 0.0 (`Noise.gauss(0.0)` does not ask
the generator), the same rule that keeps the knobs of §6.4 inert.

**No line of sight.** `Source.intensity()` has no world argument at all: no ray, no shadow, no
reflection, because a gamma source does not care about the shelf in front of it. The walls of the arena
appear twice in this story — in `load_sources`, which refuses a source that is not on open floor, and
nowhere in what a reading is measured against. Measured in `production` with the shipped source,
standing at (10.0, 3.0), 3.6 m away behind the corner of a table: the LIDAR beam pointed *at* the source
reports the table at **0.60 m**, the counter reports **0.072 ± 0.014** and does not care. Two sensors,
two different worlds, neither of them wrong — which is why the `p` layer draws a source as a symbol
above the floor like the GPS shadow rather than as another obstacle, and why `debug_truth` adds the
rings at `d0`, at `3·d0` and at the `range`. A ring past the range would promise a reading that the
counter cannot give, so it is not drawn.

**Validation raises.** A source outside the walls, a `range` or `d0` ≤ 0, a negative `activity`, a
`kind` outside `KINDS`, a missing name, or two sources sharing one name is a `ValueError` that names the
source. That is deliberately *not* the `_zones()` rule of §6.4: a malformed shadow zone only makes one
demo less pretty and is dropped, while a source inside a wall is a scenario nobody can solve, and a
student would debug their own controller for an hour to find a typo. It is raised at engine
construction, before the first step.

**Off means off, again.** `pois` is empty in `DEFAULT_CONFIG`, and `SimEngine._make_pois()` then answers
`None` for it rather than a sensor over an empty list: the engine never enters the publish block, so the
message streams of every graded task stayed what they were (`tests/test_sensor_reality.py`, byte for
byte). With a source planted, the counter draws from the one seeded `Noise` of the run and the other
streams *do* shift; `tests/test_world_poi_w5.py` asserts that shift instead of denying it, and
`config/tasks.json` names no source for any graded task.

**On the wire** there is no standard ROS message for a counter, so `/poi` uses the JSON-on-a-String
pattern that `/sim/robots` and `kf/info` already use (§6.7): `ros_bridge.to_ros()` writes the four
fields as one JSON object, `from_ros()` reads them back, and `ros2 topic echo /alice/poi` needs no
custom interface. As with `Gps.quality` and `Scan.missing`, what does not survive is the convenience — a
ROS node parses a string. And the source positions are published nowhere: `engine.poi_sources()` exists
for the window, and `/sim/config` carries the `poi` *block* (how the counter works) but never `pois`
(where the answer is).

**The hall that goes with it** is `worlds/open.txt` (§6.2): 30 × 20 m, border walls, nothing else,
because a wrong wheel constant should be readable as a number before it is readable as a collision.
Measured on 20 m driven straight at 0.5 m/s, seed 1, with `config/demo_open_odrift.json` (GPS switched
off by a `gps.gap` opened to the length of the exercise): the odometry counts **21.01 m** of 20.00 m
and the ghost is **1.00 m** ahead of the robot at 5 % wrong wheel radius — **0.01 m** with honest ones —
and both runs end with **0 wall contacts**.

### 6.14 `mecanum_lab/wifi.py` [integrator] — the radio the commands travel on

```python
def quality_of(rssi, floor = -85.0, good = -50.0) -> float      # 0 at the floor, 1 on the flat part
def access_point(cfg, world) -> tuple | None                    # `ap` wins, else `ap_by_world[world]`
class Wifi:
    def __init__(self, world, ap, noise, cfg=None)              # cfg = the `wifi` block
    def budget(self, x, y, shadow_db = 0.0) -> tuple            # (rssi, q, metres, wall crossings)
    def coverage(self, step = None) -> list          # [(x, y, q, walls)] per world cell, no fade
    def latency(self, quality) -> float                         # s on the wire, quality-dependent
    def admit(self, name, pose, kind, payload, t) -> bool       # may this frame be sent at all?
    def due(self, name, t) -> list                              # frames whose flight time is over
    def step(self, name, pose, dt) -> LinkState                 # timer for the failsafe
    def message(self, name) -> Link | None                      # what /link publishes
    def health(self, name) -> tuple                             # nine numbers for the window
    def forget(self, name), def reset(self)                     # robot gone / run restart
```

Every command in the simulator arrives over a radio, and until this module existed every one of them
arrived instantly and perfectly. The model is a **link budget**, not a WiFi simulation: no channels,
no association, no OFDM, no DHCP, no retransmission — those are fields of their own and the lab has
two other experiments. What it knows is dBm and the three things that decide a link in a hall
(`rssi = tx_dbm − 10·n·log10(d/d0) − k·wall_db + shadow`), which the module docstring derives term by
term. `n = 2.4` costs 7.2 dB per doubling of distance, at 2 m as much as at 16 m; `k` counts the
wall crossings on the straight line AP → robot **with the LIDAR's own ray/rectangle test**
(`sensors._ray_rect`), so a beam and a radio wave cannot disagree about which rectangles exist, and
the painted lanes of `production` (the `-` and `|` of the grid) are paint and cross for free.

**The seam is one `if` in one file.** `engine._deliver()` asks `wifi.admit()` where the plain
assignment used to be, and `engine._step_link()` calls `wifi.due()` where the outbox used to go out
the same tick. `engine.py` says what a robot does with a command; `wifi.py` says whether one arrives
at all. With `wifi.enabled` false — the shipped default — `_build_sensors()` returns `None`, both
calls are not made, and the command path is the old assignment: not one random number is drawn for
the radio, which is why the graded command streams of both experiments are byte for byte what they
were before a radio existed (`tests/test_wifi_w6.py`, on the recorded CSV).

**The output is a decision, not a level.** Below `wifi.link_up_q` (0.15) for `wifi.link_timeout`
(1.5 s) the link counts as down: `mode` flips to `autonomy`, the frames still on the wire are
thrown away — a station that lost its association keeps nothing — and `wifi.autonomy` says what the
robot does on its own (`stop`, or `hold` to keep driving the last command). The same numbers are what
a student program may read: `types.Link` on `/<robot>/link` at `wifi.rate` (`t, quality, rssi_dbm,
ap, up, dropped, latency_ms`), `robot_io.link()` for the accessor and `overlays.link_readout()` for
the readout segment and the bar above the robot. `ap` is in a message about quality on purpose: a
controller that knows where the access point is can drive out of the shadow itself, one threshold
before the failsafe does it for them.

Measured on `production` (AP at (1.0, 2.0), the shipped `config/demo_wifi.json`, no shadow term so
the numbers are reproducible): the spawn pose at (2.25, 2.75) sits at −43.9 dBm, q 1.00; along the
spawn row eastwards −54.6 dBm/q 0.87 at x = 5 m, −62.9/q 0.63 at 10 m, −67.5/q 0.50 at 15 m,
−70.1 dBm/q 0.42 at the far wall — with the wire latency rising from 25 ms to 43 ms and the frame loss
from 1.7 % to 33 %. Nowhere near the failsafe: in an unfurnished line this AP does not die, it degrades,
which is the honest shape of the exercise — `student/link_autonomy_example.py` drives the aisles until
something does give out.

**On the wire** `/link` is JSON on a `std_msgs/msg/String` for the same reason as `/poi` (§6.13): a
link budget has no standard message, and a custom interface would put a colcon build in front of a
student who only wants to read a quality. `ros2 topic echo /alice/link` needs nothing but ROS.

## 7. LOC budgets (a target, not a kill criterion — justify a deviation > 25 %)

Authoritative list is `BUDGET` in `tools/loc.py` (`python3 tools/loc.py` prints the tally).
The table is that output — measured lines / budget of the same run, never a number carried over from
an older table:

| Module | lines / budget | | Module | lines / budget |
|---|---|---|---|---|
| types.py | 560 / 565 | | ros_bridge.py | 510 / 515 |
| stub.py | 109 / 115 | | tf_bcast.py | 125 / 135 |
| engine.py | 644 / 650 | | node.py | 743 / 755 |
| worlds.py | 127 / 135 | | robot_io.py | 286 / 290 |
| physics.py | 180 / 180 | | tasks.py | 211 / 215 |
| sensors.py | 556 / 560 | | grade.py | 669 / 675 |
| render.py | 483 / 485 | | logbook.py | 115 / 118 |
| cam.py | 106 / 115 | | menu.py | 85 / 90 |
| overlays.py | 481 / 490 | | pois.py | 164 / 170 |
| steering.py | 290 / 290 | | wifi.py | 363 / 365 |
| **simulator core (mecanum_lab/)** | **6807 / 6850** | | | |

`loc.py` counts comment and blank lines too, because that is the size a student sees while reading.
The files outside `mecanum_lab/` — the student files, the launch files, `tools/kfplot.py` — are in the
same `BUDGET` dict and are listed with their numbers in CONTRACT-KF §6.

The view grew because it now owns a camera (zoom at the cursor, pan, resizable window) and a
layer menu, and because `tf_bcast.py` is new. `tasks.py` grew with `_LEGACY_KEYS` (§6.11), the
table that keeps an old `config/tasks.json` readable; physics, bus and grading did not grow — the
English sweep renamed identifiers and added no lines. The rule behind the numbers still stands:
nothing that a student must read gets longer without a reason.

The last growth is `sensors.py` (+70), `engine.py` (+55) and `node.py` (+65): the odometry
integrator now believes its own wheel constants (§6.4), which is only useful if the file says what
each term does to a drive, and the pace of a run became a switch instead of an accident of the host
(§9.1). `grade.py` and `physics.py` are untouched — nothing about a graded measurement or about a
wheel equation changed.

The growth after that is the same story one layer further out: `sensors.py` +151 and
`overlays.py` +37 for the second half of what a sensor delivers — quality, satellite count, chip
temperature, which messages never arrived and which beams never came back (§6.4). Most of those
lines are what each term does to a drive, which is the part a student cannot derive from the code.
`types.py` +27 is one comment line per new key, `engine.py` +17 and `logbook.py` +4 are the wiring
and the columns that make the new fields provable, and `render.py` got **3 lines shorter** (469 →
466) because its two readout segments moved into the overlay kit. `grade.py`, `physics.py`,
`node.py` and `robot_io.py` did not change at all: no graded number, no wheel equation, no CLI
option and no student-facing call moved.

The newest growth is a second drive train (§5.1, §6.12), and `steering.py` is new rather than
`physics.py` bigger: 290 lines of bicycle, Ackermann axle and a second integrator, next to the
mecanum equations every graded task of experiment 1 runs on, instead of inside them. Where it did
reach into an existing file, it stayed at the seam — `physics.py` +41 for four hooks and the sentence
saying they are behaviour-neutral, `engine.py` +21 for one branch in `spawn()`/`_make_odometer()` and
the two fields the view and the students need, `types.py` +28 for the `steering` config block with one
line of meaning per number, `robot_io.py` +15 for the `drive()` demo branch, `overlays.py` +26 for the
rack readout, `render.py` +6 to draw the front wheels at their own angles, `node.py` +1 because
`--variant` existed but never reached `spawn()`. `sensors.py`, `grade.py`, `tasks.py`, `logbook.py` and
both reference solutions are untouched: no sensor, no graded threshold and no wheel equation of
experiment 1 changed, which is also why 100/100 and 90/90 are the same numbers they were.

The growth after that is one new sensor and the hall it is demonstrated in (§6.13, §6.2): `pois.py` is
new with 164 lines rather than 164 more lines inside `sensors.py`, because that file is 556 lines about
measuring a *position*, and a counter that is deliberately blind to the walls between it and its source
has nothing in common with the four of them. Where it reaches into an existing file it stays at the seam
— `overlays.py` +71 for the symbol, the field rings and the readout segment, `engine.py` +46 for
building the detector (and answering `None` when nothing is planted, which is what keeps the graded
streams byte-identical) plus one rate-limited publish block, `types.py` +38 for the message and one
comment per new key, `render.py` +3 for the `p` layer, `ros_bridge.py` +10 for the JSON mapping,
`menu.py` +1 for the row, `node.py` +0 because two topic lists only gained the word `poi`.
`tools/worldpic.py` +4 for the fifth panel and the caption of a hall without a goal. `sensors.py`,
`physics.py`, `grade.py`, `logbook.py`, `worlds.py`, both reference solutions and `config/tasks.json`
did not grow by a single line, and the 100/100, the 90/90 and the 30/30 of `tools/check.sh` are the same
three numbers they were before this package.

The two packages after that frame — the radio link (§6.14) and the pass that made these tables tell
the truth — are why the table above no longer shows the numbers the paragraphs below it end on.
Measured with `git show 472aefd:mecanum_lab/<file> | wc -l` against `wc -l` on this tree: `wifi.py` new
with 363 lines, `overlays.py` +197, `engine.py` +163, `node.py` +130, `types.py` +102, `grade.py` +56,
`ros_bridge.py` +28, `robot_io.py` +27, `logbook.py` +11, `render.py` +8, `tasks.py` +3, `menu.py` +1;
`sensors.py`, `physics.py`, `worlds.py`, `stub.py`, `cam.py`, `tf_bcast.py`, `steering.py` and `pois.py`
are the same files they were. One new file and the seams, again: `wifi.py` holds the link budget and the
delivery rule, while `engine.py` gained the seam a command goes through (`_deliver`) and the timer that
flips autonomy. Two of that growth is not the radio at all: `grade.py`'s +56 is the criteria table that
prints the limit it applied, and `node.py`'s is the option surface (`--frame-max`, `--screenshot`, and
the `spawn_player` / `student_nodes` / `timed_run` setup that `cmd_run`, `cmd_sim` and `cmd_grade`
used to each write themselves). The sentence saying *why* a file grew stays in the addendum of its own
package, in the comment above `BUDGET` in `tools/loc.py`: this section is the tally, that one is the
ledger. No threshold of `config/tasks.json` and no wheel equation is in that diff, which is why 100/100
twice, 90/90 and 30/30 are the numbers they were.

## 8. Graded tasks (Experiment 1) — details in `config/tasks.json` [D]

| ID | Name | Checks | Limits, read from `config/tasks.json` |
|---|---|---|---|
| `kinematik` | T1 — inverse kinematics (30 points) | the four wheel equations, the `VL/VR/HL/HR` order, rad/s | one 1.0 s ramp, then three phases of `duration 3.0` s each followed by a `hold_time 1.2` s pause (`grade._plan`), 10 points each: forward `vx 0.3` → `dx_min 0.35`, `dy_abs_max 0.15`, `yaw_abs_max 0.2`; sideways `vy 0.3` → `dy_min 0.35`, `dx_abs_max 0.15`, `yaw_abs_max 0.2`; turn `omega 0.6` → `yaw_min 1.2`, `dx_abs_max 0.35`, `dy_abs_max 0.35` |
| `quadrat` | T2 — square on odometry (30) | waypoints, correcting instead of chasing, the closing pose | `closure_max 0.25` m and `yaw_max_deg 20` back at the start pose, `path_min 3.0` / `path_max 12.0` m, `contacts_max 0`, `timeout 90` s |
| `korridor` | T3 — drive to the gate (30) | ray sectors, braking before the crash, a clean `done` | `target_max 0.30` m at `target "goal"` (= `world.goal`), `contacts_max 0`, `lateral_min 0.16` m — the smallest side gap seen while the robot drives straight (`\|omega\| ≤ straight_omega_max 0.25`, `\|vx\| > 0.05`) — `path_min 1.0` / `path_max 40.0` m, `timeout 120` s |
| `gps_anfahrt` | T4 — bonus with GPS (10) | another source, a dead zone against noise, the target taken from the world | `target_max 0.45` m at `target "spawn"`, `target_index -1` — the **last spawn pose** of the world (`world()['spawns'][-1]`), deliberately not `world.goal` and not the robot's own start; `contacts_max 0`, `path_min 0.5` / `path_max 40.0` m, `timeout 90` s. Why 0.45 rather than 0.30: the same finished solution measures 0.09…0.26 m over the three paces of §9.1 (0.087…0.260 m), so a limit of 0.30 would be decided by when the last command is cut off, not by the GPS — the task's own `deliverable` text and `tests/test_grenzwerte_integrator.py` say the same |

Every limit above is a key of `config/tasks.json`, not a number from an older table. The mission rows
are `MISSION_CRITERIA` in `grade.py`, and `_apply()` skips a bound the task does not name: a task is
measured by exactly the rows it writes itself, and the report prints those rows, so the limit a student
reads is the limit that was applied. The T1 bounds are the `expect` dict of each phase.

Grading runs **over the topics**, never by code analysis: students may
implement however they like, behaviour is what gets measured. T1 checks in the order
vx, vy, omega; for that the grader sets `/sim/task = kinematik` so `mission()`
does not work against itself.

## 9. Quality gate (run it yourself before submitting)

From the repository root, four commands in this order. First the tests — green without ROS and without a
window:

```bash
python3 -m pytest tests -q
```

Then the node beside a simulator that runs it headless:
```bash
SDL_VIDEODRIVER=dummy ./lab run --robot test --controller student/solution.py --headless
```

And the grade itself, in the one-process form — grader, simulator and node on one clock, which is what
the thresholds of `config/tasks.json` are calibrated on (§9.1):

```bash
./lab grade --robot test --task alle           # the reference solution passes all
```

Testability is part of the task: every agent ships its tests in `tests/`,
file name `test_<module>_<agent>.py`, so nothing gets overwritten.

### 9.1 The same grade at three paces (measured, not assumed)

A grading run used to be paced by `time.monotonic()`, so it measured whatever the machine managed:
the same seed came out differently on a loaded host, and whole seconds disappeared at the 0.25 s
clamp of `run_loop` / the 0.5 s accumulator of `SimEngine.step()` without a word. Both now count
what they lost and say it once, and the pace is a switch: `--speed N` (simulation seconds per wall
second) and `--fixed-step` (one physics step per round, no sleeping at all).

Three runs per pace with the reference solution, seed 1, `./lab grade --json`, a 120-core host with
the three runs in parallel — min..max of what the grader printed:

| Metric (experiment 1) | `--speed 1` | `--speed 4` | `--fixed-step` | limit |
|---|---|---|---|---|
| T2 `closure` | 0.181…0.186 | 0.189…0.191 | 0.127…0.175 | 0.25 |
| T2 `yaw_deg` | −3.0…−2.8 | −2.6…−2.4 | −2.7…−2.6 | 20 |
| T2 `path` | 3.82 | 3.80…3.81 | 3.82…3.87 | 3…12 |
| T3 `target_error` | 0.178…0.185 | 0.175…0.177 | 0.177…0.181 | 0.30 |
| T4 `target_error` | 0.094…0.147 | 0.158…0.250 | 0.087…0.260 | 0.45 |
| **points** | 100/100 | 100/100 | 100/100 | — |

Two things to read out of that: no limit had to move (nothing failed at any pace), and T4's spread
does **not** vanish at `--fixed-step` — the simulation stopped depending on the wall clock, the
student node in its thread did not. `tools/check.sh` therefore grades experiment 1 twice: once in
real time (as before) and once at `--speed 4`. Experiment 2 at these paces, and why it cannot be
graded at `--fixed-step` at all: `docs/CONTRACT-KF.md` §5.1.

One honest footnote: in none of those 18 runs did the loss warning fire — not even with 20 busy
processes pinned to the single core the simulation ran on. What varied here was the *timing
granularity* of the loop, not a stall beyond the 0.25 s per round that may be made up; the warning
is covered by a test with a scripted clock (`tests/test_grading_speed.py`) rather than by this host.

### 9.2 Two things a grade is not

**A grade is not a run across two processes.** `grade:=` in a launch file starts the grader in the
simulator and the controller in a second process on a real network; `./lab grade` starts both in one
process on one bus. That is not a detail, because two of the four drive tasks are measured from what the
node *believes* about its own position. Measured on one seed, reference solutions, one command each:

| how the grade was started | experiment 1 | experiment 2 |
|---|---|---|
| `./lab grade` (the gate above) | **100/100** | **90/90** |
| `ros2 launch … grade:=alle` | 70/100 | 0/90 |

Experiment 1 loses T3: its `drive_to()` gives up on its odometry estimate after 4.64 m of the 12.87 m
the corridor is long, so `target_error` comes out at 8.03 m against a limit of 0.30 m — while `path`,
`time` and `contacts` for the same task are within their limits, which is how a run that drove correctly
is graded as a run that arrived nowhere. Experiment 2 is worse: its report for the same reference solution reads `rate of kf/pose 0.0` against
`rate_min = 5.0` — the estimate that is graded, `/<robot>/kf/pose`, arrives in the other process at
nothing a rate counter can see, all four KF tasks read "never estimated", and 90 points become 0. Neither is a reason to change the measurement — a second process on a real network is what
the robot in the lab room is, and the limits were calibrated on the one-process run, so that is what is
handed in and what `tools/check.sh` grades. `grade:=` is for a supervisor who wants to see the wiring.

**A grade is not a grade of a hall nobody named.** The arena and the sensor profile of a task are picked
from the *announced* task, so a run that names only the grade (`./lab sim --grade alle`) announces
nothing and lands in the hall of `config/default.json` — `maze`. Measured:
`./lab grade --task alle --world maze --controller student/solution.py` gives **40/100**, with
`korridor` at 0/30 over thirty-one wall contacts, while the same solution in the `production` its tasks
name gives 100/100. Nothing in that report is wrong; it measures another hall. Since
`node.graded_task()` a run that grades announces what it grades, through every door, and `--world` still
overrides — an override is the point of an override.
