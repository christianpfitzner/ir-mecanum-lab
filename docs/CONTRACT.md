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
| 3 | **All written prose and all UI text in English**: comments, docstrings, log and report text, `config/tasks.json`, handouts, contracts. `tools/langcheck.py` checks it and runs as a step in `tools/check.sh`. Identifiers are English for everything new; a few German-derived names are API and stay (the launch arguments `sekunden`, `aufgabe`, `bewerten`, `wahrheit`, `protokoll`, `aufzeichnung`, ROS topic names, JSON keys, wheel names `VL/VR/HL/HR`, helper names like `simlauf()` or `welt_fuer()`) — renaming those breaks handouts and student code and is a separate, deliberate step. |
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
| `/<robot>/truth` | `geometry_msgs/msg/PoseStamped` | sim → everyone | exact pose, only with `debug_truth: true` |
| `/<robot>/mission_state` | `std_msgs/msg/String` | students → sim/grader | `"idle"`, `"running"`, `"done"`, `"failed:<reason>"` |
| `/sim/robots` | `std_msgs/msg/String` | sim → everyone | JSON: list of robots including color/marker/mode |
| `/sim/task` | `std_msgs/msg/String` | grader → sim → everyone | active task (`kinematik`, `quadrat`, …) |
| `/sim/spawn_robot` | *service*, see §4 | student/supervisor → sim | add a robot |
| `/sim/despawn_robot` | *service*, see §4 | same | remove a robot |
| `/sim/reset` | `std_srvs/srv/Trigger` | same | reset the world |
| `/clock` | `rosgraph_msgs/msg/Clock` | sim → everyone | simulation time (seconds, `use_sim_time`) |

The `wheel_speeds` order is sacred: **index 0=front left (FL), 1=front right (FR), 2=rear left (RL), 3=rear right (RR)**.

**Fallback mode (mandatory, otherwise there is no quick start):** if a robot has
*never* sent `wheel_speeds` and `cmd_vel` arrives, the sim takes the twist
directly ("pass-through"). As soon as the first `wheel_speeds` message arrives, the
robot stays in `wheels` mode forever. The mode is in `/sim/robots` and in the HUD.

## 4. Spawn service (ROS 2 has no built-in string service)

The transport is detected at runtime; the calling side (`robot_io`, `tools/lab`)
notices nothing about it:

1. **Preferred:** own interface `mecanum_lab_interfaces/srv/SpawnRobot`
   (lives in `interfaces/`, `colcon build --packages-select mecanum_lab_interfaces`):
   ```
   string name            # unique, [a-z0-9_]{2,24}
   string variant         # "" = automatic (slow|fast|agile)
   ---
   bool   success
   string message         # error reason or "ok"
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
3. **Stub mode (no ROS):** direct method call `SimEngine.spawn(name, variant)`.

Name rules: `^[a-z][a-z0-9_]{1,23}$`; duplicate → `success=false`,
`message="name already taken"`; `spawn_limit` reached → `message="robot limit reached"`.

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

## 6. Interfaces between modules (implement exactly like this)

### 6.1 `mecanum_lab/types.py` [MINE, already written — read only]
`Pose(x,y,theta)`, `Twist(vx,vy,omega)`, `Rect(x0,y0,x1,y1)`, `World(name,cell,walls,spawns,goal,markings,size)`,
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
`-`/`|` marking line (visible decoration only, no collision).
Robot centre sits in the cell centre; theta from the order: 1=`0°`, 2=`90°`, 3=`180°`, 4=`270°`, then 0° again.
Neighbouring wall cells merge into large rectangles (2-pass, reduces ray tests).

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
out along the smallest penetration depth, the affected wheel speed is set to 0,
`contacts += 1` (only on a new contact). No slip model — document that.

### 6.4 `mecanum_lab/sensors.py` [A]
```python
class Noise:  __init__(self, seed); gauss(sigma); uniform(a)
class OdometrySensor: __init__(self, g, noise, cfg); reset(pose); update(self, wheels, dt) -> Odom
class Lidar: __init__(self, world, noise, cfg); scan(self, pose) -> Scan
class GpsSensor: __init__(self, noise, cfg); fix(self, pose) -> Gps
```
**Naming rule (integrator, after a name collision was found):** the sensor classes are
`OdometrySensor`, `Lidar`, `GpsSensor`; `types.Odom/Scan/Gps` are the messages. One
module must not carry both names — `sensors.Gps` would have shadowed the import of
`types.Gps` and produced `TypeError: unexpected keyword 't'`.
Odometry **computes only from the wheel speeds it is given** (measured values,
noise included), never from the truth pose → drift arises naturally and is
the point of Experiment 1. LIDAR: ray/AABB slab test, hits clamped to `range_max`,
no hit = `inf` → reported in `LaserScan.ranges` as `range_max` (`inf` only
internally). Hits against other robots: no (Experiment 1).

### 6.5 `mecanum_lab/engine.py` [MINE, already written — read only]
```python
SimEngine(world, cfg=None, seed=None)
  .spawn(name, variant="", pose=None) -> Robot        # raises SpawnError
  .despawn(name) -> bool ; .reset() ; .set_task(name) ; .task
  .set_cmd_vel(name, Twist) ; .set_wheel_speeds(name, list4) ; .set_mission(name, str)
  .step(dt) -> None      # splits into fixed physics steps, sensor rates internal
  .drain() -> list[(kind, robot|None, payload)]        # for the bridge to process
  .robots: dict[str, Robot] ; .world ; .t ; .robots_info() ; .world_json()
```
The engine calls physics/sensors **exactly like this** (agent A has to match it):
`physics.make_geometry(cfg["robot"], variant)`,
`physics.Chassis(geom, pose, seed=…)` with **public attributes**
`geom, pose, wheels, twist, contacts` plus `set_wheels(list)`, `step(dt, walls)`;
`sensors.Noise(seed)`, `sensors.OdometrySensor(geom, noise, cfg["odom"])` with
`reset(pose)` and `update(wheels, dt) -> Odom` (called *every* physics step),
`sensors.Lidar(world, noise, cfg["lidar"]).scan(pose) -> Scan`,
`sensors.Gps(noise, cfg["gps"]).fix(pose) -> Gps`.


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
`show_kf`, `show_wheels`, `show_velocity`, `show_markers`, `show_goal`, `show_hud`. Hiding a
layer must never change what `node.simlauf()` publishes; the lidar dots and the scan on
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

### 6.9 CLI `python -m mecanum_lab.node <command>` [B]
```
run     sim + one/many controllers in ONE process (stub, no ROS)   -> quick start
        --world maze --robot alice --controller student/controller_template.py [--headless]
sim     sim alone (rclpy if ROS is present, otherwise stub + a message)
        --world maze --gui --robots alice,bob --task kinematik
controller  one student node only (needs ROS or a running stub bus) --robot alice
teleop  keyboard/twist remote control on cmd_vel          --robot alice
spawn   --name alice ; despawn --name alice ; robots ; reset ; task --name quadrat
grade   grading run  --robot alice [--task kinematik|quadrat|korridor|alle] [--json path]
docs    shows the topics of all robots
```
`./lab <command>` (bash) = `python3 -m mecanum_lab.node <command>` with the correct
`PYTHONPATH` and `source /opt/ros/$ROS_DISTRO/setup.bash`, if present.

## 7. LOC budgets (a target, not a kill criterion — justify a deviation > 25 %)

Authoritative list is `BUDGET` in `tools/loc.py` (`python3 tools/loc.py` prints the tally).
Current frame after the view and TF work:

| Module | LOC | | Module | LOC |
|---|---|---|---|---|
| types.py | 345 | | ros_bridge.py | 490 |
| stub.py | 115 | | tf_bcast.py | 135 |
| engine.py | 340 | | node.py | 540 |
| worlds.py | 135 | | robot_io.py | 255 |
| physics.py | 140 | | tasks.py | 170 |
| sensors.py | 295 | | grade.py | 620 |
| render.py | 455 | | logbook.py | 100 |
| cam.py | 115 | | menu.py | 90 |
| **simulator core (mecanum_lab/)** | **≤ 4300** | | | |

The view grew because it now owns a camera (zoom at the cursor, pan, resizable window) and a
layer menu, and because `tf_bcast.py` is new. Physics, bus and grading did not grow. The rule
behind the numbers still stands: nothing that a student must read gets longer without a reason.

## 8. Graded tasks (Experiment 1) — details in `config/tasks.json` [D]

| ID | Name | Checks | Success criterion (default) |
|---|---|---|---|
| `kinematik` | T1 | IK signs/wheel assignment | 3 phases of 3 s each (vx=0.3 / vy=0.3 / ω=0.6): Δx>+0.35, \|Δy\|<0.12, \|Δθ\|<0.18 rad etc. |
| `quadrat` | T2 | control loop + odometry | 1 m sides, 90° turns, back within 0.20 m / 15° of the start, time < 90 s |
| `korridor` | T3 | LIDAR look-ahead | reach the goal without a wall contact (`contacts == 0`), lateral distance 0.25–0.8 m |
| `gps_anfahrt` | T4 (bonus) | GPS instead of odometry | reach the goal from `world.goal` with GPS feedback, \|error\| < 0.25 m |

Grading runs **over the topics**, never by code analysis: students may
implement however they like, behaviour is what gets measured. T1 checks in the order
vx, vy, omega; for that the grader sets `/sim/task = kinematik` so `mission()`
does not work against itself.

## 9. Quality gate (run it yourself before submitting)

```bash
cd mecanum-lab
python3 -m pytest tests -q                     # must be green, without ROS, without a window
SDL_VIDEODRIVER=dummy ./lab run --robot test --controller student/solution.py --headless
./lab grade --robot test --task alle           # the reference solution passes all
```
Testability is part of the task: every agent ships its tests in `tests/`,
file name `test_<module>_<agent>.py`, so nothing gets overwritten.
