"""Shared data types, topic names and configuration.

Single source of truth for the simulator: every other module imports from here.
Deliberately without numpy/pyyaml — stdlib only, so the code reads in 20 minutes.
"""
from dataclasses import dataclass, field
import json
import math
import os
import re

VERSION = "0.1"
PACKAGE = "mecanum_lab"            # the name `ros2 launch <PACKAGE> …` and the share/ folder answer to


def data_root(module_file: str | None = None) -> str:
    """Where this installation keeps `config/`, `worlds/` and `config/rviz/`.

    Two layouts share one package name. In the source tree (`./lab`, pytest) the data sits one level
    above `mecanum_lab/`, next to the code. After `colcon build` (`ros2 launch mecanum_lab …`) the package
    lives in `<prefix>/lib/python3*/site-packages/mecanum_lab/` and its data in `<prefix>/share/mecanum_lab`
    — a directory that is *not* above the module at all. Resolving the source-tree path in the installed
    layout does not fail on the spot: it fails as "unknown world 'production'" or "no config/default.json",
    which reads as a broken simulator instead of a missing path. So the layout is asked, not guessed, and
    AMENT_PREFIX_PATH is where an installed ROS 2 says where it installed itself.
    """
    # `module_file` is a parameter for the same reason the layouts are spelled out above: the installed
    # branch cannot be reached from a test that runs inside the source tree, and an untestable fallback is
    # a fallback nobody ever sees until a student does.
    here = os.path.dirname(os.path.dirname(os.path.abspath(module_file or __file__)))
    if os.path.isdir(os.path.join(here, "worlds")):
        return here
    for prefix in [entry for entry in os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep) if entry]:
        share = os.path.join(prefix, "share", PACKAGE)
        if os.path.isdir(os.path.join(share, "worlds")):
            return share
    return here             # nothing found: name the source tree, the one place the message makes sense


ROOT = data_root()

# ----------------------------------------------------------------------------- Core types


@dataclass
class Pose:
    """Pose in world coordinates: x forward/right, y to the left, theta positive CCW.

    `t` is the simulation timestamp the engine fills in on publish — the exact pose
    on /<robot>/truth is then stamped like GPS and odometry, otherwise the grader
    cannot pair the measurement samples.
    """
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    t: float = 0.0


@dataclass
class Twist:
    """Commanded body velocity: vx forward, vy sideways left, omega turn rate."""
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass
class Rect:
    """Axis-aligned rectangle (wall, obstacle) in world meters."""
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass
class World:
    """Environment: collision rectangles, spawn poses, goal, decorative lines only."""
    name: str = "?"
    cell: float = 0.5
    walls: list = field(default_factory=list)        # list[Rect]
    spawns: list = field(default_factory=list)       # list[Pose]
    goal: Pose | None = None
    markings: list = field(default_factory=list)     # list[(x0,y0,x1,y1)] view only
    size: tuple = (10.0, 10.0)                       # (width, height) in m

    def spawn_pose(self, index: int) -> Pose:
        """Spawn pose for the n-th robot; repeats the last pose when needed."""
        if not self.spawns:
            return Pose(self.size[0] / 2, self.size[1] / 2, 0.0)
        return self.spawns[min(index, len(self.spawns) - 1)]


@dataclass
class RobotSpec:
    """What sets one robot apart from the others (name, appearance, motor behavior)."""
    name: str = "robot"
    index: int = 0
    color: str = "red"
    rgb: tuple = (0.9, 0.3, 0.3)                     # 0..1 for pygame and for ROS colors
    marker: str = "triangle"
    variant: str = "stock"


@dataclass
class Odom:
    """Odometry measurement (dead reckoning from wheel speeds, noisy)."""
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass
class Scan:
    """2D LIDAR rays in the body frame: beam 0 straight ahead, then counter-clockwise.

    Same handedness as everything else (CONTRACT §5: x forward, y left, theta CCW) — reading
    it the other way round mirrors the whole exercise, so this is not a detail.

    A beam that hit nothing is `inf` in `ranges` and is counted in `missing`. Both halves matter:
    a clipped reading looks like a wall at `range_max` to anything that only compares numbers, so
    the count is what lets a student say "17 beams came back with no echo".

    `range_max` is the sensor's limit and is spelled that way after `sensor_msgs/msg/LaserScan`, whose
    field this becomes one-to-one in `ros_bridge.py`. It is not the longest reading and not the size of
    the hall; the alternative spelling `max_range` was considered and rejected, because it would have
    been a second name for a field the ROS message already has.
    """
    t: float = 0.0
    angle_min: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.05
    range_max: float = 8.0                          # the sensor's limit: beyond this -> `inf`
    ranges: list = field(default_factory=list)       # len == beams, inf where nothing was hit
    missing: int = 0                                 # how many of those beams are `inf`


@dataclass
class Gps:
    """Global position (UWB/MoCap-like), noisy — and saying how much that number is worth.

    `quality` in a message that exists: 2 good, 1 degraded (multipath bias, inflated σ, too few
    anchors in view). **0 is not in a message, because a receiver that sees nothing sends nothing** —
    `GpsSensor.fix()` answers `None` for such a position and the engine publishes no `/gps` at all
    (`tests/test_sensor_reality.py` pins the resulting gaps). Quality 0 is a property of the *place* a
    robot is standing at, which `GpsSensor.sky()` reports and the window, the log (`q_gps`) and
    `/sensor/info` repeat; on `/gps` there is nothing to report it in.

    So the two cases a filter has to tell apart are "no message" and "a message with quality 1", and
    not two numbers in one message: no radio (gap window, blackout zone, dropped packet) against a fix
    that is worthless where it was taken. This used to claim that a receiver could answer with quality
    0, which no code path ever produced.
    """
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    quality: int = 2              # 2 good · 1 degraded; 0 belongs to a place, not to a message
    sats: int = 8                 # anchors in view (`GpsSensor.sky`), which is where 0 would come from


@dataclass
class Imu:
    """6-DOF inertial measurement in the body frame (x forward, y left, z up).

    `ax/ay/az` is the **proper acceleration** (specific force): standing on level
    ground `az ≈ +9.81`, not 0 — that is how a real MEMS IMU behaves, because the
    accelerometer measures the normal force and not the motion. `roll/pitch` are the
    tilts an IMU driver estimates from the gravity direction; they belong in the
    message because they corrupt the horizontal axes (see sensors).
    """
    t: float = 0.0
    ax: float = 0.0                 # m/s² along the driving direction
    ay: float = 0.0                 # m/s² to the left
    az: float = 9.81                # m/s² upward (includes gravity)
    gx: float = 0.0                 # rad/s about x
    gy: float = 0.0                 # rad/s about y
    gz: float = 0.0                 # rad/s about z (yaw rate — the main signal in 2D)
    roll: float = 0.0               # radius
    pitch: float = 0.0              # radius
    temp: float = 24.0              # °C at the chip: the bias walks with it, see sensors.ImuSensor


@dataclass
class Poi:
    """Reading of the radiation detector at the robot's position (CONTRACT §6.13).

    One message per source reading, and it names the source that contributes the most counts —
    a wide-band counter cannot tell two sources apart, so the name is how the simulation labels
    the loudest contribution rather than something the robot could know.

    `distance` is **not** in a real detector's output: `poi.publish_distance` is off by default and
    the field stays `None`, because turning the intensity series into a distance is the exercise.
    With the key switched on the number is the true metres to that source — the truth/debug view.

    The field model itself is in `pois.py`: `activity / (1 + (d/d0)²)` up to the source's `range`,
    0 beyond it, through the walls. Through, not around: unlike the LIDAR the detector does not
    care what stands between it and the source, which is why a shelf hides a wall and not a source.
    """
    t: float = 0.0
    intensity: float = 0.0            # counts per second, normalised (0.0 = nothing in range)
    name: str = ""                    # which source is loudest ('' = no source in this world)
    distance: float | None = None     # m to its centre, only with `poi.publish_distance`


@dataclass
class Link:
    """The radio link to one robot — the state a student program is allowed to read (CONTRACT §6.14).

    The point of this message is in the `ap` field: the position of the access point is published
    next to the quality, so a controller can drive back into the radio shadow *by itself*, one
    threshold before the failsafe does it for them. That is the difference between a robot that
    reports its link and one that has to be rescued by it.

    `quality` is `wifi.py`'s q (0 at the sensitivity floor, 1 on the flat part of the curve),
    `rssi_dbm` the level it came from, `up` the autonomy decision (`false` once q has been under
    `wifi.link_up_q` for `wifi.link_timeout`), `dropped` every command the radio did not deliver
    since the robot spawned — a lost frame and an unanswerable link look identical from the outside,
    so they share the counter — and `latency_ms` the delay the wire is adding right now.
    """
    t: float = 0.0
    quality: float = 1.0            # 0 = at the floor, 1 = as good as it gets
    rssi_dbm: float = -50.0         # the level that number came from
    ap: tuple = (0.0, 0.0)          # (x, y) of the access point, in world metres
    up: bool = True                 # False: nothing external is delivered at all
    dropped: int = 0                # commands lost or refused since spawn
    latency_ms: float = 0.0         # what the wire costs at this quality


@dataclass
class SensorInfo:
    """What one robot's field instruments say about themselves — the `/sensor/info` message.

    Five numbers that explain a measurement and fit into none of the measurement messages: the GPS
    `quality`, its `sats` and the `lost` count (no field of `geometry_msgs/msg/PoseStamped`), the
    chip `temp` (no field of `sensor_msgs/msg/Imu`) and the LIDAR's `scan_gaps` (`LaserScan` reports
    `range_max` for a beam that came back as nothing, see CONTRACT §6.4). Over ROS those five were
    therefore visible only in the window and in the log; this message is what makes
    `ros2 topic echo /alice/sensor/info` answer the same question the readout line answers.

    It is not a measurement and carries no position: it is the state of the instruments. The same
    five numbers are columns of the log CSV (`q_gps`, `sats_gps`, `lost_gps`, `temp_imu`,
    `noecho_scan`), so a run can be examined after the fact as well as while it is driving.

    The name says sensors and not GPS although four of the five fields are the receiver's, because
    the temperature is the IMU's and the gaps are the LIDAR's: a message called `/gps/info` that
    answers for a chip would be the first thing a student disbelieves about it.
    """
    t: float = 0.0
    quality: int = 0                # of the PLACE the robot is in: 2 usable, 1 degraded, 0 nothing
    sats: int = 0                   # anchors in view (GpsSensor.sky), not "satellites in the sky"
    lost: int = 0                   # messages this receiver discarded since the robot spawned
    latency_ms: float = 0.0         # `gps.latency`, in ms: the delay the wire is configured to cost
    temp: float = 0.0               # IMU chip temperature in °C, see CONTRACT §6.4 and CONTRACT-KF §3
    scan_gaps: int = 0              # beams of the last scan that returned nothing


@dataclass
class Kf:
    """Students' own state estimate — the grader measures it against `truth`.

    `sx/sy/sth` are the **standard deviation** (1σ, not variance) reported with it.
    Without them the graded task `kf_kovarianz` cannot be scored: a filter reporting
    5 m of uncertainty that misses by 0.1 m is not estimated, it is guessed.
    """
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    sx: float = 0.0
    sy: float = 0.0
    sth: float = 0.0
    n: int = 0                      # samples in the filter (diagnostic, may be 0)


@dataclass
class Robot:
    """State of a robot in the simulation — read-only for the renderer and ROS."""
    spec: RobotSpec = field(default_factory=RobotSpec)
    chassis: object = None                           # physics.Chassis (owned by the physics)
    pose: Pose = field(default_factory=Pose)         # mirror of the truth (for GUI/grader)
    twist: Twist = field(default_factory=Twist)
    wheels: list = field(default_factory=lambda: [0.0] * 4)
    wheel_cmd: list | None = None                    # last wheel commands (rad/s)
    vel_cmd: Twist | None = None                     # last cmd_vel
    mode: str = "pass-through"     # "pass-through" | "wheels" | "autonomy" (radio down, wifi.py)
    t_cmd: float = -1.0                              # timestamp of the last wheel command
    t_vel: float = -1.0                              # timestamp of the last cmd_vel
    cmd_keys: float = -1.0             # sim time the window's teleop keys last published; HUD only
    odom: Odom | None = None
    scan: Scan | None = None
    gps: Gps | None = None
    imu: Imu | None = None
    poi: Poi | None = None                          # radiation reading (pois.py), None = no source
    kf: Kf | None = None                            # students' estimate
    kf_err: float | None = None                     # |kf − truth| in m, HUD/grader only
    contacts: int = 0
    distance: float = 0.0                            # distance driven (m)
    mission_state: str = "idle"
    ticks: dict = field(default_factory=dict)        # sensor accumulators, internal


# ------------------------------------------------------------------- Colors and markers

# Order = order of the participant numbers. rgb in 0..1 (pygame * 255).
PALETTE = [
    ("red", (0.95, 0.33, 0.28)), ("blue", (0.32, 0.58, 0.95)),
    ("green", (0.36, 0.82, 0.44)), ("orange", (0.98, 0.66, 0.25)),
    ("violet", (0.72, 0.47, 0.93)), ("cyan", (0.30, 0.83, 0.83)),
    ("yellow", (0.94, 0.87, 0.36)), ("pink", (0.95, 0.5, 0.72)),
    ("grey", (0.7, 0.7, 0.72)), ("lime", (0.65, 0.9, 0.3)),
]
MARKERS = ["triangle", "square", "diamond", "circle", "pentagon", "hexagon",
           "star", "cross"]
# Motor variants a robot is cycle-assigned when `--variant` is empty. The steered cars of
# steering.py are deliberately not in this list: a run without `--variant` stays a mecanum run,
# which is what every threshold in config/tasks.json is calibrated on.
VARIANTS = ["stock", "agile", "slow", "fast"]        # motor variants, visible in the HUD

NAME_RE = re.compile(r"^[a-z][a-z0-9_]{1,23}$")


def sanitize_name(raw: str) -> str:
    """Check and normalize a participant name; raises ValueError on invalid input."""
    name = str(raw or "").strip().lower().replace("-", "_").replace(" ", "_")
    if not NAME_RE.match(name):
        raise ValueError(
            f"Invalid robot name '{raw}': lowercase letters, digits, '_', "
            f"2-24 characters, the first character a letter.")
    return name


# ------------------------------------------------------------------------- ROS topics

# kind -> (ROS type, topic suffix, scope) ; scope: "robot" | "sim" | "global"
MSG_SPECS = {
    "twist":   ("geometry_msgs/msg/Twist",          "cmd_vel",       "robot"),
    "wheels":  ("std_msgs/msg/Float64MultiArray",   "wheel_speeds",  "robot"),
    "odom":    ("nav_msgs/msg/Odometry",            "odom",          "robot"),
    "scan":    ("sensor_msgs/msg/LaserScan",        "scan",          "robot"),
    "gps":     ("geometry_msgs/msg/PoseStamped",    "gps",           "robot"),
    "imu":     ("sensor_msgs/msg/Imu",              "imu",           "robot"),
    "kf":      ("geometry_msgs/msg/PoseWithCovarianceStamped", "kf/pose", "robot"),
    "kfinfo":  ("std_msgs/msg/String",              "kf/info",       "robot"),
    "truth":   ("geometry_msgs/msg/PoseStamped",    "truth",         "robot"),
    # No standard message fits a radiation reading, so /poi uses the JSON-on-a-String pattern of
    # §6.7 that /sim/robots and kf/info already use: the fields of `Poi` as one JSON object.
    "poi":     ("std_msgs/msg/String",              "poi",           "robot"),
    # Same pattern as /poi: no standard message carries a link budget, so /link is the JSON form of
    # types.Link on a String. The autonomy decision belongs on a topic: a student program has to be
    # able to read the same `up` the window draws and the failsafe acts on.
    "link":    ("std_msgs/msg/String",              "link",          "robot"),
    # The five numbers about the instruments themselves, on a String: none of the three standard
    # messages has a place for them (CONTRACT §6.4), so this is the JSON pattern of /poi and /link
    # again, with the same reason — a custom interface would put a colcon build in front of someone
    # who only wants to read a quality and a temperature. Named `/sensor/info` rather than
    # `/gps/info`, because three instruments answer here and not one.
    "sensorinfo": ("std_msgs/msg/String",           "sensor/info",    "robot"),
    "mission": ("std_msgs/msg/String",              "mission_state", "robot"),
    "robots":  ("std_msgs/msg/String",              "robots",        "sim"),
    "world":   ("std_msgs/msg/String",              "world",         "sim"),
    "config":  ("std_msgs/msg/String",              "config",        "sim"),
    "task":    ("std_msgs/msg/String",              "task",          "sim"),
    "clock":   ("rosgraph_msgs/msg/Clock",          "clock",         "global"),
    "spawn":   ("service",                          "spawn_robot",   "sim"),
    "despawn": ("service",                          "despawn_robot", "sim"),
    "reset":   ("std_srvs/srv/Trigger",             "reset",         "sim"),
}


def topic(kind: str, robot: str | None = None) -> str:
    """Absolute topic name, e.g. topic("odom", "alice") -> "/alice/odom"."""
    _, tail, scope = MSG_SPECS[kind]
    if scope == "global":
        return "/" + tail
    if scope == "sim":
        return "/sim/" + tail
    return f"/{robot}/{tail}"


# ------------------------------------------------------------------------- Configuration

DEFAULT_CONFIG = {
    "world": "maze",
    "rate": 50,                     # physics steps per second (fixed step size)
    "gui_rate": 30,
    "gui": True,
    "width": 1120,
    "height": 700,
    "debug_truth": False,           # publish /<robot>/truth with the exact pose
    "spawn_limit": 12,
    "cmd_timeout": 0.35,            # s: after this, wheel commands count as stale
    "robot": {
        "lx": 0.14, "ly": 0.13, "r": 0.05, "footprint_r": 0.21,
        "max_speed": 12.0,          # rad/s
        "max_accel": 40.0,          # rad/s^2
        "tau": 0.06,                # s, 1st-order motor inertia
        "slip": 1.0,                # 1 = wheels spin on at a wall, 0 = ideal static friction
        "variants": {"stock": {}, "slow": {"max_speed": 8.0, "tau": 0.12},
                     "fast": {"max_speed": 16.0}, "agile": {"max_accel": 80.0,
                                                            "tau": 0.03}},
    },
    "truth": {"rate": 20.0},          # s. debug_truth: rate of the exact pose
    "odom": {"rate": 50.0, "sigma_wheel": 0.04, "sigma_xy": 0.0015,
             "sigma_theta": 0.0012, "bias_omega": 0.0,
             # Uneven stamps: a real encoder report reaches the bus when it arrives, not every
             # 1/rate s. Upper bound of that lateness as a fraction of the period, drawn per
             # message and always positive (a stamp can be late, never early, never out of order).
             # Only the stamp moves — the values are integrated every physics step either way.
             "jitter": 0.0,
             # Model error of the *geometry the integrator believes* (sensors.odom_geometry):
             # real odometry is wrong mainly because the wheels are not what the drawing says.
             #   wheel_radius_scale  every driven metre scales with it — the path grows/shrinks,
             #                       yaw included (the whole kinematics divides by r)
             #   lever_scale         the lever arm a = lx + ly you assume; a != truth turns every
             #                       straight command into a arc and lands a full circle rotated
             #   wheel_base_scale    only the vehicle length mis-measured (lx), so yaw and lateral
             #                       mix differently while |v| stays right
             #   scale_xy            body velocity too fast/slow while the yaw rate stays right
             #                       (wrong radius, honest gyro): a circle becomes a spiral
             #   bias_xy             constant offset of the reported pose (wrong odom origin)
             # {} means: the integrator believes the true geometry, i.e. perfect wheel constants.
             "geometry": {},
             # Only the steering car reads this one (steering.Odometry): the rack end stop its
             # model believes, as a factor of the true `steering.steer_max_deg`. There is no
             # steering encoder on a car like this, so an end stop that is 10 % off (linkage play,
             # a stop bolt in the wrong hole) cannot be noticed and turns every corner the same
             # wrong way. 1.0 = the model knows the car; only then is the odometry exact.
             "steer_max_scale": 1.0},
    "lidar": {"rate": 20.0, "beams": 360, "range_max": 8.0, "range_min": 0.05,
              "sigma": 0.015, "max_walls": 400,   # max_walls: segments one scan may use
              # Weakest echo a beam may return, as |cos| of the incidence angle on the surface it
              # hit. 0.0 = every wall reflects like a mirror (experiment 1); 0.25 loses the walls
              # seen at a grazing angle, which is why real lidars miss painted posts.
              "reflectivity_min": 0.0},
    "gps": {"rate": 5.0, "sigma_xy": 0.06, "sigma_theta": 0.03, "bias_xy": [0, 0],
            "delay_ticks": 0,         # deliver each fix N emissions late (ring buffer)
            "dropout": 0.0,           # probability per message that the transport loses it
            "latency": 0.0,           # s on the way, jittered ±50 %; shifts the stamp, not the fix
            "sats": 8,                # anchors in view on open floor -> Gps.sats / quality 2
            "sats_min": 4,            # below this the fix is only degraded (quality 1)
            "gap": None,              # [start, duration] in s: no fix in this window
            "bias_step": None,        # [start, duration, dx, dy]: jumping bias (outlier)
            "zones": []},             # place-based degradation, see sensors.GpsSensor._zones

    # Realistic MEMS IMU (MPU-6050/ICM-20948 class). Densities in unit/√Hz,
    # bias random walk in unit/√s — so the arithmetic stays checkable.
    "imu": {"rate": 100.0, "gyro_noise": 1.0e-4, "gyro_bias": 5.0e-3,
            "gyro_bias_walk": 2.0e-5, "gyro_scale": 2.0e-3,
            "accel_noise": 2.0e-3, "accel_bias": 0.05, "accel_bias_walk": 5.0e-4,
            "accel_scale": 1.0e-3, "tilt_sigma": 0.006, "tilt_tau": 0.4,
            "vibration": 0.08, "vibration_hz": 16.0, "gravity": 9.81,
            "startup": 0.4, "startup_bias": 0.5,
            # Temperature: a MEMS module warms up with its own electronics and the bias walks with
            # it. All three default to "cold and stays cold", because a graded filter must not be
            # tuned against a drift the lab course cannot see: temp_motor = °C the drive adds at
            # full speed, temp_tau = s to reach it, temp_walk/temp_walk_gyro = bias per °C.
            "temp_start": 24.0, "temp_motor": 0.0, "temp_tau": 30.0,
            "temp_walk": 0.0, "temp_walk_gyro": 0.0},
    # Note for the students: the simulation does not tune itself with this block, it is the
    # starting recommendation for their own filter (see student/kf_template.py). The one
    # exception is gps_delay: that is the delay their filter has to live with, so the engine
    # simulates it as a late GPS fix (equivalent to gps.delay_ticks, in seconds).
    "kf": {"rate": 20.0, "q_acc": 0.6, "q_turn": 0.02, "gps_delay": 0.0},
    # Second drive train (`--variant steering`, `spawn(name, variant="steering")`): a kinematic
    # bicycle with Ackermann geometry on the front axle, modelled in steering.py. The angles are
    # in degrees here because that is what a workshop speaks; the code turns them into radians.
    # Nothing in this block is read for the mecanum robot, so no graded number depends on it.
    "steering": {
        "wheel_base": 1.0,            # m, front axle to rear axle: the L of R = L / tan(delta)
        "track": 0.62,                # m, distance between the two front wheels (Ackermann arm)
        "steer_max_deg": 32,          # deg, physical end stop of the rack -> R_min = 1.60 m below
        "steer_rate_deg_s": 60,       # how fast the rack moves: a corner needs lead, not a step
        "v_max": 0.8,                 # m/s, the one drive
        "max_accel": 2.0,             # m/s^2 — metres here, not rad/s: one motor instead of four
        "tau": 0.08,                  # s, first-order lag of that drive
        "r": 0.05,                    # m, drive wheel radius (rad/s of the encoders <-> m/s)
        "footprint_r": 0.62,          # m, collision circle: half the diagonal of the body
        "slip": 1.0,                  # 1 = the drive wheels spin on against an obstacle
        # Sizes a student can ask for later with `rob.spawn("car", variant="steering-big")`.
        "variants": {"steering": {},
                     "steering-big": {"wheel_base": 1.4, "track": 0.8,
                                      "v_max": 1.2, "footprint_r": 0.85}}},
    # Points of Interest (pois.py): the sources of one world, and the detector that reads them.
    # `pois` is empty by default — then the engine builds no detector, publishes no /poi message and
    # draws no random number, so every graded stream stays byte for byte what it was. A world that
    # has sources lists them (validated by pois.load_sources: inside the walls, range > 0, unique
    # names). A demo file that plants a source therefore also names the hall it is planted in
    # (`config/demo_poi_exploration.json`), and `--world production` moves it into the furnished one.
    "pois": [],
    # The detector: messages per second, the `d0` of §6.13 for a source that does not name its own,
    # how many counts one unit of intensity is worth per reading (`counts`: the Poisson sigma is
    # sqrt(counts), so a weak reading is a noisier reading, which is what a counter does), and
    # whether the true distance may travel with the message. Off by default: see types.Poi.
    "poi": {"rate": 5.0, "d0": 1.0, "counts": 400.0, "publish_distance": False},
    # The radio link (`wifi.py`): one access point per hall, and the commands arrive through it.
    # `enabled` is the one option, and it is off: the graded runs of both experiments are calibrated
    # on a link that delivers everything at once, and every threshold in config/tasks.json stays that
    # way. Off also means *nothing happens*: no radio is built, no random number is drawn, no /link
    # message is published — the command stream is byte for byte the one the golden test guards.
    "wifi": {
        "enabled": False,
        # Where the hall's AP hangs. `ap_by_world` is the installer's view — each hall gets the spot a
        # real technician would use, above the loading area / the door the robots start at, never in
        # the middle of the free floor — and is the reason a demo file does not have to know the hall.
        # `ap` (null by default) overrides it for one run: `--set wifi.ap=[18,10]`.
        "ap": None,
        "ap_by_world": {"production": [1.0, 2.0], "arena": [1.0, 1.5], "maze": [1.5, 2.5],
                        "track": [1.5, 1.5], "open": [1.0, 3.0]},
        "rate": 5.0,                # /link messages per second
        "tx_dbm": -40.0,            # level at the reference distance d0, not the radiated power
        "d0": 1.0,                  # m: the metre the level above is measured at
        "n": 2.4,                   # path-loss exponent: 2 free space, ~2.4 furnished hall, 4 concrete
        "wall_db": 12.0,            # per wall *crossing*: a loaded rack, not a painted line
        "floor_dbm": -85.0,         # q = 0 here (receiver sensitivity of a robot module)
        "good_dbm": -50.0,          # q = 1 here (flat part of the rate curve)
        "shadow_db": 3.0,           # amplitude of the slow fade, so a knife edge stops flickering
        "shadow_period": 2.0,       # s between two fade targets
        "latency_ms": 20.0,         # wire delay at q = 1; x3 at q = 0
        "link_up_q": 0.15,          # below this for `link_timeout` the link is down
        "link_timeout": 1.5,        # s of sim time the low level has to last before autonomy
        "autonomy": "stop",         # "stop" (watchdog holds it) | "dead_reckoning" (last cmd stays)
    },
    # Cell size per world: the robot is the same size everywhere, but the maze is built on a
    # coarser grid, so its corridors are wide enough to drive and to see. See worlds.py.
    "worlds": {"cell": 0.5, "cell_by_world": {"maze": 1.0}},
    # TF tree of the simulator (tf_bcast.py): map -> <robot>/odom -> <robot>/base_link. With
    # map_to_odom "odom" the transform stays the identity, so map -> base_link is the drifting
    # odometry — the students close that gap themselves. "truth" is the tutor's view.
    "tf": {"enabled": True, "rate": 20.0, "map_to_odom": "odom", "namespaces": True,
           "mount": {"laser": [0.0, 0.0, 0.0], "imu": [0.0, 0.0, 0.0]}},
    "gui_style": {"wall": [0.34, 0.36, 0.42], "floor": [0.13, 0.14, 0.17],
                  "void": [0.06, 0.065, 0.08], "trail_len": 400, "px_per_meter_min": 40,
                  "wheel_scale": 2.0, "chassis_scale": 1.0},
    # What the window draws when nobody said otherwise. "clean" leaves the four layers of raw
    # measurements empty (scan dots, GPS cross, odometry trail, odometry ghost) — see
    # `render.RAW_LAYERS` — because those numbers are in the readout line, on the topics and in the
    # measurement log either way, and dots a student cannot yet interpret are not a lesson. A demo
    # config that is about one of them names it in `layers`; `--view sensors` or the `m` panel brings
    # all of them back. Nothing here changes what is published: hiding a layer hides a drawing.
    "view": {"profile": "clean", "layers": {}},
}


def load_config(path: str | None = None, overrides: dict | None = None) -> dict:
    """DEFAULT_CONFIG <- `config/default.json` <- file (JSON) <- overrides. Nested dictionaries are merged.

    A `--config` that cannot be found is an **error**, not the default configuration: typed from another
    directory, `--config config/demo_wifi.json` used to run the plain lab setup without saying anything,
    and a demo whose window is indistinguishable from the normal one costs an hour of suspecting one's own
    typing. A name that is relative is therefore also looked up next to the installation's own `config/`,
    which is the one place `ros2 launch mecanum_lab demo_…` can point at from any working directory.
    """
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))       # deep copy, cheap enough
    with open(os.path.join(ROOT, "config", "default.json"), encoding="utf-8") as handle:
        _merge(cfg, json.load(handle))
    if path:
        candidates = [path, os.path.join(ROOT, os.path.basename(path)), os.path.join(ROOT, path)]
        found = next((candidate for candidate in candidates if os.path.exists(candidate)), None)
        if found is None:
            raise FileNotFoundError(f"config file '{path}' is not there — looked in "
                                    + ", ".join(repr(os.path.dirname(c) or ".") for c in candidates[:2]))
        with open(found, encoding="utf-8") as handle:
            _merge(cfg, json.load(handle))
    _merge(cfg, overrides or {})
    return cfg


def _merge(dst: dict, src: dict) -> None:
    """Overlay `src` on `dst`. A value of None means "nothing overridden", never "delete".

    The layers arrive as one tree from the CLI (`{"world": args.world}` with no `--world`
    given is `{"world": None}`), so treating None as a deletion erased the `world` from
    config/default.json on every run without `--world`. A list is the way to say "off":
    `"gap": []` disables the GPS outage, `"zones": []` the shadow zones.
    """
    for key, val in src.items():
        if val is None:
            continue
        elif isinstance(val, dict) and isinstance(dst.get(key), dict):
            _merge(dst[key], val)
        else:
            dst[key] = val


def merge(dst: dict, src: dict) -> None:
    """Public spelling of `_merge` — for module swaps while the system is running."""
    _merge(dst, src)


def cfg_get(cfg: dict, path: str, default=None):
    """cfg_get(cfg, "robot.lx") — reader for nested settings."""
    node = cfg
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def wrap_angle(a: float) -> float:
    """Normalize an angle to (-pi, pi]."""
    return (a + math.pi) % (2 * math.pi) - math.pi
