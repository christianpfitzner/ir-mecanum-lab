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
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

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
    """2D LIDAR rays, relative to the robot edge, clockwise from front-left."""
    t: float = 0.0
    angle_min: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.05
    range_max: float = 8.0
    ranges: list = field(default_factory=list)       # len == beams, inf wenn nichts getroffen


@dataclass
class Gps:
    """Global position (UWB/MoCap-like), noisy."""
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


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
    roll: float = 0.0               # rad
    pitch: float = 0.0              # rad


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
    mode: str = "pass-through"                       # "pass-through" | "wheels"
    t_cmd: float = -1.0                              # timestamp of the last wheel command
    t_vel: float = -1.0                              # timestamp of the last cmd_vel
    odom: Odom | None = None
    scan: Scan | None = None
    gps: Gps | None = None
    imu: Imu | None = None
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
        "variants": {"stock": {}, "slow": {"max_speed": 8.0, "tau": 0.12},
                     "fast": {"max_speed": 16.0}, "agile": {"max_accel": 80.0,
                                                            "tau": 0.03}},
    },
    "truth": {"rate": 20.0},          # s. debug_truth: rate of the exact pose
    "odom": {"rate": 50.0, "sigma_wheel": 0.04, "sigma_xy": 0.0015,
             "sigma_theta": 0.0012, "bias_omega": 0.0},
    "lidar": {"rate": 20.0, "beams": 360, "range_max": 8.0, "range_min": 0.05,
              "sigma": 0.015, "max_walls": 400},
    "gps": {"rate": 5.0, "sigma_xy": 0.06, "sigma_theta": 0.03, "bias_xy": [0, 0],
            "gap": None,              # [start, duration] in s: no fix in this window
            "bias_step": None},       # [start, duration, dx, dy]: jumping bias (outlier)
    # Realistic MEMS IMU (MPU-6050/ICM-20948 class). Densities in unit/√Hz,
    # bias random walk in unit/√s — so the arithmetic stays checkable.
    "imu": {"rate": 100.0, "gyro_noise": 1.0e-4, "gyro_bias": 5.0e-3,
            "gyro_bias_walk": 2.0e-5, "gyro_scale": 2.0e-3,
            "accel_noise": 2.0e-3, "accel_bias": 0.05, "accel_bias_walk": 5.0e-4,
            "accel_scale": 1.0e-3, "tilt_sigma": 0.006, "tilt_tau": 0.4,
            "vibration": 0.08, "vibration_hz": 16.0, "gravity": 9.81,
            "startup": 0.4, "startup_bias": 0.5},
    # Note for the students: the simulation does not use this block, it is the
    # starting recommendation for their own filter (see student/kf_template.py).
    "kf": {"rate": 20.0, "q_acc": 0.6, "q_turn": 0.02, "gps_delay": 0.0},
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
                  "wheel_scale": 2.4, "chassis_scale": 1.25},
}


def load_config(path: str | None = None, overrides: dict | None = None) -> dict:
    """DEFAULT_CONFIG <- file (JSON) <- overrides. Nested dictionaries are merged."""
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))       # deep copy, cheap enough
    for p in [os.path.join(ROOT, "config", "default.json"), path]:
        if p and os.path.exists(p):
            with open(p) as fh:
                _merge(cfg, json.load(fh))
    _merge(cfg, overrides or {})
    return cfg


def _merge(dst: dict, src: dict) -> None:
    for key, val in src.items():
        if val is None:
            dst.pop(key, None)
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
