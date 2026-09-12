"""Gemeinsame Datentypen, Themen-Namen und Konfiguration.

Single source of truth fuer den Simulator: alle anderen Module importieren von hier.
Bewusst ohne numpy/pyyaml — nur Stdlib, damit der Code in 20 Minuten lesbar ist.
"""
from dataclasses import dataclass, field
import json
import math
import os
import re

VERSION = "0.1"
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# ----------------------------------------------------------------------------- Grundtypen


@dataclass
class Pose:
    """Pose in Weltkoordinaten: x nach vorn/rechts, y nach links, theta CCW positiv.

    `t` ist der Simulationszeitstempel, den die Engine beim Veraeffentlichen eintraegt —
    die exakte Pose auf /<robot>/truth ist damit ebenso zeitgestempelt wie GPS und Odometrie,
    sonst kann der Bewerter Messpaare nicht zuordnen.
    """
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    t: float = 0.0


@dataclass
class Twist:
    """Kommando-Körpergeschwindigkeit: vx vorwärts, vy seitlich links, omega Drehrate."""
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass
class Rect:
    """Achsenparalleles Rechteck (Wand, Hindernis) in Weltmetern."""
    x0: float
    y0: float
    x1: float
    y1: float


@dataclass
class World:
    """Umgebung: Kollisionsrechtecke, Startposes, Ziel, reine Deko-Linien."""
    name: str = "?"
    cell: float = 0.5
    walls: list = field(default_factory=list)        # list[Rect]
    spawns: list = field(default_factory=list)       # list[Pose]
    goal: Pose | None = None
    markings: list = field(default_factory=list)     # list[(x0,y0,x1,y1)] nur Ansicht
    size: tuple = (10.0, 10.0)                       # (breite, höhe) in m

    def spawn_pose(self, index: int) -> Pose:
        """Startpose für den n-ten Roboter; letzte Pose wird bei Bedarf wiederholt."""
        if not self.spawns:
            return Pose(self.size[0] / 2, self.size[1] / 2, 0.0)
        return self.spawns[min(index, len(self.spawns) - 1)]


@dataclass
class RobotSpec:
    """Was einen Roboter von den anderen unterscheidet (Name, Aussehen, Motorik)."""
    name: str = "robot"
    index: int = 0
    color: str = "red"
    rgb: tuple = (0.9, 0.3, 0.3)                     # 0..1 für pygame und für ROS-Farben
    marker: str = "triangle"
    variant: str = "stock"


@dataclass
class Odom:
    """Odometrie-Messung (Dead Reckoning aus Radgeschwindigkeiten, verrauscht)."""
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    omega: float = 0.0


@dataclass
class Scan:
    """2D-LIDAR-Strahlen, relativ zur Roboterkante, im Uhrzeigersinn von vorn-links."""
    t: float = 0.0
    angle_min: float = 0.0
    angle_increment: float = 0.0
    range_min: float = 0.05
    range_max: float = 8.0
    ranges: list = field(default_factory=list)       # len == beams, inf wenn nichts getroffen


@dataclass
class Gps:
    """Globale Position (UWB/MoCap-Charakter), verrauscht."""
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Imu:
    """6-DOF-Trägheitsmessung im Körperframe (x vorn, y links, z aufwärts).

    `ax/ay/az` ist die **kinesische Beschleunigung** (Specific Force): im Stand auf
    ebener Fläche ist `az ≈ +9,81`, nicht 0 — so tickt eine echte MEMS-IMU, weil der
    Beschleunigungsmesser die Normalkraft und nicht die Bewegung misst. `roll/pitch`
    sind die Neigungen, die ein IMU-Treiber aus der Schwerkraftrichtung schätzt; sie
    gehören zur Nachricht, weil sie die Horizontalachsen verfälschen (siehe sensors).
    """
    t: float = 0.0
    ax: float = 0.0                 # m/s² in Fahrtrichtung
    ay: float = 0.0                 # m/s² nach links
    az: float = 9.81                # m/s² aufwärts (enthält die Schwerkraft)
    gx: float = 0.0                 # rad/s um x
    gy: float = 0.0                 # rad/s um y
    gz: float = 0.0                 # rad/s um z (Gierrate — das Hauptsignal in 2D)
    roll: float = 0.0               # rad
    pitch: float = 0.0              # rad


@dataclass
class Kf:
    """Eigene Zustandsschätzung der Studierenden — der Bewerter misst sie gegen `truth`.

    `sx/sy/sth` sind die dazu angegebene **Standardabweichung** (1σ, nicht Varianz).
    Ohne sie ist der Auftrag `kf_kovarianz` nicht bewertbar: ein Filter, der 5 m
    Unsicherheit meldet und 0,1 m daneben liegt, ist nicht geschätzt, geraten.
    """
    t: float = 0.0
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    sx: float = 0.0
    sy: float = 0.0
    sth: float = 0.0
    n: int = 0                      # Stichproben im Filter (Diagnose, darf 0 sein)


@dataclass
class Robot:
    """Zustand eines Roboters in der Simulation — von Renderer und ROS nur gelesen."""
    spec: RobotSpec = field(default_factory=RobotSpec)
    chassis: object = None                           # physics.Chassis (Besitz der Physik)
    pose: Pose = field(default_factory=Pose)         # Spiegel der Wahrheit (für GUI/Bewerter)
    twist: Twist = field(default_factory=Twist)
    wheels: list = field(default_factory=lambda: [0.0] * 4)
    wheel_cmd: list | None = None                    # letzte Rad-Kommandos (rad/s)
    vel_cmd: Twist | None = None                     # letzte cmd_vel
    mode: str = "pass-through"                       # "pass-through" | "wheels"
    t_cmd: float = -1.0                              # Zeitstempel des letzten Rad-Kommandos
    t_vel: float = -1.0                              # Zeitstempel der letzten cmd_vel
    odom: Odom | None = None
    scan: Scan | None = None
    gps: Gps | None = None
    imu: Imu | None = None
    kf: Kf | None = None                            # Schätzung der Studierenden
    kf_err: float | None = None                     # |kf − truth| in m, nur für HUD/Bewerter
    contacts: int = 0
    distance: float = 0.0                            # zurückgelegter Weg (m)
    mission_state: str = "idle"
    ticks: dict = field(default_factory=dict)        # Sensor-Akkumulatoren, intern


# ------------------------------------------------------------------- Farben und Marker

# Reihenfolge = Reihenfolge der Teilnehmer-Nummern. rgb in 0..1 (pygame * 255).
PALETTE = [
    ("red", (0.95, 0.33, 0.28)), ("blue", (0.32, 0.58, 0.95)),
    ("green", (0.36, 0.82, 0.44)), ("orange", (0.98, 0.66, 0.25)),
    ("violet", (0.72, 0.47, 0.93)), ("cyan", (0.30, 0.83, 0.83)),
    ("yellow", (0.94, 0.87, 0.36)), ("pink", (0.95, 0.5, 0.72)),
    ("grey", (0.7, 0.7, 0.72)), ("lime", (0.65, 0.9, 0.3)),
]
MARKERS = ["triangle", "square", "diamond", "circle", "pentagon", "hexagon",
           "star", "cross"]
VARIANTS = ["stock", "agile", "slow", "fast"]        # Motorvarianten, im HUD sichtbar

NAME_RE = re.compile(r"^[a-z][a-z0-9_]{1,23}$")


def sanitize_name(raw: str) -> str:
    """Teilnehmername prüfen/normalisieren; wirft ValueError bei ungültiger Eingabe."""
    name = str(raw or "").strip().lower().replace("-", "_").replace(" ", "_")
    if not NAME_RE.match(name):
        raise ValueError(
            f"Ungültiger Robotername '{raw}': kleine Buchstaben, Ziffern, '_', "
            f"2-24 Zeichen, erster Zeichen ein Buchstabe.")
    return name


# ------------------------------------------------------------------------- ROS-Themen

# kind -> (ROS-Typ, Topic-Endung, scope) ; scope: "robot" | "sim" | "global"
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
    """Absoluter Themenname, z. B. topic("odom", "alice") -> "/alice/odom"."""
    _, tail, scope = MSG_SPECS[kind]
    if scope == "global":
        return "/" + tail
    if scope == "sim":
        return "/sim/" + tail
    return f"/{robot}/{tail}"


# ------------------------------------------------------------------------- Konfiguration

DEFAULT_CONFIG = {
    "world": "maze",
    "rate": 50,                     # Physik-Schritte pro Sekunde (fixe Schrittweite)
    "gui_rate": 30,
    "gui": True,
    "width": 1120,
    "height": 700,
    "debug_truth": False,           # /<robot>/truth mit exakter Pose veröffentlichen
    "spawn_limit": 12,
    "cmd_timeout": 0.35,            # s: danach gelten Rad-Kommandos als veraltet
    "robot": {
        "lx": 0.14, "ly": 0.13, "r": 0.05, "footprint_r": 0.21,
        "max_speed": 12.0,          # rad/s
        "max_accel": 40.0,          # rad/s^2
        "tau": 0.06,                # s, Motor-Trägheit 1. Ordnung
        "variants": {"stock": {}, "slow": {"max_speed": 8.0, "tau": 0.12},
                     "fast": {"max_speed": 16.0}, "agile": {"max_accel": 80.0,
                                                            "tau": 0.03}},
    },
    "truth": {"rate": 20.0},          # s. debug_truth: Rate der exakten Pose
    "odom": {"rate": 50.0, "sigma_wheel": 0.04, "sigma_xy": 0.0015,
             "sigma_theta": 0.0012, "bias_omega": 0.0},
    "lidar": {"rate": 20.0, "beams": 360, "range_max": 8.0, "range_min": 0.05,
              "sigma": 0.015, "max_walls": 400},
    "gps": {"rate": 5.0, "sigma_xy": 0.06, "sigma_theta": 0.03, "bias_xy": [0, 0],
            "gap": None,              # [start, dauer] in s: in der Zeit gibt es keinen Fix
            "bias_step": None},       # [start, dauer, dx, dy]: springender Bias (Ausreißer)
    # Realistische MEMS-IMU (MPU-6050/ICM-20948-Klasse). Dichten in Einheit/√Hz,
    # Bias-Random-Walk in Einheit/√s — damit ist die Rechung nachvollziehbar.
    "imu": {"rate": 100.0, "gyro_noise": 1.0e-4, "gyro_bias": 5.0e-3,
            "gyro_bias_walk": 2.0e-5, "gyro_scale": 2.0e-3,
            "accel_noise": 2.0e-3, "accel_bias": 0.05, "accel_bias_walk": 5.0e-4,
            "accel_scale": 1.0e-3, "tilt_sigma": 0.006, "tilt_tau": 0.4,
            "vibration": 0.08, "vibration_hz": 16.0, "gravity": 9.81,
            "startup": 0.4, "startup_bias": 0.5},
    # Hinweis für die Studierenden: die Simulation benutzt diesen Block nicht, er ist
    # die Startempfehlung für den eigenen Filter (siehe student/kf_template.py).
    "kf": {"rate": 20.0, "q_acc": 0.6, "q_turn": 0.02, "gps_delay": 0.0},
    "gui_style": {"wall": [0.34, 0.36, 0.42], "floor": [0.13, 0.14, 0.17],
                  "grid": True, "lidar_alpha": 70, "trail_len": 400},
}


def load_config(path: str | None = None, overrides: dict | None = None) -> dict:
    """DEFAULT_CONFIG <- Datei(JSON) <- overrides. Verschachtelt zusammengeführt."""
    cfg = json.loads(json.dumps(DEFAULT_CONFIG))       # tiefe Kopie, billig genug
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
    """Öffentliche Schreibweise von `_merge` — für Modulwechsel während des laufenden Betriebs."""
    _merge(dst, src)


def cfg_get(cfg: dict, path: str, default=None):
    """cfg_get(cfg, "robot.lx") — Lesehilfe für verschachtelte Einstellungen."""
    node = cfg
    for part in path.split("."):
        if not isinstance(node, dict) or part not in node:
            return default
        node = node[part]
    return node


def wrap_angle(a: float) -> float:
    """Winkel auf (-pi, pi] normieren."""
    return (a + math.pi) % (2 * math.pi) - math.pi
