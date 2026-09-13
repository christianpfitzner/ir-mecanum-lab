#!/usr/bin/env python3
"""Reference solution for lab 1 — Mecanum kinematics, odometry, LIDAR, GPS.

Reads like a section of the handout: first the calculation (T1), then a controller that
uses it (T2), then sensing (T3, T4). The same material as in the handout, only executed.
Check it with:

    ./lab grade --robot muster --task alle          # in real time, as in the lab course
    python3 tools/fastgrade.py --task alle --speed 4   # accelerated (instructors)

Convention (handout §2): x forward, y LEFT, theta counter-clockwise,
wheel order [VL, VR, HL, HR] in rad/s.
"""
import math
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import Gps, cfg_get, load_config, wrap_angle

CFG = load_config()
LX, LY, R = (cfg_get(CFG, "robot.lx"), cfg_get(CFG, "robot.ly"), cfg_get(CFG, "robot.r"))
A = LX + LY                                        # lever arm for yaw: a = lx + ly
RADIUS = cfg_get(CFG, "robot.footprint_r", 0.21)
DRIVE = 2 * RADIUS                                 # clearance we intend to keep free
V_MAX, OM_MAX = 0.45, 1.1                          # m/s, rad/s — deliberately below the max
K_POS, K_ROT = 1.1, 2.2                            # P gains [1/s]
TOL_XY, TOL_TH = 0.035, 0.06                        # goal tolerances of the P controller
DIRECTIONS = 24                                    # sectors of the freedom field (15 deg)


# ------------------------------------------------------------- T1: inverse kinematics

def inverse_kinematics(vx, vy, omega):
    """Body velocity -> four wheel speeds [VL, VR, HL, HR] in rad/s.

    Derivation on the example of VL (front left, its roller axis points forward-in): the
    roller allows free motion along its own axis, the drive acts across it. Add the body
    velocity (vx, vy) and the yaw share ω × lever arm and each wheel keeps exactly one
    linear combination — divided by the wheel radius:

        [VL]   1 [ 1  -1  -a ] [  vx  ]
        [VR] = - [ 1  +1  +a ] [  vy  ]        forward kinematics = exact inverse (§2.4)
        [HL]   r [ 1  +1  -a ] [ omega ]
        [HR]     [ 1  -1  +a ]

    Sign checks against the GUI (cost three lines, save two lab hours):
        vx > 0 -> all four positive      (straight ahead)
        vy > 0 -> VL and HR negative     (toward the LEFT side)
        ω > 0  -> VL and HL negative     (turning left)
    """
    return [(vx - vy - A * omega) / R,             # VL
            (vx + vy + A * omega) / R,             # VR
            (vx + vy - A * omega) / R,             # HL
            (vx - vy + A * omega) / R]             # HR


def clamp(value, magnitude):
    return max(-magnitude, min(magnitude, value))


# ------------------------------------------------------------------ Sensor helpers

def clearance_field(scan, directions=DIRECTIONS):
    """Shortest hit per driving direction: 360 beams become 24 sectors.

    Take the minimum per sector, not the mean — whoever averages misses the table edge.
    That one number per direction is all an avoidance controller needs.
    """
    if scan is None or not getattr(scan, "ranges", None):
        return [8.0] * directions
    n, border = len(scan.ranges), scan.range_max
    breite = max(n // directions // 2, 1)
    feld = []
    for k in range(directions):
        mitte = k * n // directions
        feld.append(min(min(scan.ranges[(mitte + i) % n] for i in range(-breite, breite + 1)),
                        border))
    return feld


def vy_lateral(scan, dead_zone=0.04):
    """Small lateral correction from the side sectors keeps a distance to the walls.

    In a 1 m corridor you would otherwise inevitably scrape along one side; the
    difference of the lateral freedom on the left and the right is the lateral offset
    itself. Sign: positive vy means left, so vy has to go negative when the left side
    has less room.
    """
    if scan is None or not getattr(scan, "ranges", None):
        return 0.0
    n, border = len(scan.ranges), scan.range_max
    seitlich = lambda indices: min(min(scan.ranges[i] for i in indices), border)
    left = seitlich([n // 4, n // 5, 3 * n // 10])          # 90°, 72°, 108°
    right = seitlich([3 * n // 4, 4 * n // 5, 7 * n // 10])  # 270°, 288°, 252°
    fehl = (left - right) / 2.0
    if min(left, right) > 1.8 or abs(fehl) < dead_zone:
        return 0.0
    return clamp(1.1 * fehl, 0.24)


class Smoothing:
    """Exponential moving average over x and y — theta is NEVER averaged.

    An angle averaged across ±π gives garbage (the robot spins in circles). Without
    smoothing a P controller at 5 Hz GPS and σ = 6 cm chases every noise spike and
    swings around the goal — that is exactly the learning point of T4.
    """

    def __init__(self, alpha=0.45):
        self.alpha, self.value = alpha, None

    def __call__(self, measured):
        if measured is None:
            return None
        self.value = (measured.x, measured.y) if self.value is None else tuple(
            alt + self.alpha * (fresh - alt) for alt, fresh in zip(self.value, (measured.x, measured.y)))
        return Gps(t=measured.t, x=self.value[0], y=self.value[1], theta=measured.theta)


# ------------------------------------------------------- Controller 1: drive to a pose (T2)

def drive_to_pose(rob, target, read_out, vmax=V_MAX, time_max=20.0, tolerance=TOL_XY, brake=False):
    """P controller for a goal pose; the control output is our own inverse kinematics.

    The position error is measured in the world, the control acts in body velocity, so
    rotate the error by -theta into the body frame:

        x_k =  cos θ · Δx + sin θ · Δy        y_k = -sin θ · Δx + cos θ · Δy

    After that it is pure P: vx = K·x_k, vy = K·y_k, ω = K_θ·Δθ. No integral term needed —
    odometry is itself the memory, and that it drifts is exactly the point of the task.
    `brake=True` lets the LIDAR in front have a say (for driving through a full arena);
    T2 leaves it out, because there only odometry counts by design.
    The deadline runs on the simulation time in the measurement, not on the wall clock.
    """
    ende, vmax, smooth = None, max(vmax, 0.05), Smoothing() if read_out is rob.gps else None
    while rob.running():
        measured = read_out()
        if measured is None:
            rob.spin(0.05)
            continue
        if smooth is not None:
            measured = smooth(measured)
        if ende is None:
            ende = measured.t + time_max
        elif measured.t > ende:
            return False
        dx, dy = target[0] - measured.x, target[1] - measured.y
        along = math.cos(measured.theta) * dx + math.sin(measured.theta) * dy
        across = -math.sin(measured.theta) * dx + math.cos(measured.theta) * dy
        dtheta = wrap_angle(target[2] - measured.theta)
        if math.hypot(dx, dy) < tolerance and abs(dtheta) < TOL_TH:
            rob.publish_cmd_vel(0.0, 0.0, 0.0)
            return True
        vx, vy = clamp(K_POS * along, vmax), clamp(K_POS * across, vmax * 0.7)
        om = clamp(K_ROT * dtheta, OM_MAX)
        if brake:
            feld = clearance_field(rob.scan())
            if feld[0] < DRIVE:                       # obstacle in the way: do not drive
                vx = min(vx, 0.05)                    # into it, turn past it instead
                om = clamp(om + (1.2 if feld[DIRECTIONS // 4] > feld[3 * DIRECTIONS // 4]
                                 else -1.2), OM_MAX)
        rob.publish_cmd_vel(vx, vy, om)
        rob.spin(0.02)
    return False


def drive_square(rob):
    """T2: four sides of 1 m, 90° left turns, back to the start pose — odometry only."""
    start = rob.odom()
    if start is None:
        raise RuntimeError(f"no odometry for '{rob.name}' — is the simulator running?")
    side, ecke, pose = 1.0, math.pi / 2, [start.x, start.y, start.theta]
    for _ in range(4):
        target = [pose[0] + side * math.cos(pose[2]), pose[1] + side * math.sin(pose[2]),
                pose[2]]
        if not drive_to_pose(rob, target, rob.odom, time_max=22.0):
            raise RuntimeError("side not reached — tolerance too tight or time too short")
        pose = [target[0], target[1], wrap_angle(pose[2] + ecke)]
    if not drive_to_pose(rob, [start.x, start.y, start.theta], rob.odom, time_max=22.0):
        raise RuntimeError("did not get back to the start position")


# ------------------------------------------- Controller 2: across the arena, LIDAR-assisted

def drive_to(rob, target, hysteresis="odom", tolerance=0.20, time_max=100.0, vmax=V_MAX):
    """T3 and T4: reach a point with the freedom field — only the source sets them apart.

    Two phases, because both have their own source of error:

    1. Long leg: of the 24 directions of the field only those with at least DRIVE free
       are candidates; of those we take the one closest to the goal direction, with a
       slight preference for open directions (breaks the circling of a table leg, the
       classic mistake of pure potential fields). Speed grows with the freedom of the
       chosen direction — braking happens by itself, before anything hits.
       Against getting wedged: 15 s without 30 cm of approach means "dead end" — then, for
       three seconds, follow only the freest direction (the cheap form of replanning).
    2. Last 1.3 m: there the field controller brakes too hard and stalls; so ease in
       precisely with the pose controller and LIDAR braking.

    `hysteresis` is the learning difference: T3 measures itself by odometry, T4 by smoothed GPS.
    """
    quelle = rob.odom if hysteresis == "odom" else rob.gps
    smooth = Smoothing() if quelle is rob.gps else None
    letzte_entfernung, standstill, ende, umweg_bis, letzte_messt = None, 0.0, None, 0.0, None
    step = 2 * math.pi / DIRECTIONS
    while rob.running():
        measured, scan = quelle(), rob.scan()
        if measured is None or scan is None:
            rob.spin(0.05)
            continue
        if smooth is not None:
            measured = smooth(measured)
        if ende is None:
            ende = measured.t + time_max
        entfernung = math.hypot(target[0] - measured.x, target[1] - measured.y)
        if entfernung < tolerance:
            rob.publish_cmd_vel(0.0, 0.0, 0.0)
            rob.spin(0.4)
            return True
        if entfernung < 1.3:
            return drive_to_pose(rob, [target[0], target[1], measured.theta], quelle, vmax=0.24,
                                  time_max=max(1.0, ende - measured.t), tolerance=min(tolerance, 0.20),
                                  brake=True)
        elapsed = 0.0 if letzte_messt is None else max(0.0, measured.t - letzte_messt)
        letzte_messt = measured.t
        if letzte_entfernung is None:
            letzte_entfernung = entfernung
        elif entfernung < letzte_entfernung - 0.30:
            letzte_entfernung, standstill = entfernung, 0.0
        else:                                  # measure progress in MESSAGE time, not per
            standstill += elapsed          # loop iteration (accelerated runs!)
        umweg = measured.t < umweg_bis
        if not umweg and standstill > 15.0:
            umweg_bis, standstill = measured.t + 3.0, 0.0
        feld = clearance_field(scan)
        fahrbar = [k for k in range(DIRECTIONS) if feld[k] >= DRIVE]
        if not fahrbar:                               # no room anywhere: freest direction
            k = max(range(DIRECTIONS), key=lambda k: feld[k])
            vx, om = 0.0, clamp(1.4 * wrap_angle(k * step), 1.4)
        else:
            zielwinkel = wrap_angle(math.atan2(target[1] - measured.y, target[0] - measured.x)
                                    - measured.theta)

            def costs(k):
                richtung = wrap_angle(k * step)
                if umweg:                             # freedom only, ignore the goal
                    return -feld[k]
                return abs(wrap_angle(zielwinkel - richtung)) + 0.12 * max(0.0, 1.6 - feld[k])

            k = min(fahrbar, key=costs)
            richtung = wrap_angle(k * step)
            vx = vmax * min(1.0, max(0.0, (feld[k] - DRIVE) / 0.55))
            om = clamp(K_ROT * wrap_angle(richtung if umweg
                                          else 0.35 * zielwinkel + 0.65 * richtung), OM_MAX)
        rob.publish_cmd_vel(vx, vy_lateral(scan), om)
        rob.spin(0.02)
        if measured.t > ende:
            raise RuntimeError(f"goal not reached in {time_max:.0f} s, still "
                               f"{entfernung:.2f} m away")
    return False


def drive_corridor(rob):
    """T3: from the start to the goal of the world. Odometry knows the direction, LIDAR the walls."""
    target = (rob.world() or {}).get("goal")
    if not target:
        raise RuntimeError("world has no goal — is G set in worlds/<name>.txt?")
    return drive_to(rob, target, "odom", tolerance=0.20, time_max=100.0)


def drive_to_gps(rob):
    """T4 (bonus): loading spot = the world's last spawn point, driven to with GPS.

    Only the source is new compared to T3: GPS is noisy and slower (5 Hz) -> smoothing
    and a dead zone of 15 cm, so the controller does not swing around the goal. And the
    goal is deliberately NOT your own start pose, otherwise T4 would be handed to you.
    """
    starts = (rob.world() or {}).get("spawns") or []
    if not starts:
        raise RuntimeError("world has no start points (S/2/3/4)")
    return drive_to(rob, starts[-1], "gps", tolerance=0.15, time_max=80.0, vmax=0.30)


# ------------------------------------------------------------------------- Plumbing

def mission(rob, task):
    """Called by the runner exactly once for T2..T4, as long as rob.running().

    Finished = return normally (the runner then reports "done"). Given up = raise an
    exception: the runner reports "failed:<reason>", which is more honest than a
    "done" without reaching the goal.
    """
    for name, action in (("quadrat", drive_square), ("korridor", drive_corridor),
                           ("gps_anfahrt", drive_to_gps)):
        if task.startswith(name):
            return action(rob)
    raise RuntimeError(f"unknown task '{task}'")


if __name__ == "__main__":
    # At T1 serve() passes every cmd_vel through inverse_kinematics; for T2..T4 it
    # calls mission(); publish_cmd_vel stays the only control path.
    robot_io.serve(sys.modules[__name__])
