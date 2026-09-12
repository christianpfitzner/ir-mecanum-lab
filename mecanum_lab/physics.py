"""Mecanum chassis: kinematics, motor inertia, wall collision.

A right-handed system throughout: x forward, y to the **left**, theta
counter-clockwise (CCW) positive. Wheel order is everywhere
[FL, FR, RL, RR], matching what later travels on the `wheel_speeds` topic.
The signs are CONTRACT §5 — they are the point of the exercise.

Slip is modelled where it physically happens: against an obstacle the body stands still while
the wheels keep the speed the motor demands (`robot.slip`, 1 = full slip) — that is the one
place where odometry may lie, because it integrates wheel speeds. Everywhere else four wheel
speeds give the body velocity uniquely (pseudo-inverse of the kinematics). Noise does not belong
in the mechanics but in sensors.py — the truth stays deterministic.
"""
from dataclasses import dataclass, fields
import math

from .types import Pose, Twist, wrap_angle

WHEELS = ("VL", "VR", "HL", "HR")          # index 0..3; the order is contractual


@dataclass
class Geometry:
    """Wheel and motor values of a robot; arm = lever arm for the turn rate."""
    lx: float = 0.14                        # half vehicle length (wheel base front/rear)
    ly: float = 0.13                        # half vehicle width
    r: float = 0.05                         # wheel radius
    max_speed: float = 12.0                 # rad/s per wheel
    max_accel: float = 40.0                 # rad/s² per wheel
    tau: float = 0.06                       # s, first-order time constant
    footprint_r: float = 0.21               # collision circle
    slip: float = 1.0                       # 1 = wheels spin on at a wall, 0 = static friction
    name: str = "stock"

    @property
    def arm(self):
        return self.lx + self.ly


def make_geometry(cfg: dict, variant: str = "stock") -> Geometry:
    """Layer a motor variant ("", stock, slow, fast, agile) over the base configuration."""
    base = {k: v for k, v in (cfg or {}).items() if isinstance(v, (int, float))}
    for key, val in ((cfg or {}).get("variants") or {}).get(variant, {}).items():
        base[key] = val
    base["name"] = variant or "stock"
    keep = {f.name for f in fields(Geometry)}
    return Geometry(**{k: v for k, v in base.items() if k in keep})


def inverse_kinematics(g: Geometry, vx: float, vy: float, omega: float) -> list:
    """Body velocity -> four target speeds [VL, VR, HL, HR] in rad/s."""
    a = g.arm
    return [(vx - vy - a * omega) / g.r, (vx + vy + a * omega) / g.r,
            (vx + vy - a * omega) / g.r, (vx - vy + a * omega) / g.r]


def forward_kinematics(g: Geometry, wheels) -> tuple:
    """Four wheel speeds -> (vx, vy, omega); the exact inverse of the lines above."""
    fl, fr, rl, rr = wheels
    a = g.arm
    return (g.r / 4 * (fl + fr + rl + rr),
            g.r / 4 * (-fl + fr + rl - rr),
            g.r / (4 * a) * (-fl + fr - rl + rr))


class Chassis:
    """Truth of a chassis: actual speeds, pose, body velocity, contacts.

    `seed` is deliberately ignored — this class is noise-free and therefore
    strictly deterministic. Noise only appears in the sensors above it.
    """

    def __init__(self, g: Geometry, pose: Pose, seed: int | None = None):
        self.geom = g
        self.pose = Pose(pose.x, pose.y, pose.theta)
        self.twist = Twist()
        self.wheels = [0.0] * 4             # measured actual speed (rad/s)
        self.cmd = [0.0] * 4                # target speed (clamped)
        self.contacts = 0                   # number of wall contacts (new ones only)
        self.touching = False

    def set_wheels(self, wheels) -> None:
        """Set target speeds, clamped to ±max_speed per wheel."""
        lim, vals = self.geom.max_speed, list(wheels)[:4]
        vals += [0.0] * (4 - len(vals))
        self.cmd = [max(-lim, min(lim, float(v))) for v in vals]

    def step(self, dt: float, walls: list) -> None:
        """One physics step: motor follows command, kinematics, pose integration, wall."""
        g = self.geom
        ramp, lag = g.max_accel * dt, (1.0 - math.exp(-dt / g.tau)) if g.tau > 0 else 1.0
        for i in range(4):
            delta = (self.cmd[i] - self.wheels[i]) * lag
            self.wheels[i] = max(-g.max_speed, min(
                g.max_speed, self.wheels[i] + max(-ramp, min(ramp, delta))))
        vx, vy, omega = forward_kinematics(g, self.wheels)
        self.twist.vx, self.twist.vy, self.twist.omega = vx, vy, omega
        mid = self.pose.theta + 0.5 * omega * dt      # mid-step angle, less drift error
        cos, sin = math.cos(mid), math.sin(mid)
        self.pose.x += (vx * cos - vy * sin) * dt
        self.pose.y += (vx * sin + vy * cos) * dt
        self.pose.theta = wrap_angle(self.pose.theta + omega * dt)
        self._collide(walls)

    def _collide(self, walls: list) -> None:
        """Circle against rectangles: push out, wheels off, count a contact only once."""
        g, p = self.geom, self.pose
        r, hit = g.footprint_r, False
        for w in walls:
            dx = p.x - min(max(p.x, w.x0), w.x1)
            dy = p.y - min(max(p.y, w.y0), w.y1)
            d2 = dx * dx + dy * dy
            if d2 >= r * r:
                continue
            hit = True
            if d2 > 1e-12:                            # an edge or corner is touched
                push = (r - math.sqrt(d2)) / math.sqrt(d2)
                p.x, p.y = p.x + dx * push, p.y + dy * push
            else:                                     # centre inside the rectangle: one
                left, right = p.x - w.x0, w.x1 - p.x        # edge leads out, never diagonal
                down, up = p.y - w.y0, w.y1 - p.y
                kante = min((left, 0), (right, 1), (down, 2), (up, 3))[1]
                if kante == 0:
                    p.x = w.x0 - r
                elif kante == 1:
                    p.x = w.x1 + r
                elif kante == 2:
                    p.y = w.y0 - r
                else:
                    p.y = w.y1 + r
        if hit:
            # The body is blocked, the motor is not: the wheels keep what the motor wants, so
            # odometry integrates metres that were never driven (robot.slip=0 -> standstill).
            self.wheels = [c * g.slip for c in self.cmd]
            self.twist.vx, self.twist.vy, self.twist.omega = 0.0, 0.0, 0.0
            if not self.touching:
                self.contacts += 1
        self.touching = hit
