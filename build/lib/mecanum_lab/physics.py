"""Mecanum-Fahrwerk: Kinematik, Motor-Trägheit, Wandkollision.

Durchgehend ein rechtshändiges System: x in Fahrtrichtung, y nach **links**,
theta gegen den Uhrzeigersinn (CCW) positiv. Radreihenfolge ist überall
[VL, VR, HL, HR] (= FL, FR, RL, RR), so wie sie später auf dem Thema
`wheel_speeds` steht. Die Vorzeichen sind CONTRACT §5 — sie sind der Übungsstoff.

Schlupf wird nicht modelliert: vier Radgeschwindigkeiten ergeben die
Körpergeschwindigkeit eindeutig (Pseudo-Inverse der Kinematik). Rauschen gehört
nicht in die Mechanik, sondern in sensors.py — die Wahrheit ist deterministisch.
"""
from dataclasses import dataclass, fields
import math

from .types import Pose, Twist, wrap_angle

WHEELS = ("VL", "VR", "HL", "HR")          # Index 0..3, Reihenfolge ist Vertrag


@dataclass
class Geometry:
    """Rad- und Motorwerte eines Roboters; arm = Hebelarm für die Drehrate."""
    lx: float = 0.14                        # halbe Fahrzeuglänge (Radmitte vorne/hinten)
    ly: float = 0.13                        # halbe Fahrzeugbreite
    r: float = 0.05                         # Radradius
    max_speed: float = 12.0                 # rad/s pro Rad
    max_accel: float = 40.0                 # rad/s² pro Rad
    tau: float = 0.06                       # s, Zeitkonstante 1. Ordnung
    footprint_r: float = 0.21               # Kollisionskreis
    name: str = "stock"

    @property
    def arm(self):
        return self.lx + self.ly


def make_geometry(cfg: dict, variant: str = "stock") -> Geometry:
    """Motorvariante ("", stock, slow, fast, agile) über die Basiskonfiguration legen."""
    base = {k: v for k, v in (cfg or {}).items() if isinstance(v, (int, float))}
    for key, val in ((cfg or {}).get("variants") or {}).get(variant, {}).items():
        base[key] = val
    base["name"] = variant or "stock"
    keep = {f.name for f in fields(Geometry)}
    return Geometry(**{k: v for k, v in base.items() if k in keep})


def inverse_kinematics(g: Geometry, vx: float, vy: float, omega: float) -> list:
    """Körpergeschwindigkeit -> vier Solldrehzahlen [VL, VR, HL, HR] in rad/s."""
    a = g.arm
    return [(vx - vy - a * omega) / g.r, (vx + vy + a * omega) / g.r,
            (vx + vy - a * omega) / g.r, (vx - vy + a * omega) / g.r]


def forward_kinematics(g: Geometry, wheels) -> tuple:
    """Vier Radgeschwindigkeiten -> (vx, vy, omega); exakte Umkehrung der obigen Zeilen."""
    fl, fr, rl, rr = wheels
    a = g.arm
    return (g.r / 4 * (fl + fr + rl + rr),
            g.r / 4 * (-fl + fr + rl - rr),
            g.r / (4 * a) * (-fl + fr - rl + rr))


class Chassis:
    """Wahrheit eines Fahrwerks: Ist-Drehzahlen, Pose, Körpergeschwindigkeit, Kontakte.

    `seed` wird bewusst ignoriert — diese Klasse ist rauschfrei und damit strikt
    deterministisch. Rauschen kommt erst in den Sensoren darüber.
    """

    def __init__(self, g: Geometry, pose: Pose, seed: int | None = None):
        self.geom = g
        self.pose = Pose(pose.x, pose.y, pose.theta)
        self.twist = Twist()
        self.wheels = [0.0] * 4             # gemessene Ist-Drehzahl (rad/s)
        self.cmd = [0.0] * 4                # Solldrehzahl (begrenzt)
        self.contacts = 0                   # Anzahl Wandberührungen (neue Kontakte)
        self.touching = False

    def set_wheels(self, wheels) -> None:
        """Solldrehzahlen setzen, pro Rad auf ±max_speed begrenzt."""
        lim, vals = self.geom.max_speed, list(wheels)[:4]
        vals += [0.0] * (4 - len(vals))
        self.cmd = [max(-lim, min(lim, float(v))) for v in vals]

    def step(self, dt: float, walls: list) -> None:
        """Ein Physickschritt: Motor follows Befehl, Kinematik, Pose fortschreiben, Wand."""
        g = self.geom
        ramp, lag = g.max_accel * dt, (1.0 - math.exp(-dt / g.tau)) if g.tau > 0 else 1.0
        for i in range(4):
            delta = (self.cmd[i] - self.wheels[i]) * lag
            self.wheels[i] = max(-g.max_speed, min(
                g.max_speed, self.wheels[i] + max(-ramp, min(ramp, delta))))
        vx, vy, omega = forward_kinematics(g, self.wheels)
        self.twist.vx, self.twist.vy, self.twist.omega = vx, vy, omega
        mid = self.pose.theta + 0.5 * omega * dt      # Mitteleckwinkel gegen Driftfehler
        cos, sin = math.cos(mid), math.sin(mid)
        self.pose.x += (vx * cos - vy * sin) * dt
        self.pose.y += (vx * sin + vy * cos) * dt
        self.pose.theta = wrap_angle(self.pose.theta + omega * dt)
        self._collide(walls)

    def _collide(self, walls: list) -> None:
        """Kreis gegen Rechtecke: raus schieben, Räder aus, Kontakt nur einmal zählen."""
        g, p = self.geom, self.pose
        r, hit = g.footprint_r, False
        for w in walls:
            dx = p.x - min(max(p.x, w.x0), w.x1)
            dy = p.y - min(max(p.y, w.y0), w.y1)
            d2 = dx * dx + dy * dy
            if d2 >= r * r:
                continue
            hit = True
            if d2 > 1e-12:                            # Flanke oder Ecke berührt
                push = (r - math.sqrt(d2)) / math.sqrt(d2)
                p.x, p.y = p.x + dx * push, p.y + dy * push
            else:                                     # Mittelpunkt im Rechteck: eine
                links, rechts = p.x - w.x0, w.x1 - p.x      # Kante führt raus, nie diagonal
                unten, oben = p.y - w.y0, w.y1 - p.y
                kante = min((links, 0), (rechts, 1), (unten, 2), (oben, 3))[1]
                if kante == 0:
                    p.x = w.x0 - r
                elif kante == 1:
                    p.x = w.x1 + r
                elif kante == 2:
                    p.y = w.y0 - r
                else:
                    p.y = w.y1 + r
        if hit:
            self.wheels = [0.0] * 4
            # Am Hindernis ist die Körpergeschwindigkeit null: twist aus den stehenden
            # Rädern neu rechnen, sonst meldet der Roboter weiter Fahrt gegen die Wand.
            self.twist.vx, self.twist.vy, self.twist.omega = forward_kinematics(g, self.wheels)
            if not self.touching:
                self.contacts += 1
        self.touching = hit
