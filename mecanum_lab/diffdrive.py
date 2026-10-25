"""Third drive train: a differential drive, two wheels on one axle and a caster.

Two driven wheels at y = +b/2 (left) and y = -b/2 (right) on the axle through the centre:

    v     = r/2 * (w_R + w_L)              forward kinematics
    omega = r/b * (w_R - w_L)
    w_L   = (v - omega * b/2) / r          inverse kinematics
    w_R   = (v + omega * b/2) / r

`x` forward, `y` to the left, theta counter-clockwise — CONTRACT §5, the same frame as the other two.
`vy` has no solution: the wheels cannot push sideways, so `cmd_vel` drops it with one warning.

Why a subclass of the mecanum chassis and not a chassis of its own: with the wheels on the axle
(lx = 0, ly = b/2) the mecanum equations of `physics.py` *are* the ones above, as long as each side
carries one speed — left = FL = RL, right = FR = RR. `set_wheels()` enforces exactly that, so the
motor lag, the integration and the wall of `physics.Chassis` stay one implementation for all three.

On `/<robot>/wheel_speeds` this robot reads slot 0 (left) and slot 1 (right); slots 2 and 3 are
ignored, so the message stays the four values of CONTRACT §3. Its encoders report two wheels.
"""
import logging
from dataclasses import dataclass, fields

from . import physics

log = logging.getLogger("mecanum.diffdrive")
ENCODERS = (("wheel_left", 0), ("wheel_right", 1))      # joint name, slot in chassis.wheels


@dataclass
class DiffGeometry(physics.Geometry):
    """A mecanum `Geometry` with the wheels on the axle: lx = 0, ly = b/2, so arm = b/2."""
    lx: float = 0.0
    ly: float = 0.15
    body_lx: float = 0.14                  # half the plate length, for the window only
    name: str = "diff"

    @property
    def track(self):
        """b, the distance between the two wheels."""
        return 2.0 * self.ly


def is_diff(variant: str) -> bool:
    return str(variant or "").startswith("diff")


def make_geometry(cfg: dict, variant: str = "diff") -> DiffGeometry:
    """`types.DEFAULT_CONFIG["diff"]` as a geometry; `track` becomes ly = track / 2."""
    base = {k: v for k, v in (cfg or {}).items() if isinstance(v, (int, float))}
    base["ly"] = float(base.pop("track", 0.30)) / 2.0
    base["name"] = variant or "diff"
    keep = {f.name for f in fields(DiffGeometry)} - {"lx"}
    return DiffGeometry(**{k: v for k, v in base.items() if k in keep})


def inverse_kinematics(g: DiffGeometry, v: float, omega: float) -> list:
    """(v, omega) -> [w_L, w_R] in rad/s."""
    return [(v - omega * g.ly) / g.r, (v + omega * g.ly) / g.r]


class Chassis(physics.Chassis):
    """Truth of a differential drive: one speed per side, no strafing."""

    encoders = ENCODERS
    rollers = False                        # plain tyres: the window draws tread, not 45° rollers

    def __init__(self, g: DiffGeometry, pose, seed: int | None = None):
        super().__init__(g, pose, seed)
        self._vy_seen = False
        log.info("differential drive '%s': track %.2f m, wheel radius %.3f m", g.name, g.track, g.r)

    def set_wheels(self, wheels) -> None:
        """Slot 0 drives the left wheel, slot 1 the right one; 2 and 3 are not read."""
        vals = [float(w) for w in list(wheels)[:2]]
        left, right = (vals + [0.0, 0.0])[:2]
        super().set_wheels([left, right, left, right])

    def set_twist(self, vx: float, vy: float, omega: float) -> None:
        """`cmd_vel` through the differential kinematics; a sideways part cannot be driven."""
        if abs(float(vy)) > 1e-9 and not self._vy_seen:
            self._vy_seen = True
            log.warning("differential drive cannot strafe, vy=%.2f dropped", float(vy))
        self.set_wheels(inverse_kinematics(self.geometry, vx, omega))


def make_chassis(cfg: dict, variant: str, pose, seed: int | None = None) -> Chassis:
    return Chassis(make_geometry(cfg, variant), pose, seed=seed)
