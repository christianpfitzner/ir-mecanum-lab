"""Second drive train: a car with Ackermann steering, next to the mecanum robot.

A mecanum robot can follow any body velocity it is told — it has no preferred direction. This
chassis cannot: it has one driven axle and one steered axle, so its velocity has **one** degree of
freedom (metres per second along the body x-axis) and a turn rate that is not free but follows from
the steering angle. Three numbers describe the whole car:

    wheel_base L            front axle to rear axle
    steer_max  delta_max    the physical end stop of the rack
    R_min = L / tan(delta_max)      the smallest turning radius, 1.60 m with the defaults

Everything below is a consequence of those:

    omega = v * tan(delta) / L              kinematic bicycle, signs as in CONTRACT section 5
    R     = L / tan(delta)                  the turning radius the car drives at this angle
    delta = atan2(omega * L, |v|)           the same equation turned around (cmd_vel input)

`x` forward, `y` to the left, theta counter-clockwise — the same right-handed system as the
mecanum side, so a left turn is positive omega here too. Wheel order stays
[VL, VR, HL, HR] = [FL, FR, RL, RR] because those labels are printed on the handout.

Two limits of this drive are teaching elements rather than bugs, and both say what they do:
`vy` from `cmd_vel` has no solution on an Ackermann car (warning once per robot, the drive keeps
driving), and the odometry has no steering encoder, so it replays the angle the rack was *told* to
take and a wrong end stop becomes a yaw error that never averages out.
"""
import logging
import math
from dataclasses import dataclass, fields

from . import physics, sensors
from .types import Odom, wrap_angle

log = logging.getLogger("mecanum.steering")
V_FLOOR = 0.05        # m/s: below this a turn command pre-steers instead of dividing by ~0
VY_TOLERANCE = 1e-9   # m/s: what still counts as "no strafe commanded"


@dataclass
class SteeringGeometry:
    """The car: wheel base, track, rack end stop, drive limits.

    The config speaks degrees (`steer_max_deg`, `steer_rate_deg_s`) because that is what a
    workshop speaks; this class holds radians, because that is what `math.tan` takes.
    """
    wheel_base: float = 1.0                       # m, front axle to rear axle (the L of L/tan)
    track: float = 0.62                           # m, distance between the wheels of one axle
    steer_max: float = math.radians(32.0)         # rad, physical end stop of the rack
    steer_rate: float = math.radians(60.0)        # rad/s, how fast the rack can move
    v_max: float = 0.8                            # m/s, drive
    max_accel: float = 2.0                        # m/s^2 (metres here: one motor, not four)
    tau: float = 0.08                             # s, first-order lag of the drive
    r: float = 0.05                               # m, drive wheel radius
    footprint_r: float = 0.62                     # m, collision circle around the centre
    slip: float = 1.0                             # 1 = the drive wheels spin on at a wall
    name: str = "steering"

    @property
    def lx(self):
        """Half the wheel base — the view sizes the chassis with lx/ly (CONTRACT 6.6)."""
        return self.wheel_base / 2

    @property
    def ly(self):
        """Half the track."""
        return self.track / 2

    @property
    def r_min(self):
        """Smallest turning radius this car can do: L / tan(delta_max)."""
        return self.wheel_base / math.tan(self.steer_max)


def is_steering(variant: str) -> bool:
    """Is this variant name one of the steered cars? `steering-big` and friends included."""
    return str(variant or "").startswith("steering")


def make_geometry(cfg: dict, variant: str = "steering") -> SteeringGeometry:
    """Layer a car size over the base steering configuration.

    Same layering as `physics.make_geometry`: keys of the variant win, unknown variants are the
    base car, so `spawn(name, variant="steering-big")` asks for the bigger entry of the variant
    table in `types.DEFAULT_CONFIG["steering"]["variants"]` without any code change.
    """
    base = {k: v for k, v in (cfg or {}).items() if isinstance(v, (int, float))}
    for key, val in ((cfg or {}).get("variants") or {}).get(variant, {}).items():
        base[key] = val
    base["steer_max"] = math.radians(base.pop("steer_max_deg", 32.0))
    base["steer_rate"] = math.radians(base.pop("steer_rate_deg_s", 60.0))
    base["name"] = variant or "steering"
    keep = {f.name for f in fields(SteeringGeometry)}
    return SteeringGeometry(**{k: v for k, v in base.items() if k in keep})


def clamp(value: float, limit: float) -> float:
    """Keep `value` inside [-limit, limit] — the one clamp this module needs a lot."""
    return max(-limit, min(limit, value))


def approach(now: float, target: float, step: float) -> float:
    """Move `now` towards `target`, at most `step` per call (rate limit, no overshoot)."""
    return now + clamp(target - now, step)


def ackermann(g: SteeringGeometry, delta: float) -> list:
    """Angles of the two front wheels [left, right] for the axle angle `delta`.

    Both front wheels must turn about the same point on the rear axle line, so the inner one needs
    the bigger angle: with R = L / tan(delta) and half the track on each side, the wheel `half`
    towards the inside of the turn drives `atan(L / (R - half))` and the wheel `half` to the outside
    `atan(L / (R + half))`. Driving both front wheels with the same angle — the usual shopping-cart
    version of this mistake — makes the inner wheel slide sideways.
    """
    inv_r = math.tan(delta) / g.wheel_base                  # 1 / R, signed
    half = g.track / 2
    return [math.atan2(g.wheel_base * inv_r, 1 - half * inv_r),
            math.atan2(g.wheel_base * inv_r, 1 + half * inv_r)]


def wheel_speeds(g: SteeringGeometry, v: float, delta: float) -> list:
    """The four rolling speeds [VL, VR, HL, HR] in rad/s that the state (v, delta) implies.

    Every wheel rolls without sliding, so its speed is its distance from the turning centre times
    the yaw rate. On the rear axle that distance is R - W/2 for the left wheel and R + W/2 for the
    right one (a positive `delta` is a left turn), on the front axle the hypotenuse of that with L.
    Written with 1/R instead of R so that driving straight (R infinite) is one line and not a
    special case. Straight ahead all four are v / r; in a corner the inner wheel is measurably
    slower than the outer one, which is the Ackermann picture in numbers.
    """
    inv_r = math.tan(delta) / g.wheel_base
    half = g.track / 2
    left, right = 1 - half * inv_r, 1 + half * inv_r        # rear wheels, as a factor of v
    turn = g.wheel_base * inv_r
    return [v * math.hypot(turn, left) / g.r, v * math.hypot(turn, right) / g.r,
            v * left / g.r, v * right / g.r]


class Chassis(physics.Chassis):
    """Truth of a steered car: one drive, one steered axle.

    The state is `(v, delta)` and not four wheel speeds, because that is what this drive can
    command. The four wheel speeds are an *output* of the model (`wheel_speeds()`), published with
    the usual labels; the two front wheel **angles** are not speeds and do not fit into that list,
    so they travel separately as `chassis.steer` and as `steer_deg` on `/sim/robots`.

    The obstacle rule is the mecanum one, inherited unchanged: the body is blocked, the drive is
    not, so `physics.Chassis._collide` leaves the wheels at `slip` times what the motor demands
    while the pose stands still. Only the *drive* wheels can demand anything here, which is why a
    steering robot that grinds against a wall drifts in distance *and* in yaw.
    """

    def __init__(self, g: SteeringGeometry, pose, seed: int | None = None):
        super().__init__(g, pose, seed)
        self.v = 0.0                                # m/s, actual drive speed
        self.v_cmd = 0.0                            # m/s, what the drive is told
        self.delta = 0.0                            # rad, actual rack angle
        self.steer_cmd = 0.0                        # rad, what the rack is told (the odom feed)
        self.steer = [0.0, 0.0]                     # rad, front wheel angles [left, right]
        self._vy_seen = False
        log.info("steering car '%s': wheel base %.2f m, rack end stop %.0f deg -> minimum "
                 "turning radius %.2f m", g.name, g.wheel_base, math.degrees(g.steer_max),
                 g.r_min)

    @property
    def wheel_headings(self) -> list:
        """Turn angle of each wheel in the body frame — the view draws the front pair with it."""
        return [self.steer[0], self.steer[1], 0.0, 0.0]

    def set_wheels(self, wheels) -> None:
        """Four rolling speeds in rad/s — accepted, but they cannot steer anything.

        This drive has one motor, so four wheel speeds carry one quantity: their mean becomes the
        drive speed and the rack stays where it is. A student who sends mecanum inverse-kinematics
        output here drives a straight line at the average of the four, which is the honest answer
        to a command for a motion this car does not have.
        """
        vals = [float(w) for w in list(wheels)[:4]]
        vals += [0.0] * (4 - len(vals))
        self.v_cmd = sum(vals) * self.geometry.r / 4.0

    def set_twist(self, vx: float, vy: float, omega: float) -> None:
        """Put a `cmd_vel` onto a chassis that cannot strafe.

        `delta = atan2(omega * L, max(|v|, V_FLOOR))` is the turn-around of omega = v tan(delta)/L
        written so that a standstill command still turns the rack to a sane angle instead of
        dividing by zero. `vy` has no solution here — saying it once per robot is the point, and
        the drive keeps driving, because a simulator that stands still on a stray 0.25 is
        impossible to debug from a window.
        """
        if abs(float(vy)) > VY_TOLERANCE and not self._vy_seen:
            self._vy_seen = True
            log.warning("steering robot cannot strafe, vy=%.2f dropped", float(vy))
        self.v_cmd = float(vx)
        self.steer_cmd = math.atan2(omega * self.geometry.wheel_base,
                                    max(abs(float(vx)), V_FLOOR))

    def odo_feed(self) -> tuple:
        """The two measured quantities of this car: four wheel speeds and the commanded angle."""
        return (self.wheels, self.steer_cmd)

    def reset_motion(self) -> None:
        """Zero speed **and** zero rack: the odometry resets its angle as well, so the car must."""
        super().reset_motion()
        self.v = self.v_cmd = self.steer_cmd = self.delta = 0.0
        self.steer = [0.0, 0.0]

    def step(self, dt: float, walls: list) -> None:
        """One step: rack rate-limited, drive ramped, arc integrated, obstacle check."""
        g = self.geometry
        self.delta = approach(self.delta, clamp(self.steer_cmd, g.steer_max), g.steer_rate * dt)
        target = clamp(self.v_cmd, g.v_max)
        lag = (1.0 - math.exp(-dt / g.tau)) if g.tau > 0 else 1.0
        self.v = clamp(g.v_max, self.v + clamp((target - self.v) * lag, g.max_accel * dt))
        omega = self.v * math.tan(self.delta) / g.wheel_base
        self.twist.vx, self.twist.vy, self.twist.omega = self.v, 0.0, omega
        mid = self.pose.theta + 0.5 * omega * dt          # mid-step angle, as in physics.Chassis
        self.pose.x += self.v * math.cos(mid) * dt
        self.pose.y += self.v * math.sin(mid) * dt
        self.pose.theta = wrap_angle(self.pose.theta + omega * dt)
        self.steer = ackermann(g, self.delta)
        self.wheels = wheel_speeds(g, self.v, self.delta)          # what a wheel encoder reports
        self.cmd = wheel_speeds(g, target, self.delta)             # what the motor is told
        self._collide(walls)


class Odometry(sensors.OdometrySensor):
    """Dead reckoning of the car: it integrates its own `(v, delta)`.

    Two differences to the mecanum integrator, both of them the reason this class exists:

    * the drive speed comes from the mean of the **rear** pair. The bicycle model refers to the
      rear axle, and the two rear wheel speeds average to v exactly, so the longer path of the
      front wheels in a corner is not silently paid for twice.
    * there is no steering encoder. The angle that goes into omega is the one the rack was told to
      take, rate-limited with the believed rate — a replay of the drive model, not a measurement.
      That is what makes `odom.steer_max_scale` interesting: an end stop that is 10 % off turns
      every corner the same wrong way, and a yaw error that systematic never averages out.
    """

    def __init__(self, believed_wheels, true_g: SteeringGeometry, noise, cfg: dict | None = None):
        cfg = cfg or {}
        super().__init__(believed_wheels, noise, cfg)
        self.wheel_base = 2.0 * believed_wheels.lx        # believed, from odom.geometry
        self.steer_max = true_g.steer_max * float(cfg.get("steer_max_scale") or 1.0)
        self.steer_rate = true_g.steer_rate
        self.delta = 0.0                                  # the angle the model believes

    def reset(self, pose) -> None:
        """Straight rack at the odom origin — otherwise the first step reports a turn it never made."""
        super().reset(pose)
        self.delta = 0.0

    def update(self, feed, dt: float) -> Odom:
        """One step on the believed numbers; `feed` is `Chassis.odo_feed()`."""
        wheels, steer_cmd = feed
        measured = [w + self.noise.gauss(self.sigma_wheel) for w in wheels]
        v = self.g.r * sum(measured[2:4]) / 2.0 * self.scale_xy
        self.delta = approach(self.delta, clamp(steer_cmd, self.steer_max), self.steer_rate * dt)
        omega = v * math.tan(self.delta) / self.wheel_base + self.bias_omega
        mid = self.pose.theta + 0.5 * omega * dt
        self.pose.x += v * math.cos(mid) * dt
        self.pose.y += v * math.sin(mid) * dt
        self.pose.theta = wrap_angle(self.pose.theta + omega * dt)
        self.t += dt
        self.twist = (v, 0.0, omega)
        return Odom(t=self.t_offset + self.t,
                    x=self.pose.x + self.bias_xy[0] + self.noise.gauss(self.sigma_xy),
                    y=self.pose.y + self.bias_xy[1] + self.noise.gauss(self.sigma_xy),
                    theta=wrap_angle(self.pose.theta + self.noise.gauss(self.sigma_theta)),
                    vx=v, vy=0.0, omega=omega)


def build_odometer(true_g: SteeringGeometry, noise, cfg: dict | None) -> Odometry:
    """The car's odometry, believed numbers and all.

    `odom.geometry` is the block that already says what a wrong wheel radius or a wrong wheel base
    does (CONTRACT 6.4) — the car hands it a plain `Geometry` with `lx` = L/2, so the two knobs keep
    their meaning. Only the rack end stop needs a key of its own (`odom.steer_max_scale`), because
    nothing on the car measures it.
    """
    cfg = cfg or {}
    believed = sensors.odom_geometry(physics.Geometry(r=true_g.r, lx=true_g.wheel_base / 2,
                                                      ly=true_g.track / 2), cfg.get("geometry"))
    return Odometry(believed, true_g, noise, cfg)


def make_chassis(cfg: dict, variant: str, pose, seed: int | None = None) -> Chassis:
    """Geometry and chassis for one steered car, in the call the engine uses."""
    return Chassis(make_geometry(cfg, variant), pose, seed=seed)
