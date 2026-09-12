"""Sensors: odometry (dead reckoning), 2D LIDAR, global position, inertial IMU.

All of them compute against the truth from physics.py, but return only what a real
sensor would deliver — which is why odometry drifts, why GPS is noisy and why the IMU
is a bias swamp.

Odometry noise model (kept deliberately simple, so it stays explainable):
* `sigma_wheel`    noise on every measured wheel speed — it is **integrated**,
                   which is where the actual drift comes from (random walk).
* `bias_omega`     systematic turn-rate error, grows linearly with time.
* `sigma_xy`/`sigma_theta` measurement noise on the **output** result; it is not fed
                   back, so the estimate stays free of stairsteps.

IMU noise (experiment 2) is given as a **density** per √Hz and converted to the
sampling rate — see `ImuSensor`, which also explains why the bias wins.
"""
import math
import random

from . import physics
from .types import Gps, Imu, Odom, Pose, Scan, wrap_angle


class Noise:
    """One single random stream for the whole simulation — same seed, same picture."""

    def __init__(self, seed: int | None = None):
        self.rng = random.Random(seed)

    def gauss(self, sigma: float) -> float:
        return self.rng.gauss(0.0, sigma) if sigma else 0.0

    def uniform(self, amp: float) -> float:
        return self.rng.uniform(-amp, amp) if amp else 0.0


class OdometrySensor:
    """Integrates only the wheel speeds it is given, never the truth."""

    def __init__(self, g: physics.Geometry, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.g, self.noise = g, noise
        self.sigma_wheel = cfg.get("sigma_wheel", 0.0)
        self.sigma_xy = cfg.get("sigma_xy", 0.0)
        self.sigma_theta = cfg.get("sigma_theta", 0.0)
        self.bias_omega = cfg.get("bias_omega", 0.0)
        self.pose = Pose()
        self.t = 0.0
        self.t_offset = 0.0                      # the engine hooks this clock to simulation time
        self.twist = (0.0, 0.0, 0.0)

    def reset(self, pose) -> None:
        """The odometry origin is the spawn pose; from there on it only adds."""
        self.pose = Pose(pose.x, pose.y, pose.theta)
        self.t, self.twist = 0.0, (0.0, 0.0, 0.0)

    def update(self, wheels, dt: float) -> Odom:
        """One integration step; returns the current (noisy) estimate."""
        measured = [w + self.noise.gauss(self.sigma_wheel) for w in wheels]
        vx, vy, omega = physics.forward_kinematics(self.g, measured)
        omega += self.bias_omega
        mid = self.pose.theta + 0.5 * omega * dt
        cos, sin = math.cos(mid), math.sin(mid)
        self.pose.x += (vx * cos - vy * sin) * dt
        self.pose.y += (vx * sin + vy * cos) * dt
        self.pose.theta = wrap_angle(self.pose.theta + omega * dt)
        self.t += dt
        self.twist = (vx, vy, omega)
        return Odom(t=self.t_offset + self.t, x=self.pose.x + self.noise.gauss(self.sigma_xy),
                    y=self.pose.y + self.noise.gauss(self.sigma_xy),
                    theta=wrap_angle(self.pose.theta + self.noise.gauss(self.sigma_theta)),
                    vx=vx, vy=vy, omega=omega)


class Lidar:
    """360 beams evenly over 2*pi; beam 0 points forward, then counter-clockwise.

    Hit distance in the world frame via a ray/rectangle slab test against the walls of
    the world. Other robots are not seen in experiment 1. No hit -> `inf` (the ROS
    bridge turns that into `range_max`, see CONTRACT §6.4).
    """

    def __init__(self, world, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.world, self.noise = world, noise
        self.beams = int(cfg.get("beams", 360))
        self.range_max = float(cfg.get("range_max", 8.0))
        self.range_min = float(cfg.get("range_min", 0.05))
        self.sigma = cfg.get("sigma", 0.0)
        step = 2 * math.pi / self.beams
        self.dirs = [(math.cos(i * step), math.sin(i * step)) for i in range(self.beams)]
        self.increment = step

    def scan(self, pose) -> Scan:
        """One full-circle measurement starting from `pose`."""
        cands = [w for w in self.world.walls if self._near(pose, w)]
        cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
        ranges = []
        for i, (bx, by) in enumerate(self.dirs):
            dx = bx * cos_t - by * sin_t                 # beam direction in the world frame
            dy = bx * sin_t + by * cos_t
            hit = self.range_max
            for w in cands:
                t = _ray_rect(pose.x, pose.y, dx, dy, w)
                if t is not None and t < hit:
                    hit = t
            ranges.append(max(self.range_min, hit + self.noise.gauss(self.sigma))
                          if hit < self.range_max else float("inf"))
        return Scan(t=0.0, angle_min=0.0, angle_increment=self.increment,
                    range_min=self.range_min, range_max=self.range_max, ranges=ranges)

    def _near(self, pose, w) -> bool:
        """Pre-filter: a rectangle farther away than range_max can never be a hit."""
        dx = max(w.x0 - pose.x, 0.0, pose.x - w.x1)
        dy = max(w.y0 - pose.y, 0.0, pose.y - w.y1)
        return dx * dx + dy * dy <= self.range_max * self.range_max


def _ray_rect(px: float, py: float, dx: float, dy: float, w) -> float | None:
    """Distance to entering an axis-aligned rectangle, else None (slab test)."""
    lo, hi = 0.0, float("inf")
    for o, d, a, b in ((px, dx, w.x0, w.x1), (py, dy, w.y0, w.y1)):
        if abs(d) < 1e-12:
            if not a <= o <= b:
                return None
            continue
        t1, t2 = (a - o) / d, (b - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        lo, hi = max(lo, t1), min(hi, t2)
        if lo > hi:
            return None
    return lo if lo > 0 else 0.0


class GpsSensor:
    """Global position (UWB/MoCap): constant bias plus Gaussian measurement noise.

    Named GpsSensor (not Gps) because types.Gps is the message — this sensor produces
    it. Same reasoning as for OdometrySensor and Lidar.

    Three interventions are what make the Kalman filter tasks interesting: `gap`
    suppresses every fix for `gap[1]` seconds from `gap[0]` on (GPS outage in the
    arena), `bias_step` adds an extra offset on top for the same time (multi-path — the
    absolute classic indoors), and `zones` degrade or kill the fix **by place** instead
    of by time — the shadow under a shelf, the corner between two metal walls. Empty by
    default: a task that wants them asks for them (see config/demo_gps_shadow.json).
    """

    def __init__(self, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.noise = noise
        self.sigma_xy = cfg.get("sigma_xy", 0.0)
        self.sigma_theta = cfg.get("sigma_theta", 0.0)
        self.bias = cfg.get("bias_xy") or (0.0, 0.0)
        self.gap = _fenster(cfg.get("gap"), 2)
        self.step = _fenster(cfg.get("bias_step"), 4)
        self.zones = _zonen(cfg.get("zones"))          # place-based degradation, [] = off
        self.t0 = 0.0                                # time reference of the windows (see set_task)

    def fix(self, pose, t: float = 0.0) -> Gps | None:
        """One measurement at simulation time `t`; None means 'no fix' (antenna gone).

        `gap` and `bias_step` are read **relative to task start** — the engine sets `t0` in
        `set_task`. Otherwise the start button decides when the GPS outage hits.
        """
        auf_t = t - self.t0                      # time since task start
        if self.gap and self.gap[0] <= auf_t < self.gap[0] + self.gap[1]:
            return None
        zone = self._zone(pose)
        if zone and zone["block"]:
            return None                                # no satellite visible from here
        sigma = self.sigma_xy * (zone["sigma_scale"] if zone else 1.0)
        dx = dy = 0.0
        if self.step and self.step[0] <= auf_t < self.step[0] + self.step[1]:
            dx, dy = self.step[2], self.step[3]
        if zone:
            dx += zone["bias"][0]
            dy += zone["bias"][1]
        return Gps(t=0.0, x=pose.x + self.bias[0] + dx + self.noise.gauss(sigma),
                   y=pose.y + self.bias[1] + dy + self.noise.gauss(sigma),
                   theta=wrap_angle(pose.theta + self.noise.gauss(self.sigma_theta)))

    def _zone(self, pose):
        """The first zone rectangle that contains `pose`, or None (then nothing changes)."""
        for z in self.zones:
            x0, y0, x1, y1 = z["rect"]
            if x0 <= pose.x <= x1 and y0 <= pose.y <= y1:
                return z
        return None


def _zonen(wert) -> list:
    """Check `gps.zones`: [{name, rect:[x0,y0,x1,y1], sigma_scale, bias_xy, block}, …].

    Rectangles are world metres, the first zone containing the robot wins, and one broken
    entry is dropped instead of killing the run — a typo in a demo file must not stop a lab.
    """
    out = []
    for z in wert or []:
        try:
            x0, y0, x1, y1 = [float(v) for v in z["rect"]]
            bias = [float(v) for v in (z.get("bias_xy") or (0.0, 0.0))][:2]
            out.append({"name": str(z.get("name", f"zone {len(out) + 1}")),
                        "rect": (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)),
                        "sigma_scale": max(1.0, float(z.get("sigma_scale", 1.0))),
                        "bias": (bias + [0.0, 0.0])[:2], "block": bool(z.get("block"))})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def _fenster(wert, n: int) -> list | None:
    """Check a `[start, duration, …]` config; disable nonsense instead of crashing."""
    if not isinstance(wert, (list, tuple)) or len(wert) != n or float(wert[1]) <= 0:
        return None
    return [float(v) for v in wert]


class ImuSensor:
    """6-DOF IMU in the body frame — as broken as a good MEMS module from a kit.

    In order, from the largest error to the smallest:

    1. **Bias** `accel_bias`/`gyro_bias`: fixed offset per axis and robot (a draw from
       the seed). Double-integrated over 10 s, 0.05 m/s² becomes some 2 m — the reason
       you never integrate an IMU openly, but only use it as a motion model with a
       small contribution.
    2. **Bias random walk** `*_bias_walk` (unit/√s): the bias does not stay where it was.
    3. **Tilt**: the suspension wobbles (Ornstein-Uhlenbeck with `tilt_sigma`/`tilt_tau`),
       which tips gravity into the horizontal axes: `dx_ax = -g·pitch`.
    4. **Vibration** `vibration` at `vibration_hz`: chassis/motor, indirect.
    5. **White noise** as a density per √Hz — per sample `density·√(rate/2)`.
    6. **Scale error** (relative, fixed per robot): averaging does not make it vanish.
    7. **Settling**: the first `startup` seconds carry `startup_bias` on top.

    `az` contains — like a real IMU — the specific force: at rest `+9.81`.
    The tilt is reported as well, because an IMU driver estimates it from the gravity
    direction and students would otherwise have to guess how to de-tilt it.
    """

    def __init__(self, noise: Noise, cfg: dict | None = None, robot: str = ""):
        cfg = cfg or {}
        self.noise, self.robot = noise, robot
        self.rate = max(float(cfg.get("rate", 100.0)), 1.0)
        self.g = float(cfg.get("gravity", 9.81))
        self.s_g = float(cfg["gyro_noise"]) * math.sqrt(self.rate / 2.0)
        self.s_a = float(cfg["accel_noise"]) * math.sqrt(self.rate / 2.0)
        self.w_g = float(cfg["gyro_bias_walk"]) * math.sqrt(1.0 / self.rate)
        self.w_a = float(cfg["accel_bias_walk"]) * math.sqrt(1.0 / self.rate)
        self.k_g = 1.0 + noise.uniform(float(cfg["gyro_scale"]))
        self.k_a = 1.0 + noise.uniform(float(cfg["accel_scale"]))
        self.b_g = [noise.gauss(float(cfg["gyro_bias"])) for _ in range(3)]
        self.b_a = [noise.gauss(float(cfg["accel_bias"])) for _ in range(3)]
        self.t_offset = 0.0                      # the engine hooks this clock to simulation time
        self.tilt_sigma = float(cfg["tilt_sigma"])
        self.tilt_tau = max(float(cfg["tilt_tau"]), 1e-3)
        self.vib = float(cfg["vibration"])
        self.vib_hz = float(cfg["vibration_hz"])
        self.startup = max(float(cfg["startup"]), 1e-3)
        self.startup_bias = float(cfg["startup_bias"])
        self.reset()

    def reset(self, pose=None) -> None:
        """Restart: time, buffer and tilt go back; the bias stays (it is the real one).

        `t_offset` stays as it is — the engine sets it again anyway when hooking it to
        simulation time.
        """
        self.t, self.buf, self.phase = 0.0, 0.0, self.noise.uniform(math.tau)
        self.tilt = [0.0, 0.0]                       # (roll, pitch)
        self.tilt_rate = [0.0, 0.0]
        self.vel = (0.0, 0.0)                        # world velocity of the previous sample

    def sample(self, pose, twist, dt: float) -> list:
        """Zero to several IMU samples for one physics step of `dt` seconds.

        The truth is only known at the physics rate (50 Hz here), the IMU ticks faster
        (100 Hz by default): intermediate values hold the acceleration constant — it is
        still noised and integrated at the IMU's own measurement rate.
        """
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        vel = (twist.vx * c - twist.vy * s, twist.vx * s + twist.vy * c)
        if self.t <= 0.0:
            self.vel = vel                           # first sample: no step derivative
        a_welt = ((vel[0] - self.vel[0]) / max(dt, 1e-6), (vel[1] - self.vel[1]) / max(dt, 1e-6))
        self.vel = vel
        a_body = (a_welt[0] * c + a_welt[1] * s, -a_welt[0] * s + a_welt[1] * c)
        self.buf += dt
        raus = []
        periode = 1.0 / self.rate
        while self.buf >= periode:
            self.buf -= periode
            self.t += periode
            raus.append(self._stichprobe(a_body, twist, periode))
        return raus

    def _stichprobe(self, a_body, twist, dt: float) -> Imu:
        """One sample: move the tilt, let the bias walk, put noise on top."""
        for i in range(3):            # bias random walk: the bias does not stay where it was
            self.b_a[i] += self.noise.gauss(self.w_a)
            self.b_g[i] += self.noise.gauss(self.w_g)
        rauschung = math.sqrt(2.0 * dt / self.tilt_tau) * self.tilt_sigma
        for i in range(2):
            alt = self.tilt[i]
            self.tilt[i] += -alt * dt / self.tilt_tau + self.noise.gauss(rauschung)
            self.tilt_rate[i] = (self.tilt[i] - alt) / dt
        schwing = self.vib * math.sin(math.tau * self.vib_hz * self.t + self.phase)
        warm = self.startup_bias * math.exp(-3.0 * self.t / self.startup)
        roll, pitch = self.tilt
        return Imu(
            t=self.t_offset + self.t,
            ax=self.k_a * a_body[0] - self.g * pitch + self.b_a[0] + warm
               + schwing + self.noise.gauss(self.s_a),
            ay=self.k_a * a_body[1] + self.g * roll + self.b_a[1] + 0.7 * warm
               - 0.6 * schwing + self.noise.gauss(self.s_a),
            # Specific force: positive upward, +g at rest — no flaw, that is how the chip ticks
            az=self.g + self.b_a[2] + self.noise.gauss(self.s_a),
            gx=self.k_g * self.tilt_rate[0] + self.b_g[0] + self.noise.gauss(self.s_g),
            gy=self.k_g * self.tilt_rate[1] + self.b_g[1] + self.noise.gauss(self.s_g),
            gz=self.k_g * twist.omega + self.b_g[2] + self.noise.gauss(self.s_g),
            roll=roll + self.noise.gauss(0.2 * self.s_a / max(self.g, 1e-6)),
            pitch=pitch + self.noise.gauss(0.2 * self.s_a / max(self.g, 1e-6)))
