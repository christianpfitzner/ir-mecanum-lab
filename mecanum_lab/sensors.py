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

The second half of a measurement is how good it was: `Gps.quality`/`sats` from `GpsSensor.sky()`,
`Scan.missing`, `Imu.temp`, and the messages that never arrived (`GpsSensor.drops`). Every knob
that makes one of those imperfect is off in `DEFAULT_CONFIG`, and off means not one random number
is drawn (`Noise.chance`, `Noise.late`) — the streams of the graded tasks are unchanged.
"""
import math
import random
from dataclasses import replace

from . import physics
from .types import Gps, Imu, Odom, Pose, Scan, wrap_angle

LATENCY_JITTER = 0.5      # ±50 % on `gps.latency`: a radio is never exactly one latency
TEMP_FULL_SPEED = 0.8     # m/s of driving at which the IMU's electronics are warm


class Noise:
    """One single random stream for the whole simulation — same seed, same picture."""

    def __init__(self, seed: int | None = None):
        self.rng = random.Random(seed)

    def gauss(self, sigma: float) -> float:
        return self.rng.gauss(0.0, sigma) if sigma else 0.0

    def uniform(self, amp: float) -> float:
        return self.rng.uniform(-amp, amp) if amp else 0.0

    def chance(self, p: float) -> bool:
        """Yes with probability `p` — and with `p` 0 without touching the generator at all.

        That last half is what lets a new knob default to "off" without moving a single number in
        the streams of the graded tasks: a knob that is off must not so much as ask a question.
        """
        return 0.0 < p <= 1.0 and self.rng.random() < p

    def late(self, frac: float) -> float:
        """How late an arrival is: 0 … `frac`, never negative, 0.0 without a random draw.

        One-sided on purpose. A report that comes early does not exist; the uneven stamps of a real
        bus are all lateness, and an offset that can only be positive cannot put a measurement
        behind the one before it either (a filter that predicts with a negative dt is broken).
        """
        return self.rng.uniform(0.0, frac) if frac > 0.0 else 0.0


class OdometrySensor:
    """Integrates only the wheel speeds it is given, never the truth.

    Two kinds of error act on it: noise (`sigma_*`, `bias_omega` below) and **model error** —
    the geometry the integrator believes is not the chassis (see `odom_geometry`). Noise averages
    out over a drive, model error does not: it is why a real robot does not come back to the spot
    it started from, which is exactly what T2 makes the students see.
    """

    def __init__(self, g: physics.Geometry, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.g, self.noise = g, noise
        self.sigma_wheel = cfg.get("sigma_wheel", 0.0)
        self.sigma_xy = cfg.get("sigma_xy", 0.0)
        self.sigma_theta = cfg.get("sigma_theta", 0.0)
        self.bias_omega = cfg.get("bias_omega", 0.0)
        model = cfg.get("geometry") or {}        # model error, see odom_geometry()
        self.scale_xy = float(model.get("scale_xy") or 1.0)
        self.bias_xy = model.get("bias_xy") or (0.0, 0.0)
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
        vx, vy = vx * self.scale_xy, vy * self.scale_xy    # path too long/short, yaw untouched
        omega += self.bias_omega
        mid = self.pose.theta + 0.5 * omega * dt
        cos, sin = math.cos(mid), math.sin(mid)
        self.pose.x += (vx * cos - vy * sin) * dt
        self.pose.y += (vx * sin + vy * cos) * dt
        self.pose.theta = wrap_angle(self.pose.theta + omega * dt)
        self.t += dt
        self.twist = (vx, vy, omega)
        return Odom(t=self.t_offset + self.t,
                    x=self.pose.x + self.bias_xy[0] + self.noise.gauss(self.sigma_xy),
                    y=self.pose.y + self.bias_xy[1] + self.noise.gauss(self.sigma_xy),
                    theta=wrap_angle(self.pose.theta + self.noise.gauss(self.sigma_theta)),
                    vx=vx, vy=vy, omega=omega)


def odom_geometry(true_g: physics.Geometry, scales: dict | None) -> physics.Geometry:
    """The geometry the odometry integrator believes — deliberately not the true one.

    Every term of `odom.geometry` is a mistake a real robot makes (all default to 1.0):

    * `wheel_radius_scale` — the wheels are worn, or reprinted, or the diameter was used where
      the radius belongs. Every wheel speed converts into the wrong distance, so **the whole
      path scales with it**: 1.05 reports 5 % too many metres, the turns included.
    * `lever_scale` — the wheel positions you assume are all off by the same relative factor,
      which scales the lever arm a = lx + ly. The turn rate is r/(4a) · (-w1+w2-w3+w4), so it
      comes out too large by 1/k: a straight command drives an arc and **a full circle ends
      rotated** by 2*pi*(1/k - 1).
    * `wheel_base_scale` — only the vehicle length mis-measured (lx): the four wheel speeds mix
      into (vx, vy, omega) in the wrong ratio while |v| stays about right.

    `lever_scale` scales lx and ly together, `wheel_base_scale` only lx — that is what keeps both
    knobs expressible on a `Geometry`, whose arm is lx + ly. The other two terms of
    `odom.geometry` are not geometry and are applied by the sensor: `scale_xy` multiplies the body
    velocity (path too long/short with the yaw rate right, e.g. wrong radius but an honest gyro),
    `bias_xy` offsets the reported pose (a wrong odom origin — a shifted world, not drift).
    """
    s = scales or {}                             # a key that is there but null: not set
    k_r = float(s.get("wheel_radius_scale") or 1.0)
    k_b = float(s.get("wheel_base_scale") or 1.0)
    k_a = float(s.get("lever_scale") or 1.0)
    if (k_r, k_b, k_a) == (1.0, 1.0, 1.0):
        return true_g
    return replace(true_g, r=true_g.r * k_r, lx=true_g.lx * k_b * k_a, ly=true_g.ly * k_a)


class Lidar:
    """360 beams evenly over 2*pi; beam 0 points forward, then counter-clockwise.

    Hit distance in the world frame via a ray/rectangle slab test against the walls of
    the world. Other robots are not seen in experiment 1. No hit -> `inf` (the ROS
    bridge turns that into `range_max`, see CONTRACT §6.4).

    `max_walls` is how many segments one scan may use at all. A real LIDAR reports a fixed
    number of echoes, and what it did not report is not there for the robot — so the cap is
    taken in world order and the walls behind it are simply not seen. The default (400) is
    above the wall count of every world in `worlds/`, so by default nothing is hidden.

    A real LIDAR also does not see every wall it hits, because what arrives at the receiver is an
    echo and not a hit: see `reflectivity_min` and `_reflects()` below. Both knobs are off by
    default, and the beams of a graded scan are the beams of experiment 1.
    """

    def __init__(self, world, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.world, self.noise = world, noise
        self.beams = int(cfg.get("beams", 360))
        self.range_max = float(cfg.get("range_max", 8.0))
        self.range_min = float(cfg.get("range_min", 0.05))
        self.max_walls = int(cfg.get("max_walls") or 400)
        self.sigma = cfg.get("sigma", 0.0)
        # Weakest echo a surface may send back, as |cos| of the incidence angle (0.0 = off).
        self.reflectivity_min = min(max(float(cfg.get("reflectivity_min") or 0.0), 0.0), 1.0)
        step = 2 * math.pi / self.beams
        self.dirs = [(math.cos(i * step), math.sin(i * step)) for i in range(self.beams)]
        self.increment = step

    def scan(self, pose) -> Scan:
        """One full-circle measurement starting from `pose`."""
        cands = [w for w in self.world.walls if self._near(pose, w)][:self.max_walls]
        cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
        ranges = []
        for i, (bx, by) in enumerate(self.dirs):
            dx = bx * cos_t - by * sin_t                 # beam direction in the world frame
            dy = bx * sin_t + by * cos_t
            hit, face = self.range_max, 0
            for w in cands:
                found = _ray_rect(pose.x, pose.y, dx, dy, w)
                if found is not None and found[0] < hit:
                    hit, face = found
            if hit < self.range_max and self._reflects(dx, dy, face):
                ranges.append(max(self.range_min, hit + self.noise.gauss(self.sigma)))
            else:
                ranges.append(float("inf"))         # no echo — never a wall at `range_max`
        return Scan(t=0.0, angle_min=0.0, angle_increment=self.increment,
                    range_min=self.range_min, range_max=self.range_max, ranges=ranges,
                    missing=sum(1 for r in ranges if math.isinf(r)))

    def _reflects(self, dx: float, dy: float, face: int) -> bool:
        """Does the surface the beam entered send anything back along this beam?

        The returned energy of a flat surface goes roughly with the cosine of the angle between
        beam and surface normal, so a wall the robot looks at *along* it reflects far less than one
        it looks at straight on. `reflectivity_min` is the sensitivity cut-off in that cosine:
        0.25 drops every beam flatter than ~15° to the surface, which is exactly why a real lidar
        does not see a painted post it drives past or a white line on the floor.

        Only the nearest surface is asked; a real sensor reports one echo per beam, not a list.
        With the default 0.0 the answer is always yes and no wall map of a graded task changes.
        """
        if self.reflectivity_min <= 0.0:
            return True
        return abs(dx if face == 0 else dy) >= self.reflectivity_min

    def _near(self, pose, w) -> bool:
        """Pre-filter: a rectangle farther away than range_max can never be a hit."""
        dx = max(w.x0 - pose.x, 0.0, pose.x - w.x1)
        dy = max(w.y0 - pose.y, 0.0, pose.y - w.y1)
        return dx * dx + dy * dy <= self.range_max * self.range_max


def _ray_rect(px: float, py: float, dx: float, dy: float, w) -> tuple | None:
    """(distance to entering an axis-aligned rectangle, which axis was crossed) or None.

    The axis belongs in the answer because whether a LIDAR sees a wall depends on the wall's
    orientation, not only on it being there (`Lidar._reflects`): face 0 means the ray came in
    through an x face, so the surface normal at the hit is (±1, 0) and |cos| is |dx|.
    """
    lo, hi, face = 0.0, float("inf"), 0
    for axis, (o, d, a, b) in enumerate(((px, dx, w.x0, w.x1), (py, dy, w.y0, w.y1))):
        if abs(d) < 1e-12:
            if not a <= o <= b:
                return None
            continue
        t1, t2 = (a - o) / d, (b - o) / d
        if t1 > t2:
            t1, t2 = t2, t1
        if t1 > lo:
            lo, face = t1, axis                        # this pair of faces is what was entered
        hi = min(hi, t2)
        if lo > hi:
            return None
    return (lo if lo > 0 else 0.0), face


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

    A fourth one is a plain transport delay: `delay_ticks` (or `kf.gps_delay` seconds, which the
    engine converts into ticks) keeps each fix in a ring buffer and hands it out N emissions
    later. The message keeps the stamp it was **generated** with, so a filter can see the delay
    (`now - fix.t`) and predict over it instead of feeding itself a stale position — which is the
    whole point of CONTRACT-KF §5's `delay_ticks >= 1`.

    Two more belong to the transport rather than to the sky: `dropout` (probability per message
    that the packet is lost, counted per robot in `drops`) and `latency` (seconds on the wire,
    jittered). Both are deterministic: the drop decision is taken **before** anything is measured,
    so the drop pattern of a run depends on the seed and on the number of emissions and not on
    where the robot happened to be.

    Every fix carries what it is worth: `quality` 2 good, 1 degraded, 0 no fix, and `sats`, the
    anchors in view. `sky()` computes both from the position alone, so the window can show them
    while the receiver is silent — which is the only way a student can tell "no fix here" apart
    from "the radio lost it".
    """

    def __init__(self, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.noise = noise
        self.sigma_xy = cfg.get("sigma_xy", 0.0)
        self.sigma_theta = cfg.get("sigma_theta", 0.0)
        self.bias = cfg.get("bias_xy") or (0.0, 0.0)
        self.gap = _window(cfg.get("gap"), 2)
        self.step = _window(cfg.get("bias_step"), 4)
        self.zones = _zones(cfg.get("zones"))          # place-based degradation, [] = off
        self.delay_ticks = max(int(cfg.get("delay_ticks", 0) or 0), 0)
        self.dropout = min(max(float(cfg.get("dropout") or 0.0), 0.0), 1.0)
        self.latency = max(float(cfg.get("latency") or 0.0), 0.0)
        self.sats = max(int(cfg.get("sats") if cfg.get("sats") is not None else 8), 0)
        self.sats_min = max(int(cfg.get("sats_min") or 4), 1)   # under this: degraded, not good
        self.drops: dict[str, int] = {}                # lost messages per robot (one receiver model,
        #                                                several robots share this one)
        self._ring = []                                # fixes waiting for their turn (FIFO)
        self._wire = []                                # (time it is due, fix) while `latency` > 0
        self.t0 = 0.0                                  # time reference of the windows (see set_task)

    def reset(self) -> None:
        """Restart: what was on its way is gone, and the drive counters start at zero."""
        self._ring.clear()
        self._wire.clear()
        self.drops.clear()

    def fix(self, pose, t: float = 0.0, robot: str = "") -> Gps | None:
        """One measurement at simulation time `t`; None means 'no fix' (antenna gone).

        `gap` and `bias_step` are read **relative to task start** — the engine sets `t0` in
        `set_task`. Otherwise the start button decides when the GPS outage hits.

        With `delay_ticks` the fix is still generated now (its own noise, its own place in the
        gap window) and only delivered N emissions later, so the delay is a transport delay and
        not a wrong position. While the buffer fills, no fix comes out yet. `latency` is the same
        thing in seconds with a jitter, for a radio that is not synchronous with the sim step.

        `robot` is only the name the drop counter is kept under: a lost message is counted per
        robot, so the readout of the robot that lost it is the one that says so.
        """
        if self.noise.chance(self.dropout):
            self.drops[robot] = self.drops.get(robot, 0) + 1      # counted before anything is read
            return None                                # no radio: this emission never happened
        fresh = self._measure(pose, t)
        if self.latency:
            return self._deliver(fresh, t)
        if self.delay_ticks == 0 or fresh is None:
            return fresh
        self._ring.append(fresh)
        if len(self._ring) <= self.delay_ticks:
            return None                                # buffer still filling: no fix received
        return self._ring.pop(0)

    def _deliver(self, fresh, t: float) -> Gps | None:
        """The latency queue: a fix is due `latency` seconds after it was measured, ±50 % jittered.

        One fix leaves per emission, so a queue that fell behind drains over the following
        emissions instead of handing out two positions in the same instant.
        """
        if fresh is not None:
            due = t + self.latency * (1.0 + self.noise.uniform(LATENCY_JITTER))
            self._wire.append((due, fresh))
        for i, (due, queued) in enumerate(self._wire):
            if due <= t:
                return self._wire.pop(i)[1]
        return None                                    # everything measured is still on the wire

    def _measure(self, pose, t: float) -> Gps | None:
        """The fix generated at simulation time `t` — stamped with that time, not with delivery."""
        since_task = t - self.t0                      # time since task start
        if self.gap and self.gap[0] <= since_task < self.gap[0] + self.gap[1]:
            return None
        zone = self._zone(pose)
        quality, sats = self.sky(pose, zone)
        if quality == 0:
            return None                                # no anchors left: there is no position to send
        sigma = self.sigma_xy * (zone["sigma_scale"] if zone else 1.0)
        dx = dy = 0.0
        if self.step and self.step[0] <= since_task < self.step[0] + self.step[1]:
            dx, dy = self.step[2], self.step[3]
            quality = min(quality, 1)                  # multi-path: a fix, but not a usable one
        if zone:
            dx += zone["bias"][0]
            dy += zone["bias"][1]
        return Gps(t=t, x=pose.x + self.bias[0] + dx + self.noise.gauss(sigma),
                   y=pose.y + self.bias[1] + dy + self.noise.gauss(sigma),
                   theta=wrap_angle(pose.theta + self.noise.gauss(self.sigma_theta)),
                   quality=quality, sats=sats)

    def sky(self, pose, zone=None) -> tuple:
        """(quality, anchors in view) at `pose` — the same rule the fix reports with.

        No random number is drawn here, so the readout can ask every frame and a test can state the
        answer without a seed. A zone costs anchors: `sats` in the zone entry says how many the
        receiver still sees there, and without it every unit of `sigma_scale` inflation counts as
        one anchor less — whatever sits between the antenna and the sky does both at once.

        Quality 0 (blackout zone, or no anchors left) means **no fix**, and `fix()` then answers
        `None`: a receiver without a solution sends no position. The readout still shows 0, from
        here, next to the count of messages that did not arrive — "nothing because there is
        nothing" and "nothing because the radio lost it" are different faults.
        """
        zone = self._zone(pose) if zone is None else zone
        if zone is None:
            return 2, self.sats
        if zone["block"]:
            return 0, 0
        sats = (self.sats - max(round(zone["sigma_scale"] - 1.0), 0)
                if zone["sats"] is None else zone["sats"])
        if sats <= 0:
            return 0, 0
        weak = sats < self.sats_min or zone["sigma_scale"] > 1.0 or any(zone["bias"])
        return (1 if weak else 2), sats

    def _zone(self, pose):
        """The first zone rectangle that contains `pose`, or None (then nothing changes)."""
        for z in self.zones:
            x0, y0, x1, y1 = z["rect"]
            if x0 <= pose.x <= x1 and y0 <= pose.y <= y1:
                return z
        return None


def _zones(value) -> list:
    """Check `gps.zones`: [{name, rect:[x0,y0,x1,y1], sigma_scale, bias_xy, sats, block}, …].

    Rectangles are world metres, the first zone containing the robot wins, and one broken
    entry is dropped instead of killing the run — a typo in a demo file must not stop a lab.
    """
    out = []
    for z in value or []:
        try:
            x0, y0, x1, y1 = [float(v) for v in z["rect"]]
            bias = [float(v) for v in (z.get("bias_xy") or (0.0, 0.0))][:2]
            raw_sats = z.get("sats")
            out.append({"name": str(z.get("name", f"zone {len(out) + 1}")),
                        "rect": (min(x0, x1), min(y0, y1), max(x0, x1), max(y0, y1)),
                        "sigma_scale": max(1.0, float(z.get("sigma_scale", 1.0))),
                        "sats": None if raw_sats is None else max(int(raw_sats), 0),
                        "bias": (bias + [0.0, 0.0])[:2], "block": bool(z.get("block"))})
        except (KeyError, TypeError, ValueError):
            continue
    return out


def _window(value, n: int) -> list | None:
    """Check a `[start, duration, …]` config; disable nonsense instead of crashing."""
    if not isinstance(value, (list, tuple)) or len(value) != n or float(value[1]) <= 0:
        return None
    return [float(v) for v in value]


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
    8. **Temperature**: the chip warms up with what the robot does, and the bias walks with it
       (`temp_start`/`temp_motor`/`temp_tau`, `temp_walk`/`temp_walk_gyro`). Off by default — the
       graded filters of experiment 2 are tuned against a bias that stays where it started.

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
        # Thermal model: the module is a first-order lag towards the temperature its load asks
        # for, and the bias is a function of that temperature (see `_warm_up`).
        self.temp_start = float(cfg.get("temp_start", 24.0))     # °C cold, and what it cools back to
        self.temp_motor = float(cfg.get("temp_motor", 0.0))      # °C the drive adds at full speed
        self.temp_tau = max(float(cfg.get("temp_tau", 30.0)), 1e-3)
        self.temp_walk = float(cfg.get("temp_walk", 0.0))        # m/s² of bias per °C
        self.temp_walk_g = float(cfg.get("temp_walk_gyro", 0.0))  # rad/s of bias per °C
        self.reset()

    def reset(self, pose=None) -> None:
        """Restart: time, buffer and tilt go back; the bias stays (it is the real one).

        `t_offset` stays as it is — the engine sets it again anyway when hooking it to
        simulation time. The chip starts cold at `temp_start`: a robot that has been standing in
        the hall for ten minutes starts at its cold bias, and that is what the next drive warms.
        """
        self.t, self.carry, self.phase = 0.0, 0.0, self.noise.uniform(math.tau)
        self.tilt = [0.0, 0.0]                       # (roll, pitch)
        self.tilt_rate = [0.0, 0.0]
        self.temp = self.temp_start
        self.vel = (0.0, 0.0)                        # world velocity of the previous sample

    def _warm_up(self, dt: float, speed: float) -> None:
        """Move the chip temperature one sample towards what the load asks for.

        First-order lag with `temp_tau` towards `temp_start` + `temp_motor` at full drive: a module
        that has been standing reports its cold bias, the same module two minutes into a drive
        reports another one. Datasheets draw exactly this curve — bias against temperature — and
        until now the IMU here had a bias that could only walk, never follow the robot's load.
        """
        want = self.temp_start + self.temp_motor * min(1.0, speed / TEMP_FULL_SPEED)
        self.temp += (want - self.temp) * dt / self.temp_tau

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
        a_world = ((vel[0] - self.vel[0]) / max(dt, 1e-6), (vel[1] - self.vel[1]) / max(dt, 1e-6))
        self.vel = vel
        a_body = (a_world[0] * c + a_world[1] * s, -a_world[0] * s + a_world[1] * c)
        self.carry += dt
        out = []
        period = 1.0 / self.rate
        while self.carry >= period:
            self.carry -= period
            self.t += period
            out.append(self._one_sample(a_body, twist, period))
        return out

    def _one_sample(self, a_body, twist, dt: float) -> Imu:
        """One sample: move the tilt, let the bias walk with its temperature, put noise on top."""
        self._warm_up(dt, math.hypot(twist.vx, twist.vy))
        # The bias follows the temperature, so it walks while the robot drives and walks back while
        # it stands: not extra noise (which averaging would remove) but an offset that moves.
        warm_a = self.temp_walk * (self.temp - self.temp_start)
        warm_g = self.temp_walk_g * (self.temp - self.temp_start)
        for i in range(3):            # bias random walk: the bias does not stay where it was
            self.b_a[i] += self.noise.gauss(self.w_a)
            self.b_g[i] += self.noise.gauss(self.w_g)
        tilt_noise = math.sqrt(2.0 * dt / self.tilt_tau) * self.tilt_sigma
        for i in range(2):
            alt = self.tilt[i]
            self.tilt[i] += -alt * dt / self.tilt_tau + self.noise.gauss(tilt_noise)
            self.tilt_rate[i] = (self.tilt[i] - alt) / dt
        vibration = self.vib * math.sin(math.tau * self.vib_hz * self.t + self.phase)
        settling = self.startup_bias * math.exp(-3.0 * self.t / self.startup)
        roll, pitch = self.tilt
        return Imu(
            t=self.t_offset + self.t,
            ax=self.k_a * a_body[0] - self.g * pitch + self.b_a[0] + settling + warm_a
               + vibration + self.noise.gauss(self.s_a),
            ay=self.k_a * a_body[1] + self.g * roll + self.b_a[1] + 0.7 * settling + 0.6 * warm_a
               - 0.6 * vibration + self.noise.gauss(self.s_a),
            # Specific force: positive upward, +g at rest — no flaw, that is how the chip ticks
            az=self.g + self.b_a[2] + warm_a + self.noise.gauss(self.s_a),
            gx=self.k_g * self.tilt_rate[0] + self.b_g[0] + warm_g + self.noise.gauss(self.s_g),
            gy=self.k_g * self.tilt_rate[1] + self.b_g[1] + warm_g + self.noise.gauss(self.s_g),
            gz=self.k_g * twist.omega + self.b_g[2] + warm_g + self.noise.gauss(self.s_g),
            roll=roll + self.noise.gauss(0.2 * self.s_a / max(self.g, 1e-6)),
            pitch=pitch + self.noise.gauss(0.2 * self.s_a / max(self.g, 1e-6)),
            temp=self.temp)
