"""Sensoren: Odometrie (Dead Reckoning), 2D-LIDAR, globale Position, trägeheits-IMU.

Alle rechnen gegen die Wahrheit aus physics.py, geben aber nur zurück, was ein echter
Sensor liefern würde — deshalb ist Odometrie Drift, GPS Rauschen und die IMU ein
Bias-Sumpf.

Rauschmodell der Odometrie (bewusst einfach gehalten, damit es erklärbar bleibt):
* `sigma_wheel`    Rauschen auf jeder gemessenen Radgeschwindigkeit — wird **integriert**,
                   daraus entsteht die eigentliche Drift (ZufallsSpaziergang).
* `bias_omega`     systematischer Drehratenfehler, wächst linear mit der Zeit.
* `sigma_xy`/`sigma_theta` Messrauschen auf das **ausgegebene** Ergebnis; es wird nicht
                   zurückgekoppelt, die Schätzung bleibt also frei von Treppchen.

Rauschen der IMU (Versuch 2) wird als **Dichte** pro √Hz angegeben und auf die
Samplingrate umgerechnet — siehe `ImuSensor`, dort steht auch, warum der Bias gewinnt.
"""
import math
import random

from . import physics
from .types import Gps, Imu, Odom, Pose, Scan, wrap_angle


class Noise:
    """Ein einziger Zufallsstrom für die ganze Simulation — gleiches Seed, gleiches Bild."""

    def __init__(self, seed: int | None = None):
        self.rng = random.Random(seed)

    def gauss(self, sigma: float) -> float:
        return self.rng.gauss(0.0, sigma) if sigma else 0.0

    def uniform(self, amp: float) -> float:
        return self.rng.uniform(-amp, amp) if amp else 0.0


class OdometrySensor:
    """Integriert ausschließlich die übergebenen Radgeschwindigkeiten, nie die Wahrheit."""

    def __init__(self, g: physics.Geometry, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.g, self.noise = g, noise
        self.sigma_wheel = cfg.get("sigma_wheel", 0.0)
        self.sigma_xy = cfg.get("sigma_xy", 0.0)
        self.sigma_theta = cfg.get("sigma_theta", 0.0)
        self.bias_omega = cfg.get("bias_omega", 0.0)
        self.pose = Pose()
        self.t = 0.0
        self.t_offset = 0.0                      # Engine haengt diese Uhr an die Simulationszeit
        self.twist = (0.0, 0.0, 0.0)

    def reset(self, pose) -> None:
        """Odometrie-Ursprung ist die Spawn-Pose; von da an wird nur noch addiert."""
        self.pose = Pose(pose.x, pose.y, pose.theta)
        self.t, self.twist = 0.0, (0.0, 0.0, 0.0)

    def update(self, wheels, dt: float) -> Odom:
        """Einintegrationsschritt; liefert die aktuelle (verrauschte) Schätzung."""
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
    """360 Strahlen gleichmäßig über 2*pi; Strahl 0 zeigt in Fahrtrichtung, CCW weiter.

    Trefferentfernung im Weltframe per Strahl/Rechteck-Slab-Test gegen die Wände der
    Welt. Andere Roboter werden in Versuch 1 nicht gesehen. Kein Treffer -> `inf`
    (die ROS-Bridge macht daraus `range_max`, siehe CONTRACT §6.4).
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
        """Eine Vollkreismessung von `pose` aus."""
        cands = [w for w in self.world.walls if self._near(pose, w)]
        cos_t, sin_t = math.cos(pose.theta), math.sin(pose.theta)
        ranges = []
        for i, (bx, by) in enumerate(self.dirs):
            dx = bx * cos_t - by * sin_t                 # Strahlrichtung im Weltframe
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
        """Vorfilter: Rechteck, das weiter als range_max entfernt liegt, kann kein Treffer."""
        dx = max(w.x0 - pose.x, 0.0, pose.x - w.x1)
        dy = max(w.y0 - pose.y, 0.0, pose.y - w.y1)
        return dx * dx + dy * dy <= self.range_max * self.range_max


def _ray_rect(px: float, py: float, dx: float, dy: float, w) -> float | None:
    """Entfernung zum Eintritt in ein achsenparalleles Rechteck, sonst None (Slab-Test)."""
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
    """Globale Position (UWB/MoCap): konstanter Bias plus gaußsches Messrauschen.

    Heisst GpsSensor (nicht Gps), weil types.Gps die Nachricht ist — dieser Sensor
    erzeugt sie. Dieselbe Begründung wie bei OdometrySensor und Lidar.

    Zwei Eingriffe machen die Kalman-Filter-Aufträge erst interessant:
    `gap` unterdrückt für `gap[1]` Sekunden ab `gap[0]` jeden Fix (Funkloch in der
    Halle), `bias_step` legt für dieselbe Zeit einen zusätzlichen Versatz obendrauf
    (Multi-Path — der absolute Klassiker im Innenhof).
    """

    def __init__(self, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.noise = noise
        self.sigma_xy = cfg.get("sigma_xy", 0.0)
        self.sigma_theta = cfg.get("sigma_theta", 0.0)
        self.bias = cfg.get("bias_xy") or (0.0, 0.0)
        self.gap = _fenster(cfg.get("gap"), 2)
        self.step = _fenster(cfg.get("bias_step"), 4)
        self.t0 = 0.0                                # Zeitbezug der Fenster (siehe set_task)

    def fix(self, pose, t: float = 0.0) -> Gps | None:
        """Eine Messung zur Simulationszeit `t`; None heisst 'kein Fix' (Antenne weg).

        `gap` und `bias_step` werden **relativ zum Auftragsbeginn** gelesen — die Engine
        setzt `t0` in `set_task`. Sonst hänge vom Startknopf ab, wann das Funkloch kommt.
        """
        auf_t = t - self.t0                      # Zeit seit Auftragsbeginn
        if self.gap and self.gap[0] <= auf_t < self.gap[0] + self.gap[1]:
            return None
        dx = dy = 0.0
        if self.step and self.step[0] <= auf_t < self.step[0] + self.step[1]:
            dx, dy = self.step[2], self.step[3]
        return Gps(t=0.0, x=pose.x + self.bias[0] + dx + self.noise.gauss(self.sigma_xy),
                   y=pose.y + self.bias[1] + dy + self.noise.gauss(self.sigma_xy),
                   theta=wrap_angle(pose.theta + self.noise.gauss(self.sigma_theta)))


def _fenster(wert, n: int) -> list | None:
    """Konfiguration `[start, dauer, …]` prüfen; Unsinn abschalten statt Absturz."""
    if not isinstance(wert, (list, tuple)) or len(wert) != n or float(wert[1]) <= 0:
        return None
    return [float(v) for v in wert]


class ImuSensor:
    """6-DOF-IMU im Körperframe — so kaputt wie ein gutes MEMS-Modul aus dem Bausatz.

    Der Reihe nach, vom größten zum kleinsten Fehler:

    1. **Bias** `accel_bias`/`gyro_bias`: fester Versatz je Achse und Roboter (Ziehung
       aus dem Seed). Doppelintegriert über 10 s werden aus 0,05 m/s² gut 2 m — der
       Grund, warum man eine IMU nie offen integriert, sondern nur als Bewegungmodell
       mit kleinem Beitrag benutzt.
    2. **Bias-Random-Walk** `*_bias_walk` (Einheit/√s): der Bias bleibt nicht, wo er war.
    3. **Neigung**: Federung wackelt (Ornstein-Uhlenbeck mit `tilt_sigma`/`tilt_tau`),
       damit kippt die Schwerkraft in die Horizontalachsen: `dx_ax = -g·pitch`.
    4. **Vibration** `vibration` bei `vibration_hz`: Fahrwerk/Motor, mittelbar.
    5. **Weißes Rauschen** als Dichte pro √Hz — pro Stichprobe `dichte·√(rate/2)`.
    6. **Skalenfehler** (relativ, je Roboter fest): verschwindet nicht durch Mitteln.
    7. **Einschwingen**: die ersten `startup` Sekunden zusätzlich `startup_bias` drauf.

    `az` enthält — wie bei einer echten IMU — die Specific Force: im Stand `+9,81`.
    Die Neigung wird mit ausgegeben, weil ein IMU-Treiber sie aus der Schwerkraft-
    richtung schätzt und Studierende das Entneigen sonst raten müssen.
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
        self.t_offset = 0.0                      # Engine haengt diese Uhr an die Simulationszeit
        self.tilt_sigma = float(cfg["tilt_sigma"])
        self.tilt_tau = max(float(cfg["tilt_tau"]), 1e-3)
        self.vib = float(cfg["vibration"])
        self.vib_hz = float(cfg["vibration_hz"])
        self.startup = max(float(cfg["startup"]), 1e-3)
        self.startup_bias = float(cfg["startup_bias"])
        self.reset()

    def reset(self, pose=None) -> None:
        """Neu hochfahren: Zeit, Puffer und Neigung zurück; der Bias bleibt (er ist ja echt).

        `t_offset` bleibt, wie es ist — die Engine setzt es beim Umhaengen an die
        Simulationszeit ohnehin neu.
        """
        self.t, self.buf, self.phase = 0.0, 0.0, self.noise.uniform(math.tau)
        self.tilt = [0.0, 0.0]                       # (roll, pitch)
        self.tilt_rate = [0.0, 0.0]
        self.vel = (0.0, 0.0)                        # Weltgeschwindigkeit der letzten Stichprobe

    def sample(self, pose, twist, dt: float) -> list:
        """Null bis mehrere IMU-Stichproben für einen Physikschritt von `dt` Sekunden.

        Die Wahrheit ist nur mit Physikrate bekannt (hier 50 Hz), die IMU tickt schneller
        (Standard 100 Hz): Zwischenwerte halten die Beschleunigung konstant — gemessen
        wird sie trotzdem mit ihrer eigenen Rate verrauscht und integriert.
        """
        c, s = math.cos(pose.theta), math.sin(pose.theta)
        vel = (twist.vx * c - twist.vy * s, twist.vx * s + twist.vy * c)
        if self.t <= 0.0:
            self.vel = vel                           # erster Stich: keine Sprung-Ableitung
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
        """Eine Stichprobe: Neigung bewegen, Bias wandern lassen, Rauschen draufpacken."""
        for i in range(3):            # Bias-Random-Walk: der Bias bleibt nicht, wo er war
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
            # Specific Force: nach oben positiv, im Stand +g — kein Mesfehler, so tickt der Chip
            az=self.g + self.b_a[2] + self.noise.gauss(self.s_a),
            gx=self.k_g * self.tilt_rate[0] + self.b_g[0] + self.noise.gauss(self.s_g),
            gy=self.k_g * self.tilt_rate[1] + self.b_g[1] + self.noise.gauss(self.s_g),
            gz=self.k_g * twist.omega + self.b_g[2] + self.noise.gauss(self.s_g),
            roll=roll + self.noise.gauss(0.2 * self.s_a / max(self.g, 1e-6)),
            pitch=pitch + self.noise.gauss(0.2 * self.s_a / max(self.g, 1e-6)))
