"""SimEngine: wires world + robots + sensors into a single tick.

The engine core knows neither ROS nor pygame: it knows physics, sensors and an
`outbox` of measurements that someone else (ROS bridge, grader, test) picks up.
`Robot` additionally gets the attributes `odometer` and `inertial` (the engine's own
sensors) — dataclasses without __slots__ allow that, and it keeps the view of the
robot small for the GUI and the grader.
"""
import json
import logging
import math

from . import physics, sensors
from .types import (Gps, Pose, Robot, RobotSpec, Twist, cfg_get, load_config, merge,
                    sanitize_name, PALETTE, MARKERS, VARIANTS)

log = logging.getLogger("mecanum.engine")
MAX_OUTBOX = 20000


class SpawnError(Exception):
    """Robot name invalid, already taken, or limit reached."""


class SimEngine:
    """Owns the world and all robots; one simulated second hand per tick."""

    def __init__(self, world, cfg: dict | None = None, seed: int | None = None):
        self.cfg = cfg or load_config()
        self.world = world
        self.seed = seed
        self.t = 0.0
        self.t_task = 0.0                            # start of the current task
        self.task = ""
        self.forced: dict = {}                    # --set values: beat any test profile
        self.robots: dict[str, Robot] = {}
        self.outbox: list = []                       # (kind, robot|None, payload)
        self._noise = sensors.Noise(seed)
        self._lidar = sensors.Lidar(world, self._noise, cfg_get(self.cfg, "lidar"))
        self._gps = sensors.GpsSensor(self._noise, cfg_get(self.cfg, "gps"))
        self._index = 0
        self._sub = 1.0 / float(cfg_get(self.cfg, "rate", 50))
        self._acc = 0.0

    # ------------------------------------------------------------------ Robot management

    def spawn(self, name: str, variant: str = "", pose: Pose | None = None) -> Robot:
        """Add a new robot. The name must be unique and valid."""
        name = sanitize_name(name)
        if name in self.robots:
            raise SpawnError(f"Name '{name}' is already taken.")
        if len(self.robots) >= int(cfg_get(self.cfg, "spawn_limit", 12)):
            raise SpawnError(f"Robot limit ({cfg_get(self.cfg, 'spawn_limit')}) reached.")
        idx = self._index
        self._index += 1
        variant = variant or VARIANTS[idx % len(VARIANTS)]
        color, rgb = PALETTE[idx % len(PALETTE)]
        spec = RobotSpec(name=name, index=idx, color=color, rgb=rgb,
                         marker=MARKERS[idx % len(MARKERS)], variant=variant)
        geometry = physics.make_geometry(cfg_get(self.cfg, "robot"), variant)
        r = Robot(spec=spec, chassis=physics.Chassis(geometry, pose or self.world.spawn_pose(idx),
                                                     seed=idx + (self._index or 1)))
        r.odometer = sensors.OdometrySensor(geometry, self._noise, cfg_get(self.cfg, "odom"))
        r.odometer.reset(r.chassis.pose)          # odom origin = spawn pose, not (0,0)
        r.inertial = sensors.ImuSensor(self._noise, cfg_get(self.cfg, "imu"), robot=name)
        self._clock_to_sim(r)
        # The truth pose is only copied into the robot by a physics step; until the first one the
        # dataclass default (0, 0) would be reported — enough to streak a trail across the hall.
        r.pose = Pose(r.chassis.pose.x, r.chassis.pose.y, r.chassis.pose.theta)
        self.robots[name] = r
        self.publish_world()
        p = r.chassis.pose
        log.info("spawned robot '%s' (%s, %s) at (%.2f, %.2f, %.0f deg)",
                 name, variant, color, p.x, p.y, math.degrees(p.theta))
        log.debug("IMU '%s': bias a=(%+.3f,%+.3f,%+.3f) m/s^2, g=(%+.4f,%+.4f,%+.4f) rad/s, "
                  "scale a=%.4f g=%.4f", name, *r.inertial.b_a, *r.inertial.b_g,
                  r.inertial.k_a, r.inertial.k_g)
        return r

    def despawn(self, name: str) -> bool:
        found = self.robots.pop(name, None) is not None
        if found:
            self.publish_world()
        return found

    def _clock_to_sim(self, r) -> None:
        """Attach sensor clocks to simulation time — stamps are world time, not 'since spawn'.

        Odometry and the IMU count from their own creation. Without a reference their stamp
        would read 0 again after a respawn or profile switch, while the GPS keeps reporting
        simulation time — a filter comparing both stamps would predict from there into the
        future (and be completely wrong in the grading).
        """
        r.odometer.t_offset = self.t
        r.inertial.t_offset = self.t

    def reset_robot(self, name: str) -> bool:
        """Return one robot to its start pose — without touching the simulation time.

        The KF grader drives a command sequence blind, never seeing the robot: it has to start
        at the spawn pose, otherwise the second task drives into a wall because the first one
        ended somewhere. Simulation time keeps running — the log, the sensor stamps and the
        sensors' task-relative time accounting hang off it.
        """
        r = self.robots.get(name)
        if r is None:
            return False
        home = self.world.spawn_pose(r.spec.index)
        r.chassis.pose = Pose(home.x, home.y, home.theta)
        r.chassis.wheels = [0.0] * 4
        r.chassis.contacts = 0
        r.odometer.reset(r.chassis.pose)
        r.inertial.reset()
        self._clock_to_sim(r)
        r.wheel_cmd = r.vel_cmd = None
        r.mode, r.t_cmd, r.t_vel = "pass-through", -1.0, -1.0
        r.imu, r.kf, r.kf_err = None, None, None
        r.contacts, r.distance, r.mission_state = 0, 0.0, "idle"
        return True

    def reset(self) -> None:
        """Return all robots to their start pose, counters at zero (names stay)."""
        for r in self.robots.values():
            home = self.world.spawn_pose(r.spec.index)
            r.chassis.pose = Pose(home.x, home.y, home.theta)
            r.chassis.wheels = [0.0] * 4
            r.chassis.contacts = 0
            r.odometer.reset(r.chassis.pose)
            r.inertial.reset()
            r.wheel_cmd = r.vel_cmd = None
            r.mode, r.t_cmd, r.t_vel = "pass-through", -1.0, -1.0
            r.imu, r.kf, r.kf_err = None, None, None
            r.contacts, r.distance, r.mission_state = 0, 0.0, "idle"
        self.t = 0.0
        self._gps.t0 = 0.0
        for r in self.robots.values():
            self._clock_to_sim(r)
        self.publish_world()

    def set_task(self, name: str) -> None:
        """Set the task; this also starts the clock for the sensor interventions.

        The same task twice is not a new task: otherwise `t_task` and the sensor profile would
        reset while the task is only being handed to late joiners over and over (node.py
        republishes /sim/task once per second).

        `gps.gap` and `gps.bias_step` are seconds **after task start** in the task file —
        otherwise the start button decides when the GPS outage hits.
        """
        name = name or ""
        if name == self.task:
            return
        self.task = name
        self.t_task = self.t
        self._gps.t0 = self.t

    def set_sensor_profile(self, profile: dict) -> None:
        """Switch the sensors while running — the grader uses this per task.

        Every `sim` profile in config/tasks.json describes another arena (different GPS, IMU
        rate, GPS outage). Instead of restarting for each task the sensors are rebuilt: other
        noise inputs, nothing else. Odometry starts at the current pose, otherwise a profile
        switch would jump the pose.

        Reporting the same profile twice is ignored — otherwise every second odometry would be
        reset to the truth and the IMU bias re-rolled.
        """
        dump = json.dumps(profile, sort_keys=True, default=str)
        if not profile or dump == getattr(self, "_profile_seen", None):
            return
        self._profile_seen = dump
        merge(self.cfg, dict(profile))
        if self.forced:
            merge(self.cfg, dict(self.forced))     # what was set by hand stays put
        self._lidar = sensors.Lidar(self.world, self._noise, cfg_get(self.cfg, "lidar"))
        self._gps = sensors.GpsSensor(self._noise, cfg_get(self.cfg, "gps"))
        self._gps.t0 = getattr(self, "t_task", self.t)
        for r in self.robots.values():
            r.odometer = sensors.OdometrySensor(r.chassis.geometry, self._noise,
                                               cfg_get(self.cfg, "odom"))
            r.odometer.reset(r.chassis.pose)
            r.inertial = sensors.ImuSensor(self._noise, cfg_get(self.cfg, "imu"),
                                          robot=r.spec.name)
            self._clock_to_sim(r)
        for kind in ("gps", "odom", "imu", "truth"):
            for r in self.robots.values():
                r.ticks.pop(kind, None)
        log.info("sensor profile switched: gps σ=%.2f m %.1f Hz, imu %.0f Hz%s", float(
            cfg_get(self.cfg, "gps.sigma_xy", 0.0)), float(cfg_get(self.cfg, "gps.rate", 0.0)),
            float(cfg_get(self.cfg, "imu.rate", 0.0)),
            ", GPS outage " + str(cfg_get(self.cfg, "gps.gap")) if cfg_get(self.cfg, "gps.gap")
            else "")

    # ------------------------------------------------------------------- Commands (ROS)

    def set_cmd_vel(self, name: str, twist: Twist) -> None:
        r = self.robots.get(name)
        if r:
            r.vel_cmd, r.t_vel = twist, self.t

    def set_wheel_speeds(self, name: str, wheels) -> None:
        """Wheel speeds [FL, FR, RL, RR] in rad/s — the exercise core."""
        r = self.robots.get(name)
        if not r or len(wheels or []) != 4:
            return
        r.mode = "wheels"                            # from now on only this counts
        r.wheel_cmd, r.t_cmd = [float(w) for w in wheels], self.t

    def set_mission_state(self, name: str, state: str) -> None:
        r = self.robots.get(name)
        if r:
            r.mission_state = str(state)

    def set_kf(self, name: str, kf) -> None:
        """Accept a robot's own state estimate (experiment 2) and measure it against the truth.

        The error here is only for the HUD and `./lab robots` — grading happens in the grader
        (grade.py), because that one knows the time windows and the measurement series.
        """
        r = self.robots.get(name)
        if r and kf is not None:
            kf.t = self.t                                # stamp the arrival in simulation time
            r.kf = kf
            r.kf_err = math.hypot(kf.x - r.pose.x, kf.y - r.pose.y)

    # ------------------------------------------------------------------------- Time stepping

    def step(self, dt: float) -> None:
        """Split elapsed time into fixed physics steps (determinism)."""
        self._acc = min(self._acc + max(dt, 0.0), 0.5)
        while self._acc >= self._sub:
            self._substep(self._sub)
            self._acc -= self._sub

    def _substep(self, dt: float) -> None:
        for r in self.robots.values():
            self._drive(r, dt)
            x0, y0 = r.chassis.pose.x, r.chassis.pose.y
            r.chassis.step(dt, self.world.walls)
            p, v = r.chassis.pose, r.chassis.twist
            r.pose.x, r.pose.y, r.pose.theta = p.x, p.y, p.theta
            r.twist.vx, r.twist.vy, r.twist.omega = v.vx, v.vy, v.omega
            r.wheels = list(r.chassis.wheels)
            r.contacts = r.chassis.contacts
            r.distance += math.hypot(p.x - x0, p.y - y0)
            odo = r.odometer.update(r.chassis.wheels, dt)      # integrate every step
            for msg in r.inertial.sample(r.pose, r.twist, dt):  # IMU runs faster than the physics
                self._push("imu", r.spec.name, msg)
                r.imu = msg
            if self._due(r, "odom", cfg_get(self.cfg, "odom.rate", 50.0), dt):
                r.odom = odo
                self._push("odom", r.spec.name, odo)
            if self._due(r, "scan", cfg_get(self.cfg, "lidar.rate", 20.0), dt):
                r.scan = self._lidar.scan(r.pose)
                self._push("scan", r.spec.name, r.scan)
            if self._due(r, "gps", cfg_get(self.cfg, "gps.rate", 5.0), dt):
                fix = self._gps.fix(r.pose, self.t)
                if fix is not None:                            # GPS outage: no message, no last fix
                    r.gps = fix
                    self._push("gps", r.spec.name, fix)
            if cfg_get(self.cfg, "debug_truth") and self._due(
                    r, "truth", cfg_get(self.cfg, "truth.rate", 20.0), dt):
                self._push("truth", r.spec.name, Pose(r.pose.x, r.pose.y, r.pose.theta))
        self.t += dt

    def _drive(self, r: Robot, dt: float) -> None:
        """Apply commands: own wheel values beat cmd_vel, both with a watchdog."""
        wheels_ok = r.wheel_cmd is not None and 0 <= self.t - r.t_cmd <= self.cfg["cmd_timeout"]
        vel_ok = r.vel_cmd is not None and 0 <= self.t - r.t_vel <= self.cfg["cmd_timeout"]
        if r.mode == "wheels" and wheels_ok:
            r.chassis.set_wheels(r.wheel_cmd)
        elif r.mode == "pass-through" and vel_ok:
            v = r.vel_cmd
            r.chassis.set_wheels(physics.inverse_kinematics(
                r.chassis.geometry, v.vx, v.vy, v.omega))
        else:
            r.chassis.set_wheels([0.0] * 4)          # radio silence -> let the motors coast down

    def _due(self, r: Robot, kind: str, rate: float, dt: float) -> bool:
        """Enforce the sensor rate per robot and measured quantity."""
        period = 1.0 / max(rate, 0.001)
        acc = r.ticks.get(kind, 0.0) + dt
        if acc >= period:
            r.ticks[kind] = acc - period
            return True
        r.ticks[kind] = acc
        return False

    # ------------------------------------------------------------------------- Messages

    def _push(self, kind: str, robot: str | None, payload) -> None:
        if len(self.outbox) > MAX_OUTBOX:            # endless loop without a consumer
            del self.outbox[:MAX_OUTBOX // 2]
        if hasattr(payload, "t"):                    # stamp simulation time into the measurement
            payload.t = self.t
        self.outbox.append((kind, robot, payload))

    def drain(self) -> list:
        """Take all pending measurements and empty the outbox (called by the bridge)."""
        out, self.outbox = self.outbox, []
        return out

    def robots_info(self) -> list:
        return [{"name": r.spec.name, "index": r.spec.index, "color": r.spec.color,
                 "marker": r.spec.marker, "variant": r.spec.variant, "mode": r.mode,
                 "pose": [round(r.pose.x, 3), round(r.pose.y, 3), round(r.pose.theta, 3)],
                 "wheels": [round(w, 2) for w in r.wheels],
                 "kf": None if not r.kf else [round(r.kf.x, 3), round(r.kf.y, 3),
                                              round(r.kf.sx, 3), round(r.kf.sy, 3)],
                 "kf_err": None if r.kf_err is None else round(r.kf_err, 3),
                 "contacts": r.contacts, "distance": round(r.distance, 2),
                 "mission": r.mission_state, "task": self.task}
                for r in self.robots.values()]

    def publish_world(self) -> None:
        self._push("robots", None, json.dumps(self.robots_info()))

    def world_json(self) -> str:
        """Short world description (size, goal, spawns) — for student nodes."""
        g = self.world.goal
        return json.dumps({"name": self.world.name, "cell": self.world.cell,
                           "size": list(self.world.size),
                           "goal": None if not g else [g.x, g.y, g.theta],
                           "spawns": [[p.x, p.y, p.theta] for p in self.world.spawns],
                           "walls": len(self.world.walls)})

    def config_json(self) -> str:
        """Sensor profile of the running simulation — the grader checks its test profile here."""
        profile = {k: cfg_get(self.cfg, k) for k in
                  ("gps", "odom", "imu", "lidar", "truth", "rate", "debug_truth")}
        profile["seed"] = self.seed
        return json.dumps(profile)
