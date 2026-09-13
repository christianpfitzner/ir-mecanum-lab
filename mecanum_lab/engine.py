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

from . import physics, pois, sensors, steering, wifi
from .types import (Pose, Robot, RobotSpec, SensorInfo, Twist, cfg_get, load_config, merge,
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
        self._build_sensors()
        self._index = 0
        self._sub = 1.0 / float(cfg_get(self.cfg, "rate", 50))
        self._acc = 0.0
        self.dropped = 0.0          # s of sim time this stepper threw away (see step())
        self._warned_drop = False

    # --------------------------------------------------------------------- Sensor builders

    def _build_sensors(self) -> None:
        """(Re)build the three field sensors and the radio from the config as it stands.

        Called twice and by design: once here at startup, once by `set_sensor_profile()` when a task
        brings another test profile. One builder for both, because a fourth sensor has to be rebuilt
        by both — two copies is how a profile switch ends up keeping an old instrument.

        Each maker answers `None` when its option is off, and `None` means the loop does not ask that
        sensor at all: no detector, no radio, no message, and — the part the tests exist for — not one
        random number drawn. That is why the graded streams of both experiments are byte for byte what
        they were before `gps.zones`, `pois` and `wifi.enabled` were invented (`test_sensor_reality.py`,
        `test_wifi_w6.py`).
        """
        self._lidar = sensors.Lidar(self.world, self._noise, cfg_get(self.cfg, "lidar"))
        self._gps = self._make_gps()
        self._poi = self._make_pois()
        self._wifi = self._make_wifi()

    def _make_gps(self):
        """GPS sensor, with `kf.gps_delay` (seconds, the student-facing name) turned into ticks.

        The `kf` block is a recommendation to the students and nothing in the simulator reads it
        — except this one value: a filter has to be written for a delayed fix, so the delay has to
        be simulable. `gps.delay_ticks` (emissions) wins when both are given.
        """
        gpscfg = cfg_get(self.cfg, "gps") or {}
        ticks = int(gpscfg.get("delay_ticks", 0) or 0)
        seconds = float(cfg_get(self.cfg, "kf.gps_delay", 0.0) or 0.0)
        as_ticks = int(round(seconds * float(cfg_get(self.cfg, "gps.rate", 5.0))))
        if as_ticks > ticks:
            log.info("simulating the recommended GPS delay: kf.gps_delay = %.2f s -> "
                     "gps.delay_ticks = %d", seconds, as_ticks)
            gpscfg["delay_ticks"] = as_ticks     # so /sim/config says what the sensor will do
        return sensors.GpsSensor(self._noise, gpscfg)

    def _make_pois(self):
        """The radiation detector of this world — or None, which is the default (see `_build_sensors`).

        `pois` (the sources of the world) is validated here rather than in worlds.py because the
        check needs both sides: the point from the config and the walls of the arena it has to be
        inside. A bad entry raises, it is not dropped — a scenario with a source in a wall cannot be
        solved, so saying so at once beats a run in which nothing ever explains itself (pois.py).
        """
        sources = pois.load_sources(cfg_get(self.cfg, "pois"), self.world,
                                   float(cfg_get(self.cfg, "poi.d0", 1.0)))
        if not sources:
            return None
        log.info("%d points of interest in '%s': %s", len(sources), self.world.name,
                 ", ".join(f"{s.name} at ({s.x:g}, {s.y:g}) r={s.range_m:g} m" for s in sources))
        return pois.PoiSensor(sources, self._noise, cfg_get(self.cfg, "poi"))

    def _make_wifi(self):
        """The hall's radio — or None, which is the default (the None rule is in `_build_sensors`).

        Enabled without an AP for this world is a run that would silently deliver everything: the
        per-hall positions live in `wifi.ap_by_world`, a hall neither that nor `wifi.ap` names gets
        one warning and a plain link.
        """
        wcfg = cfg_get(self.cfg, "wifi") or {}
        if not bool(wcfg.get("enabled", False)):
            return None
        ap = wifi.access_point(wcfg, self.world)
        if ap is None:
            log.warning("wifi.enabled but world '%s' has no access point — the link delivers "
                        "everything. Name one: --set wifi.ap=[x,y]", self.world.name)
            return None
        log.info("radio link: AP at (%.2f, %.2f) in '%s', %.1f dBm at %.1f m, n=%.1f, "
                 "%.1f dB per wall crossing", ap[0], ap[1], self.world.name,
                 float(wcfg.get("tx_dbm", -34.0)), float(wcfg.get("d0", 1.0)),
                 float(wcfg.get("n", 2.4)), float(wcfg.get("wall_db", 12.0)))
        return wifi.Wifi(self.world, ap, self._noise, wcfg)

    def _make_odometer(self, r):
        """The robot's odometry: wheel speeds integrated over the *believed* geometry.

        Not the chassis geometry — `odom.geometry` is what makes a wrong wheel radius or a wrong
        lever arm expressible at all (sensors.odom_geometry). Empty config -> the true geometry.

        The steered car gets its own integrator (steering.Odometry): four wheel speeds do not carry
        a steering angle, and a bicycle model has to be integrated from `(v, delta)`.
        """
        if isinstance(r.chassis.geometry, steering.SteeringGeometry):
            odometer = steering.build_odometer(r.chassis.geometry, self._noise,
                                               cfg_get(self.cfg, "odom"))
        else:
            odometer = sensors.OdometrySensor(
                sensors.odom_geometry(r.chassis.geometry, cfg_get(self.cfg, "odom.geometry")),
                self._noise, cfg_get(self.cfg, "odom"))
        odometer.reset(r.chassis.pose)                 # odom origin = spawn pose, not (0,0)
        return odometer

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
        spawn_pose = pose or self.world.spawn_pose(idx)
        # The second drive train brings its own geometry and its own chassis; one `if` here and the
        # rest of the engine does not care which of the two it is looking at.
        chassis = (steering.make_chassis(cfg_get(self.cfg, "steering"), variant, spawn_pose,
                                         seed=idx + (self._index or 1))
                   if steering.is_steering(variant) else
                   physics.Chassis(physics.make_geometry(cfg_get(self.cfg, "robot"), variant),
                                   spawn_pose, seed=idx + (self._index or 1)))
        r = Robot(spec=spec, chassis=chassis)
        r.odometer = self._make_odometer(r)
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
            if self._wifi is not None:
                self._wifi.forget(name)   # a frame to a robot that is gone has nowhere to arrive
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
        r.chassis.reset_motion()                       # a steered car also has to straighten up
        r.chassis.contacts = 0
        r.odometer.reset(r.chassis.pose)
        r.inertial.reset()
        self._gps.drops.pop(name, None)     # the losses of the previous drive are not this one's
        if self._wifi is not None:
            self._wifi.forget(name)         # and neither is what was still on the wire to it
        self._clock_to_sim(r)
        r.wheel_cmd = r.vel_cmd = None
        r.mode, r.t_cmd, r.t_vel = "pass-through", -1.0, -1.0
        r.imu, r.kf, r.kf_err = None, None, None
        r.contacts, r.distance, r.mission_state = 0, 0.0, "idle"
        return True

    def reset(self) -> None:
        """Return all robots to their start pose, counters at zero (names stay).

        The per-robot half is `reset_robot()` — the same eight fields for all robots, and the KF
        grader depends on that list being the same for one robot and for all of them. Simulation time
        then goes back to 0, which is the only thing this does beyond the single-robot case.
        """
        for name in list(self.robots):
            self.reset_robot(name)
        self.t = 0.0
        self._gps.reset()                 # a fix that was on its way is not part of the new run
        self._gps.t0 = 0.0
        if self._wifi is not None:
            self._wifi.reset()            # nor is a frame that had not reached the robot yet
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
        self._build_sensors()
        if self._wifi is None:                       # a profile that removes the radio also ends any
            for r in self.robots.values():           # autonomy it started: nothing could lift it
                if r.mode == "autonomy":
                    r.mode = "pass-through"
        self._gps.t0 = getattr(self, "t_task", self.t)
        for r in self.robots.values():
            r.odometer = self._make_odometer(r)
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
        """A body-speed command from outside (topic, teleop keys, grader) — so it goes by radio."""
        self._deliver(name, "twist", twist)

    def set_wheel_speeds(self, name: str, wheels) -> None:
        """Wheel speeds [FL, FR, RL, RR] in rad/s — the exercise core, and a radio frame too."""
        r = self.robots.get(name)
        if not r or len(wheels or []) != 4:
            return                                  # a malformed frame never reaches the radio either
        self._deliver(name, "wheels", [float(w) for w in wheels])

    def _deliver(self, name: str, kind: str, payload) -> None:
        """Hand one external command to the radio, or straight to the robot when there is none.

        This is the seam the whole feature turns on: with no `Wifi` object the call is the old
        assignment, one step and done. With one, the command is the AP's problem now — dropped, or
        put on the wire and applied when it arrives (`_apply_cmd`), which is why `t_vel`/`t_cmd` are
        the *delivery* time and the command watchdog keeps meaning what it always meant.
        """
        if self.robots.get(name) is None:
            return
        if self._wifi is None:
            self._apply_cmd(name, kind, payload)
        else:
            self._wifi.admit(name, self.robots[name].pose, kind, payload, self.t)

    def _apply_cmd(self, name: str, kind: str, payload) -> None:
        """What a command that reached the robot does to it — the half `set_cmd_vel` used to be."""
        r = self.robots.get(name)
        if r is None:
            return
        if kind == "twist":
            r.vel_cmd, r.t_vel = payload, self.t
            return
        if r.mode != "autonomy":
            r.mode = "wheels"                    # from now on only this counts
        r.wheel_cmd, r.t_cmd = payload, self.t

    def _wire(self, name: str) -> list:
        """The commands whose simulated flight time is over — in the order they were sent."""
        return self._wifi.due(name, self.t) if self._wifi is not None else []

    def note_keys(self, name: str) -> None:
        """Record that the window's keyboard was the last publisher for this robot.

        The teleop keys publish on `/cmd_vel` like any node does — same topic, same radio, and only
        while a key is held — so nothing about the drive depends on this call. It is the answer to the
        one question the readout line has when a node and a keyboard share a topic: who was last.
        """
        r = self.robots.get(name)
        if r is not None:
            r.cmd_keys = self.t

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

    @property
    def sub_step(self) -> float:
        """The fixed physics step (1/rate) — what a `--fixed-step` run steps by."""
        return self._sub

    def step(self, dt: float) -> None:
        """Split elapsed time into fixed physics steps (determinism).

        The accumulator is capped at 0.5 s: a caller that stalls for ten seconds must not freeze
        the machine with catch-up steps afterwards. That throws sim time away, so it is counted
        (`dropped`) and said out loud once — silently losing whole seconds is how a grading run
        stops matching the seed it was started with.
        """
        want = self._acc + max(dt, 0.0)
        self._acc = min(want, 0.5)
        if want > 0.5:
            self.dropped += want - 0.5
            self._warn_drop()
        while self._acc >= self._sub:
            self._substep(self._sub)
            self._acc -= self._sub

    def _warn_drop(self) -> None:
        """Say it once, and with the number: a sim behind the wall clock is not grading."""
        if not self._warned_drop and self.dropped >= 0.5:
            self._warned_drop = True
            log.warning("sim time fell behind the wall clock by %.1f s — use --speed or "
                        "--fixed-step", self.dropped)

    def _substep(self, dt: float) -> None:
        for r in self.robots.values():
            for kind, payload in self._wire(r.spec.name):     # late frames arrive before this step
                self._apply_cmd(r.spec.name, kind, payload)
            self._drive(r, dt)
            x0, y0 = r.chassis.pose.x, r.chassis.pose.y
            r.chassis.step(dt, self.world.walls)
            p, v = r.chassis.pose, r.chassis.twist
            r.pose.x, r.pose.y, r.pose.theta = p.x, p.y, p.theta
            r.twist.vx, r.twist.vy, r.twist.omega = v.vx, v.vy, v.omega
            r.wheels = list(r.chassis.wheels)
            r.contacts = r.chassis.contacts
            r.distance += math.hypot(p.x - x0, p.y - y0)
            odo = r.odometer.update(r.chassis.odo_feed(), dt)      # integrate every step
            for msg in r.inertial.sample(r.pose, r.twist, dt):  # IMU runs faster than the physics
                self._push("imu", r.spec.name, msg)
                r.imu = msg
            if self._due(r, "odom", cfg_get(self.cfg, "odom.rate", 50.0), dt):
                r.odom = odo
                # `odom.jitter` moves the stamp and only the stamp: a report that reaches the bus
                # late is still the same measurement, and an encoder stream with perfectly even
                # stamps is the fiction. Late only, never early, so the stamps stay in order and a
                # filter can keep predicting with `dt = stamp - previous stamp`.
                lag = self._noise.late(cfg_get(self.cfg, "odom.jitter", 0.0))
                self._push("odom", r.spec.name, odo,
                           stamp=self.t - lag / float(cfg_get(self.cfg, "odom.rate", 50.0)))
            if self._due(r, "scan", cfg_get(self.cfg, "lidar.rate", 20.0), dt):
                r.scan = self._lidar.scan(r.pose)
                self._push("scan", r.spec.name, r.scan)
            if self._due(r, "gps", cfg_get(self.cfg, "gps.rate", 5.0), dt):
                fix = self._gps.fix(r.pose, self.t, r.spec.name)
                if fix is not None:                            # GPS outage: no message, no last fix
                    r.gps = fix
                    self._push("gps", r.spec.name, fix, stamp=fix.t)  # keep the emission stamp
            if cfg_get(self.cfg, "debug_truth") and self._due(
                    r, "truth", cfg_get(self.cfg, "truth.rate", 20.0), dt):
                self._push("truth", r.spec.name, Pose(r.pose.x, r.pose.y, r.pose.theta))
            if self._poi is not None and self._due(
                    r, "poi", cfg_get(self.cfg, "poi.rate", 5.0), dt):
                # A counter that is switched on reports even where it hears nothing: the 0.0 outside
                # a source's range is a measurement too, and the only way a student sees the
                # difference between "out of range" and "the topic is not running".
                reading = self._poi.read(r.pose)
                if reading is not None:
                    r.poi = reading
                    self._push("poi", r.spec.name, reading)
            if self._wifi is not None:
                self._step_link(r, dt)
            if self._due(r, "sensorinfo", cfg_get(self.cfg, "gps.rate", 5.0), dt):
                self._push("sensorinfo", r.spec.name, self.sensor_info(r.spec.name))
        self.t += dt

    def _step_link(self, r: Robot, dt: float) -> None:
        """One physics step of the radio for one robot: level, autonomy decision, `/link`.

        Stepped every step and published at `wifi.rate`, because the failsafe is a timer and a timer
        that is sampled at 5 Hz is wrong by 200 ms. The decision it takes is one bit on the robot —
        `mode` — and the frames still on the wire when the link died go with it: a station that lost
        its association keeps nothing, and a command that arrives after the failsafe has fired would
        contradict the one sentence this feature exists to state.
        """
        st = self._wifi.step(r.spec.name, r.pose, dt)
        if st.down and r.mode != "autonomy":
            st.mode_before = r.mode
            r.mode = "autonomy"
            st.pending.clear()
            log.warning("link to '%s' down at %.1f dBm (q %.2f for %.1f s) -> %s",
                        r.spec.name, st.rssi_dbm, st.quality, st.low_for, self._wifi.autonomy)
        elif not st.down and r.mode == "autonomy":
            r.mode = st.mode_before
            log.info("link to '%s' is up again at %.1f dBm (q %.2f), back to %s",
                     r.spec.name, st.rssi_dbm, st.quality, st.mode_before)
        if self._due(r, "link", cfg_get(self.cfg, "wifi.rate", 5.0), dt):
            # A robot out of range still reports: `up: false` is a measurement too, and the only way
            # a student sees the difference between "far away" and "the topic is not running".
            self._push("link", r.spec.name, self._wifi.message(r.spec.name))

    def _drive(self, r: Robot, dt: float) -> None:
        """Apply commands: own wheel values beat cmd_vel, both with a watchdog.

        `mode == "autonomy"` is the one case where the watchdog is not the boss — see `_hold()`, which
        is the two onboard rules of `wifi.autonomy`.
        """
        if r.mode == "autonomy" and self._wifi is not None:
            self._hold(r)
            return
        wheels_ok = r.wheel_cmd is not None and 0 <= self.t - r.t_cmd <= self.cfg["cmd_timeout"]
        vel_ok = r.vel_cmd is not None and 0 <= self.t - r.t_vel <= self.cfg["cmd_timeout"]
        if r.mode == "wheels" and wheels_ok:
            r.chassis.set_wheels(r.wheel_cmd)
        elif r.mode == "pass-through" and vel_ok:
            v = r.vel_cmd
            r.chassis.set_twist(v.vx, v.vy, v.omega)       # the chassis maps it, not the engine
        else:
            r.chassis.set_wheels([0.0] * 4)          # radio silence -> let the motors coast down

    def _hold(self, r: Robot) -> None:
        """What a robot does with itself while the link is down — `wifi.autonomy`, both spellings.

        `stop` is what the stale-command branch of `_drive()` already does: no frame arrives, the
        watchdog stops the motors, the robot holds the place it was left at. `dead_reckoning` replays
        the last frame that really arrived — the robot finishes the manoeuvre it was told to do, wall
        in the way or not, which is how a real robot walks itself home and why the course has to talk
        about which of the two a practice robot should ship.
        """
        st = self._wifi.state(r.spec.name)
        replay = self._wifi.autonomy == "dead_reckoning"
        if replay and st.mode_before == "wheels" and r.wheel_cmd is not None:
            r.chassis.set_wheels(r.wheel_cmd)
        elif replay and r.vel_cmd is not None:
            v = r.vel_cmd
            r.chassis.set_twist(v.vx, v.vy, v.omega)
        else:
            r.chassis.set_wheels([0.0] * 4)          # "stop": the watchdog's answer, held

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

    def _push(self, kind: str, robot: str | None, payload, stamp: float | None = None) -> None:
        """Queue a measurement, stamped with simulation time.

        `stamp` is for a sensor whose message and measurement instant differ: the GPS delay buffer
        and its latency queue keep the stamp the fix was **generated** with, and `odom.jitter`
        stamps a report with when it reached the bus rather than when it was taken. Without that
        argument the receiver cannot see the delay and the grader cannot tell an outage from a late
        fix.
        """
        if len(self.outbox) > MAX_OUTBOX:            # endless loop without a consumer
            del self.outbox[:MAX_OUTBOX // 2]
        if hasattr(payload, "t"):                    # stamp simulation time into the measurement
            payload.t = self.t if stamp is None else stamp
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
                 # The two front wheel angles of a steered car, in degrees — they are not wheel
                 # speeds and would not fit into the list above. Empty on a mecanum robot.
                 "steer_deg": [round(math.degrees(a), 1) for a in
                               getattr(r.chassis, "steer", [])],
                 "kf": None if not r.kf else [round(r.kf.x, 3), round(r.kf.y, 3),
                                              round(r.kf.sx, 3), round(r.kf.sy, 3)],
                 "kf_err": None if r.kf_err is None else round(r.kf_err, 3),
                 "contacts": r.contacts, "distance": round(r.distance, 2),
                 "mission": r.mission_state, "task": self.task}
                for r in self.robots.values()]

    def gps_health(self, name: str) -> tuple:
        """(quality, anchors in view, messages lost) for one robot's GPS.

        Asked of the receiver and not of the last message on purpose: while the robot sits in a
        blackout there *is* no fresh message, and "the sky here is empty" versus "the radio lost
        five packets" is the difference the readout and the log have to show without a second
        terminal. `GpsSensor.sky()` draws no random number, so the window can ask every frame.
        """
        r = self.robots.get(name)
        if r is None:
            return (0, 0, 0)
        q, sats = self._gps.sky(r.pose)
        return (q, sats, self._gps.drops.get(name, 0))

    def sensor_info(self, name: str) -> SensorInfo:
        """The instruments' answer for one robot — `types.SensorInfo`, published as `/sensor/info`.

        Asked of the instruments and not of their last messages, for the same reason as
        `gps_health()`: in a blackout there is no fresh fix to read, and the reason is the point.
        The temperature comes from the IMU object because it moves with every sample and reaches a
        message only by accident; `scan_gaps` is the one number that does describe the last scan.
        Published at `gps.rate`, the slowest of the three instruments — nothing here changes faster.
        """
        r = self.robots[name]
        quality, sats, lost = self.gps_health(name)
        return SensorInfo(t=self.t, quality=quality, sats=sats, lost=lost,
                          latency_ms=1000.0 * self._gps.latency, temp=r.inertial.temp,
                          scan_gaps=r.scan.missing if r.scan is not None else 0)

    def link_health(self, name: str) -> tuple | None:
        """(quality, dBm, metres, walls, up, dropped, sent, latency_ms, ap); None: no radio in this lab.

        The network layer's `gps_health()`, for the same reason: the readout has to be able to say
        what the antenna sees while no command is being delivered, and the AP's position belongs in
        the same answer the window draws the line from. None rather than a neutral tuple, so the
        layer stays empty in every run that did not ask for a radio.
        """
        return None if self._wifi is None else self._wifi.health(name)

    def poi_sources(self) -> list:
        """The sources of the running world (pois.Source) — for the window's truth view, not for a node.

        There is no topic for this on purpose: finding the place from the intensity series is the
        exercise, so `overlays.poi_sources()` is the only reader it has, and `debug_truth` decides
        whether even the rings are drawn. A student node that wants the answer can only read the
        overlay or ask a tutor, which is the same border as `--truth` for the pose.
        """
        return list(self._poi.sources) if self._poi is not None else []

    @property
    def wifi(self):
        """The radio of this run, or None when `wifi.enabled` is false (which is the default).

        A property rather than the attribute, because the network panel and the coverage layer of the
        window need to *look* at the link budget — an overlay that read `engine._wifi` would be
        reaching past the name the engine keeps private. Everything that *changes* the radio (`step`,
        `admit`, `due`) stays where it belongs: in the run loop of §6.14.
        """
        return self._wifi

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
        """Sensor profile of the running simulation — the grader checks its test profile here.

        `steering` travels along because the drive geometry is a fact of the running car, not a
        recommendation: a student node that plans corners from `R = L / tan(delta_max)` has to ask
        the simulation which car it is driving, not its own local config file.

        `poi` (the detector: rate, `d0`, counts, whether the distance is published) is in here; the
        `pois` (where the sources are) are deliberately not. One is how the sensor works, the other
        is the answer to the question the sensor asks.
        """
        profile = {k: cfg_get(self.cfg, k) for k in
                  ("gps", "odom", "imu", "lidar", "truth", "rate", "debug_truth", "steering",
                   "poi", "wifi")}
        # Which access point the running model actually uses — the per-world default and the override
        # are two different config layers and a student should never have to guess which one won.
        profile["wifi"] = dict(profile.get("wifi") or {},
                               effective_ap=None if self._wifi is None else list(self._wifi.ap))
        profile["seed"] = self.seed
        return json.dumps(profile)
