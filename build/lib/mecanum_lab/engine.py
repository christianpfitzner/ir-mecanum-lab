"""SimEngine: Welt + Roboter + Sensoren zu einem Takt verdrahtet.

Der Engine-Kern kennt weder ROS noch pygame: er kennt Physik, Sensoren und eine
`outbox` mit Messungen, die ein Andere (ROS-Bridge, Auswerter, Test) abholt.
`Robot` bekommt zusaetzlich die Attribute `odometer` und `inertial` (die Engine-eigenen
Sensoren) — Dataclasses ohne __slots__ erlauben das, und es haelt die Sicht auf den
Roboter fuer GUI und Bewerter klein.
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
    """Robotername ungueltig, vergeben, oder Limit erreicht."""


class SimEngine:
    """Besitzt die Welt und alle Roboter; ein simulerter Sekundenzeiger pro Takt."""

    def __init__(self, world, cfg: dict | None = None, seed: int | None = None):
        self.cfg = cfg or load_config()
        self.world = world
        self.seed = seed
        self.t = 0.0
        self.t_task = 0.0                            # Beginn des aktuellen Auftrags
        self.task = ""
        self.erzwungen: dict = {}                    # --set-Angaben: gewinnen gegen jedes Prüfprofil
        self.robots: dict[str, Robot] = {}
        self.outbox: list = []                       # (kind, robot|None, payload)
        self._noise = sensors.Noise(seed)
        self._lidar = sensors.Lidar(world, self._noise, cfg_get(self.cfg, "lidar"))
        self._gps = sensors.GpsSensor(self._noise, cfg_get(self.cfg, "gps"))
        self._index = 0
        self._sub = 1.0 / float(cfg_get(self.cfg, "rate", 50))
        self._acc = 0.0

    # ------------------------------------------------------------------ Roboterverwaltung

    def spawn(self, name: str, variant: str = "", pose: Pose | None = None) -> Robot:
        """Neuen Roboter aufnehmen. Name muss eindeutig und gueltig sein."""
        name = sanitize_name(name)
        if name in self.robots:
            raise SpawnError(f"Name '{name}' ist schon vergeben.")
        if len(self.robots) >= int(cfg_get(self.cfg, "spawn_limit", 12)):
            raise SpawnError(f"Roboterlimit ({cfg_get(self.cfg, 'spawn_limit')}) erreicht.")
        idx = self._index
        self._index += 1
        variant = variant or VARIANTS[idx % len(VARIANTS)]
        color, rgb = PALETTE[idx % len(PALETTE)]
        spec = RobotSpec(name=name, index=idx, color=color, rgb=rgb,
                         marker=MARKERS[idx % len(MARKERS)], variant=variant)
        geom = physics.make_geometry(cfg_get(self.cfg, "robot"), variant)
        r = Robot(spec=spec, chassis=physics.Chassis(geom, pose or self.world.spawn_pose(idx),
                                                     seed=idx + (self._index or 1)))
        r.odometer = sensors.OdometrySensor(geom, self._noise, cfg_get(self.cfg, "odom"))
        r.inertial = sensors.ImuSensor(self._noise, cfg_get(self.cfg, "imu"), robot=name)
        self._zeitbezug(r)
        self.robots[name] = r
        self.publish_world()
        p = r.chassis.pose
        log.info("Roboter '%s' gespawnt (%s, %s) an (%.2f, %.2f, %.0f Grad)",
                 name, variant, color, p.x, p.y, math.degrees(p.theta))
        log.debug("IMU '%s': Bias a=(%+.3f,%+.3f,%+.3f) m/s^2, g=(%+.4f,%+.4f,%+.4f) rad/s, "
                  "Skala a=%.4f g=%.4f", name, *r.inertial.b_a, *r.inertial.b_g,
                  r.inertial.k_a, r.inertial.k_g)
        return r

    def despawn(self, name: str) -> bool:
        found = self.robots.pop(name, None) is not None
        if found:
            self.publish_world()
        return found

    def _zeitbezug(self, r) -> None:
        """Sensoruhren an die Simulationszeit haengen — ihre Stempel sind Weltzeit, nicht 'seit Spawn'.

        Odometrie und IMU zaehlen ab ihrer Entstehung. Ohne Bezug waere ihr Stempel nach
        einem Respawn oder Profilwechsel wieder 0, waehrend das GPS weiter
        Simulationszeit meldet — ein Filter, der beide Stempel vergleicht, wuerde von dort
        in die Zukunft praedizieren (und in der Bewertung komplett daneben liegen).
        """
        r.odometer.t_offset = self.t
        r.inertial.t_offset = self.t

    def reset_robot(self, name: str) -> bool:
        """Einen Roboter zurueck an seine Startpose — ohne die Simulationszeit anzufassen.

        Der KF-Bewerter faehrt eine Kommandofolge blind, ohne den Roboter je zu sehen: die muss
        an der Spawn-Pose beginnen, sonst faehrt sie in der zweiten Aufgabe gegen eine Wand,
        weil die erste irgendwo geendet hat. Die Simulationszeit laeuft weiter — Protokoll,
        Sensorstempel und die auftragsrelative Zeitrechnung der Sensoren haengen an ihr.
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
        self._zeitbezug(r)
        r.wheel_cmd = r.vel_cmd = None
        r.mode, r.t_cmd, r.t_vel = "pass-through", -1.0, -1.0
        r.imu, r.kf, r.kf_err = None, None, None
        r.contacts, r.distance, r.mission_state = 0, 0.0, "idle"
        return True

    def reset(self) -> None:
        """Alle Roboter zurueck an ihre Startpose, Zaehler null (Namen bleiben)."""
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
            self._zeitbezug(r)
        self.publish_world()

    def set_task(self, name: str) -> None:
        """Auftrag setzen; damit beginnt auch die Zeitrechnung der Sensor-Eingriffe.

        Zweimal derselbe Auftrag ist kein neuer Auftrag: sonst würden `t_task` und das
        Sensorprofil zurückgesetzt, während der Auftrag nur laufend an Späteinsteiger
        verteilt wird (node.py veröffentlicht /sim/task einmal pro Sekunde erneut).

        `gps.gap` und `gps.bias_step` stehen in Sekunden **nach Auftragsbeginn** in der
        Aufgabendatei — sonst hängt es am Startknopf, wann das Funkloch kommt.
        """
        name = name or ""
        if name == self.task:
            return
        self.task = name
        self.t_task = self.t
        self._gps.t0 = self.t

    def set_sensor_profil(self, profil: dict) -> None:
        """Sensorik im laufenden Betrieb umschalten — der Bewerter nutzt das je Aufgabe.

        Jedes `sim`-Profil in config/tasks.json beschreibt eine andere Halle (anderes GPS,
        andere IMU-Rate, Funkloch). Statt für jede Aufgabe neu zu starten, werden die
        Sensoren neu aufgebaut: andere Rauschzugänge, sonst nichts. Die Odometrie beginnt
        an der aktuellen Pose, sonst hätte ein Profilwechsel einen Sprung in der Pose.

        Zweimal dasselbe Profil melden wird ignoriert — sonst würde jede Sekunde die
        Odometrie auf die Wahrheit zurückgesetzt und der IMU-Bias neu gewürfelt.
        """
        meldung = json.dumps(profil, sort_keys=True, default=str)
        if not profil or meldung == getattr(self, "_profil_meldung", None):
            return
        self._profil_meldung = meldung
        merge(self.cfg, dict(profil))
        if self.erzwungen:
            merge(self.cfg, dict(self.erzwungen))     # was per Hand gesetzt wurde, bleibt stehen
        self._lidar = sensors.Lidar(self.world, self._noise, cfg_get(self.cfg, "lidar"))
        self._gps = sensors.GpsSensor(self._noise, cfg_get(self.cfg, "gps"))
        self._gps.t0 = getattr(self, "t_task", self.t)
        for r in self.robots.values():
            r.odometer = sensors.OdometrySensor(r.chassis.geom, self._noise,
                                               cfg_get(self.cfg, "odom"))
            r.odometer.reset(r.chassis.pose)
            r.inertial = sensors.ImuSensor(self._noise, cfg_get(self.cfg, "imu"),
                                          robot=r.spec.name)
            self._zeitbezug(r)
        for art in ("gps", "odom", "imu", "truth"):
            for r in self.robots.values():
                r.ticks.pop(art, None)
        log.info("Sensorprofil umgestellt: gps σ=%.2f m %.1f Hz, imu %.0f Hz%s", float(
            cfg_get(self.cfg, "gps.sigma_xy", 0.0)), float(cfg_get(self.cfg, "gps.rate", 0.0)),
            float(cfg_get(self.cfg, "imu.rate", 0.0)),
            ", Funkloch " + str(cfg_get(self.cfg, "gps.gap")) if cfg_get(self.cfg, "gps.gap")
            else "")

    # ------------------------------------------------------------------- Kommandos (ROS)

    def set_cmd_vel(self, name: str, twist: Twist) -> None:
        r = self.robots.get(name)
        if r:
            r.vel_cmd, r.t_vel = twist, self.t

    def set_wheel_speeds(self, name: str, wheels) -> None:
        """Radgeschwindigkeiten [FL, FR, RL, RR] in rad/s — der Uebungskern."""
        r = self.robots.get(name)
        if not r or len(wheels or []) != 4:
            return
        r.mode = "wheels"                            # ab jetzt zaehlt nur noch das
        r.wheel_cmd, r.t_cmd = [float(w) for w in wheels], self.t

    def set_mission(self, name: str, state: str) -> None:
        r = self.robots.get(name)
        if r:
            r.mission_state = str(state)

    def set_kf(self, name: str, kf) -> None:
        """Eigene Schätzung eines Roboters annehmen (Versuch 2) und gegen die Wahrheit messen.

        Der Fehler hier ist nur fuer HUD und `./lab robots` da — bewertet wird im
        Bewerter (grade.py), weil der die Zeitfenster und die Messreihen kennt.
        """
        r = self.robots.get(name)
        if r and kf is not None:
            kf.t = self.t                                # Ankunft in Simulationszeit stempeln
            r.kf = kf
            r.kf_err = math.hypot(kf.x - r.pose.x, kf.y - r.pose.y)

    # ------------------------------------------------------------------------- Zeitschritt

    def step(self, dt: float) -> None:
        """Veraeffte Zeit in feste Physikschritte zerlegen (Determinismus)."""
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
            odo = r.odometer.update(r.chassis.wheels, dt)      # jeden Schritt integrieren
            for msg in r.inertial.sample(r.pose, r.twist, dt):  # IMU laeuft schneller als die Physik
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
                if fix is not None:                            # Funkloch: keine Meldung, kein letzter Fix
                    r.gps = fix
                    self._push("gps", r.spec.name, fix)
            if cfg_get(self.cfg, "debug_truth") and self._due(
                    r, "truth", cfg_get(self.cfg, "truth.rate", 20.0), dt):
                self._push("truth", r.spec.name, Pose(r.pose.x, r.pose.y, r.pose.theta))
        self.t += dt

    def _drive(self, r: Robot, dt: float) -> None:
        """Kommandos anwenden: eigene Radwerte schlagen cmd_vel, beides mit Watchdog."""
        wheels_ok = r.wheel_cmd is not None and 0 <= self.t - r.t_cmd <= self.cfg["cmd_timeout"]
        vel_ok = r.vel_cmd is not None and 0 <= self.t - r.t_vel <= self.cfg["cmd_timeout"]
        if r.mode == "wheels" and wheels_ok:
            r.chassis.set_wheels(r.wheel_cmd)
        elif r.mode == "pass-through" and vel_ok:
            v = r.vel_cmd
            r.chassis.set_wheels(physics.inverse_kinematics(
                r.chassis.geom, v.vx, v.vy, v.omega))
        else:
            r.chassis.set_wheels([0.0] * 4)          # Funkstille -> Motor ausrollen

    def _due(self, r: Robot, kind: str, rate: float, dt: float) -> bool:
        """Sensor-Rate-Einhaltung pro Roboter und Messgroesse."""
        period = 1.0 / max(rate, 0.001)
        acc = r.ticks.get(kind, 0.0) + dt
        if acc >= period:
            r.ticks[kind] = acc - period
            return True
        r.ticks[kind] = acc
        return False

    # ------------------------------------------------------------------------- Nachrichten

    def _push(self, kind: str, robot: str | None, payload) -> None:
        if len(self.outbox) > MAX_OUTBOX:            # Endlosschleife ohne Abnehmer
            del self.outbox[:MAX_OUTBOX // 2]
        if hasattr(payload, "t"):                    # Simulationszeit in die Messung pragen
            payload.t = self.t
        self.outbox.append((kind, robot, payload))

    def drain(self) -> list:
        """Alle offenen Messungen holen und die Outbox leeren (von der Bridge gerufen)."""
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
        """Kurzbeschreibung der Welt (Groesse, Ziel, Starts) — fuer Studierendenknoten."""
        g = self.world.goal
        return json.dumps({"name": self.world.name, "cell": self.world.cell,
                           "size": list(self.world.size),
                           "goal": None if not g else [g.x, g.y, g.theta],
                           "spawns": [[p.x, p.y, p.theta] for p in self.world.spawns],
                           "walls": len(self.world.walls)})

    def config_json(self) -> str:
        """Sensorprofil der laufenden Simulation — der Bewerter prüft darauf ihr Prüfprofil."""
        profil = {k: cfg_get(self.cfg, k) for k in
                  ("gps", "odom", "imu", "lidar", "truth", "rate", "debug_truth")}
        profil["seed"] = self.seed
        return json.dumps(profil)
