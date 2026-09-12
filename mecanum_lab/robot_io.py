"""Client für den studentischen Knoten: Messwerte hinein, Räder und Schätzung hinaus.

Themen (CONTRACT §3, CONTRACT-KF §2), die ersten relativ zum Roboternamen, die letzten absolut:
    cmd_vel        Twist         herein   Soll-Körpergeschwindigkeit (vx, vy, omega)
    wheel_speeds   4 rad/s       hinaus   [VL, VR, HL, HR] — die Aufgabe in Versuch 1
    odom gps scan  Odom Gps Scan herein   Dead-Reckoning, globale Pose, 360 LIDAR-Strahlen
    imu            Imu           herein   6-DOF-Trägheitsmessung (Versuch 2, 100 Hz)
    kf/pose        Kf            hinaus   eigene Schätzung inkl. 1σ (Versuch 2)
    mission_state  String        hinaus   idle | running | done | failed:<grund>
    /sim/task /sim/robots /sim/world /sim/config   Auftrag, Roboterliste, Welt, Sensorprofil
Läuft über echtes ROS (ros_bridge) oder den In-Prozess-Bus (stub.py) — derselbe Code im
ROS-Betrieb wie im `./lab run`-Schnelleinstieg. `serve()` ist der Runner dahinter.
"""
import json
import logging
import os
import sys
import time

from . import stub, types
from .types import Kf

log = logging.getLogger("mecanum.robotio")


def make_bus(node_name: str = "student"):
    """Echtes ROS, wenn es da und bereit ist; sonst der In-Prozess-Bus (kein ROS nötig)."""
    try:
        from . import ros_bridge                     # entsteht bei Agent B
        bus = ros_bridge.make_bus("auto", node_name=node_name)
        if bus is not None:
            return bus
    except Exception as exc:                         # kein ROS installiert/nicht gerodet
        log.debug("ros_bridge unbenutzbar (%s) — In-Prozess-Bus.", exc)
    return stub.get_bus()


def robot_info(bus, name: str) -> dict:
    # Eintrag des Roboters aus /sim/robots: mode, contacts, distance, mission, index
    payload, _ = bus.last("robots")
    try:
        return next((r for r in json.loads(payload or "[]") if r.get("name") == name), {})
    except (ValueError, TypeError):
        return {}


class RobotIO:
    """Ein Roboter von außen: Sensoren lesen, Räder und cmd_vel schreiben, Status melden."""

    def __init__(self, name: str, role: str = "controller", bus=None, cfg: dict | None = None,
                 stale_timeout: float = 2.0):
        self.name = types.sanitize_name(name)
        self.role = role
        self.bus = bus if bus is not None else make_bus(f"{role}_{self.name}")
        self.eigener_bus = bus is None and not isinstance(self.bus, stub.StubBus)
        self.cfg = cfg                                  # Sensorik-Konfiguration, faul geladen
        self.stale_timeout = float(stale_timeout)
        self._wheels, self._cmd = self.bus.pub("wheels", self.name), self.bus.pub("twist", self.name)
        self._state, self._gemeldet = self.bus.pub("mission", self.name), None
        self._kf, self._kfinfo = self.bus.pub("kf", self.name), self.bus.pub("kfinfo", self.name)
        self.ik = None            # serve() trägt hier module.inverse_kinematics ein, siehe dort

    def spin(self, timeout=0.01): self.bus.spin(timeout)
    def running(self) -> bool: return bool(self.bus.ok())
    def age(self, kind: str) -> float: return self.bus.last(kind, self.name)[1]   # 1e9: nie etwas
    def vorhanden(self) -> bool: return min(self.age("odom"), self.age("gps")) < self.stale_timeout

    def sleep(self, sec: float) -> None:
        """Schlafen und dabei zustellen — ohne spin() altern die Sensoren."""
        ende = time.monotonic() + max(sec, 0.0)
        while self.running() and time.monotonic() < ende:
            self.spin(min(0.02, ende - time.monotonic()))

    def _wert(self, kind: str, frisch: bool = False):
        """Sampler: letzter Wert eines Themas; `frisch` wendet cmd_timeout als Watchdog an."""
        wert, alter = self.bus.last(kind, self.name)
        if frisch:
            if self.cfg is None:
                self.cfg = types.load_config()
            return wert if alter <= float(self.cfg.get("cmd_timeout", 0.35)) else None
        return wert

    def odom(self): return self._wert("odom")
    def gps(self): return self._wert("gps")
    def scan(self): return self._wert("scan")
    def imu(self): return self._wert("imu")
    def kf(self): return self._wert("kf")
    def cmd_vel(self): return self._wert("twist", frisch=True)
    def task(self): return str(self.bus.last("task")[0] or "")
    def mission_state(self): return str(self.bus.last("mission", self.name)[0] or "")
    def robots(self) -> list: return json.loads(self.bus.last("robots")[0] or "[]")

    def sensor_profil(self) -> dict:
        """Aktives Sensorprofil der Simulation (gps/odom/imu …) — aus /sim/config.

        Nicht raten, welche Rauschwerte gerade gelten: hier stehen sie. Der Bewerter
        prüft dieselben Werte gegen das Prüfprofil des Auftrags.
        """
        try:
            return json.loads(self.bus.last("config")[0] or "{}")
        except ValueError:
            return {}

    def truth(self):
        """Exakte Pose — **nur** für Selbstkontrolle und Bewertung, nie im Filter verwenden!

        Gibt es nur, wenn die Simulation mit `debug_truth` läuft (sonst immer None); der
        Bewerter misst ohnehin gegen sie. Ein Filter, der hier reinschaut, ist kein Filter.
        """
        return self.bus.last("truth", self.name)[0]

    def world(self) -> dict:
        # {name, size, goal, spawns}: /sim/world, sonst direkt worlds/<name>.txt (ohne Sim-Takt)
        payload, _ = self.bus.last("world")
        try:
            if payload:
                return json.loads(payload)
        except ValueError:
            log.warning("world-Topic enthält kein JSON — Welt wird lokal gelesen.")
        try:
            from . import worlds
            w = worlds.load_world(str((self.cfg or types.load_config()).get("world")))
        except Exception as exc:
            log.warning("Welt nicht lesbar (%s) — Ziel unbekannt.", exc)
            return {}
        return {"name": w.name, "size": list(w.size),
                "goal": None if not w.goal else [w.goal.x, w.goal.y, w.goal.theta],
                "spawns": [[p.x, p.y, p.theta] for p in w.spawns]}

    def send_wheels(self, w) -> None:
        # Vier Radgeschwindigkeiten [VL, VR, HL, HR] in rad/s — Reihenfolge ist Vertrag
        try:
            rad = [float(v) for v in w]
        except (TypeError, ValueError):
            rad = []
        if len(rad) != 4 or not all(abs(v) < 1e6 for v in rad):
            raise ValueError(f"wheel_speeds braucht genau vier endliche Werte in rad/s: {w!r}")
        self._wheels(rad)

    def publish_cmd_vel(self, vx: float, vy: float = 0.0, omega: float = 0.0) -> None:
        """Körpergeschwindigkeit vorgeben — der Stellbefehl für T2..T4.

        Ist `ik` gesetzt (macht serve()), wandert der Befehl sofort durch die eigene
        inverse Kinematik auf wheel_speeds. Das ist Absicht: mission() läuft blockierend,
        der Runner könnte währenddessen nichts durchreichen — und so ist die eigene
        Kinematik in jedem Auftrag der einzige Stellpfad. Wer lieber direkt über
        send_wheels fährt, lässt publish_cmd_vel einfach weg.
        """
        self._cmd(types.Twist(vx=vx, vy=vy, omega=omega))
        if self.ik is not None:
            self.send_wheels(self.ik(vx, vy, omega))

    def set_mission_state(self, state: str) -> None:
        if state != self._gemeldet:                      # topics nicht zuspammen
            self._gemeldet = state
            self._state(str(state))

    def send_kf(self, x: float, y: float, theta: float = 0.0, sx: float = 0.0,
                sy: float = 0.0, sth: float = 0.0, info: dict | None = None) -> None:
        """Eigene Zustandsschätzung melden (Versuch 2) — inklusive Unsicherheit.

        `sx/sy/sth` sind Standardabweichungen (1σ) in m bzw. rad, keine Varianzen. Ohne
        diese Zahlen ist der Auftrag `kf_kovarianz` nicht bewertbar; ein Filter, der 5 m
        meldet und 0,1 m daneben liegt, hat nichts geschätzt. `info` geht als JSON auf
        `/<robot>/kf/info` und erscheint in der GUI — für Q, R, Zähler, was auch immer.
        """
        self._kf(Kf(t=0.0, x=float(x), y=float(y), theta=float(theta),
                    sx=abs(float(sx)), sy=abs(float(sy)), sth=abs(float(sth))))
        if info is not None:
            self._kfinfo(json.dumps(info, ensure_ascii=False))

    def config(self, pfad: str, standard=None):
        """Sensorik-Wert der laufenden Simulation, z. B. `rob.config("imu.rate")`.

        Erst die Werte aus `/sim/config` (die zählen wirklich), dann der lokale
        Config-Baum — damit ein vom Bewerter gesetztes Prüfprofil nicht übersehen wird.
        """
        wert = types.cfg_get(self.sensor_profil(), pfad, None)
        if wert is not None:
            return wert
        if self.cfg is None:
            self.cfg = types.load_config()
        return types.cfg_get(self.cfg, pfad, standard)

    def spawn(self, name: str, variant: str = "") -> dict:
        return self.bus.call(types.topic("spawn"), {"name": name, "variant": variant})

    def close(self) -> None:
        if self.eigener_bus:                             # gemeinsamer In-Prozess-Bus bleibt
            self.bus.shutdown()


def _name_from_argv(argv: list) -> str:
    for i, arg in enumerate(argv):
        if arg in ("--robot", "--name") and i + 1 < len(argv):
            return argv[i + 1]
    return argv[0] if argv and not argv[0].startswith("-") else ""


def serve(module, name: str | None = None, hz: float = 50.0, argv: list | None = None,
          bus=None, startup: float = 60.0) -> None:
    """Runner hinter der einen Zeile im Template: bei Task ""/"kinematik" wird jeden Tick
    cmd_vel in vier Räder übersetzt, sonst läuft module.mission(rob, task) genau einmal.
    Endet, wenn der Roboter verschwindet oder der Bus schließt."""
    name = (name or _name_from_argv(sys.argv[1:] if argv is None else argv)
            or os.environ.get("MECANUM_ROBOT") or "student")
    rob, dauer = RobotIO(name, bus=bus), 1.0 / min(max(float(hz), 1.0), 200.0)
    rob.ik = getattr(module, "inverse_kinematics", None)   # publish_cmd_vel -> eigene IK
    anfang, letzter, letzter_fehler, kam = time.monotonic(), None, 0.0, False
    log.info("Knoten '%s' startet für Roboter '%s' (%s).", getattr(module, "__name__", "?"),
             rob.name, "ROS" if rob.eigener_bus else "In-Prozess-Bus")
    while rob.running():
        kam = kam or rob.age("odom") < 1e8 or rob.age("gps") < 1e8
        if not kam and time.monotonic() - anfang < startup:
            rob.spin(0.05)                               # noch auf den Spawn warten
            continue
        if not kam or not rob.vorhanden():               # Roboter weg oder Funkstille
            log.info("Kein Roboter '%s' (spawnen: ./lab spawn --name %s) oder keine Daten mehr.",
                     rob.name, rob.name)
            break
        task, start = rob.task(), time.monotonic()
        kinematik = task in ("", "kinematik")
        if task != letzter:
            letzter, _ = task, rob.set_mission_state("idle" if kinematik else "running")
            if not kinematik:
                try:
                    module.mission(rob, task)            # Mission bringt ihre eigene Schleife mit
                    rob.set_mission_state("done")
                except Exception as exc:                 # Fehler zeigen, nicht schlucken
                    log.exception("mission('%s') fehlgeschlagen", task)
                    rob.set_mission_state(f"failed:{type(exc).__name__}: {exc}")
                continue
        befehl = rob.cmd_vel() if kinematik else None
        if befehl is not None:
            try:
                rob.send_wheels(module.inverse_kinematics(befehl.vx, befehl.vy, befehl.omega))
            except Exception as exc:
                if time.monotonic() - letzter_fehler > 2.0:
                    letzter_fehler = time.monotonic()
                    log.error("inverse_kinematics(%.2f, %.2f, %.2f) -> %s: %s",
                              befehl.vx, befehl.vy, befehl.omega, type(exc).__name__, exc)
        rob.spin(max(0.0, dauer - (time.monotonic() - start)))
    rob.close()
