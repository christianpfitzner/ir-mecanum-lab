"""ROS-2-Anbindung: dieselbe Bus-Oberfläche wie `stub.StubBus`, aber echte Topics.

Zwei Dinge passieren hier und sonst nirgends:

1. **Typumwandlung.** Intern rechnet der Simulator nur mit Dataclasses aus `types.py`
   (Twist, Odom, Scan, Gps, String, vier Radgeschwindigkeiten). Erst hier entstehen
   echte ROS-Meldungen (`Odometry`, `LaserScan`, `PoseStamped`, …) — und beim
   Abonnieren wieder zurück. Deshalb kann `engine.py` ROS nicht sehen.
2. **Services.** ROS 2 hat kein eingebautes String-Service, `std_srvs` kennt nur
   Empty/SetBool/Trigger. Für `spawn_robot`/`despawn_robot` wird deshalb das kleine
   eigene Interface `mecanum_lab_interfaces/srv/SpawnRobot` benutzt, **wenn** es gebaut
   wurde; sonst weicht der Bus auf einen JSON-Handshake aus
   (`/sim/spawn_robot/request` -> `/sim/spawn_robot/result`). Der Aufrufer merkt davon
   nichts, `handler(dict) -> dict` gilt in beiden Fällen.

`make_bus("auto")` liefert `None`, wenn kein rclpy da ist — `robot_io` und `node`
fallen dann auf den In-Prozess-Bus zurück. Bewusst überall Standard-QoS (reliable):
ein Best-Effort-Publisher und ein Subscriber mit Default-QoS finden nicht zusammen,
und genau das wäre die erste frustrierende Erfahrung im Praktikum.
"""
import json
import logging
import math
import os
import threading
import time

from . import stub
from .types import Gps, Imu, Kf, Odom, Pose, Scan, Twist, topic

log = logging.getLogger("mecanum.ros")

# kind -> (Klasse im M-Wörterbuch, Richtung) ; Richtung "p" = Sim sendet, "s" = Sim empfängt
KIND_MSG = {"twist": "Twist", "wheels": "Float64MultiArray", "odom": "Odometry",
            "scan": "LaserScan", "gps": "PoseStamped", "truth": "PoseStamped",
            "imu": "Imu", "kf": "PoseWithCovarianceStamped", "kfinfo": "String",
            "mission": "String", "robots": "String", "world": "String",
            "task": "String", "config": "String", "clock": "Clock"}
FRAME = {"odom": ("odom", "base_link"), "gps": ("map", None), "truth": ("map", None),
         "kf": ("map", None), "imu": ("imu_link", None), "scan": ("laser", None)}
# Platzhalter-Unsicherheit der IMU-Baugruppe (Diagonale), damit RViz und rqt nicht mit
# Null-Kovarianzen arbeiten. Wer genauere Zahlen will: sie gehoeren in den Filter, nicht
# in den Treiber — der Filter kennt seinen eigenen Zustand.
IMU_COV = {"angular": 1.0e-4, "linear": 4.0e-4, "orientation": 1.0e-4}
_M, _M_LOCK = None, threading.Lock()


def load_msgs(force: bool = False):
    """Alle ROS-Nachrichtklassen einmal importieren; None, wenn kein ROS da ist."""
    global _M
    with _M_LOCK:
        if _M is not None and not force:
            return _M
        if os.environ.get("MECANUM_ROS", "").lower() == "stub":
            return None
        try:
            from builtin_interfaces.msg import Time
            from geometry_msgs.msg import (Pose as P, PoseStamped,
                                           PoseWithCovarianceStamped as PWCS,
                                           Quaternion, Twist as T, Vector3)
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu, LaserScan
            from std_msgs.msg import Float64MultiArray, String
            from rosgraph_msgs.msg import Clock
            try:
                from rclpy.msg import Header                    # ab ROS 2 Kilted
            except ImportError:
                from std_msgs.msg import Header                 # bis ROS 2 Jazzy
        except ImportError:
            return None
        _M = {"Time": Time, "Header": Header, "String": String, "Clock": Clock,
              "Twist": T, "Vector3": Vector3, "Pose": P, "Quaternion": Quaternion,
              "PoseStamped": PoseStamped, "PoseWithCovarianceStamped": PWCS,
              "Odometry": Odometry, "LaserScan": LaserScan, "Imu": Imu,
              "Float64MultiArray": Float64MultiArray}
        return _M


# ------------------------------------------------------- reine Umrechnung (ohne rclpy testbar)


def yaw_to_quat(yaw: float) -> tuple:
    """Drehung um z (CCW) als Quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q) -> float:
    """Gierwinkel aus Quaternion — für empfangene PoseStamped-Meldungen."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cov36(diag: tuple) -> list:
    """Diagonale -> 6x6-Kovarianz in zeilenmajorer Reihenfolge (36 Einträge)."""
    m = [0.0] * 36
    for i, v in enumerate(diag[:6]):
        m[i * 7] = v
    return m


def cov_diag(cov, n: int = 6) -> list:
    """Diagonale aus einer 6x6- (oder 3x3-)Kovarianz herauslesen — die Umkehrung von cov36.

    Unter ROS ist `cov` ein numpy-artiges Feld: ein `cov or []` waere dort ein mehrdeutiger
    Wahrheitswert (ValueError in der Callback-Spur), also wird immer ueber die Laenge gegangen.
    """
    total = len(cov) if cov is not None else 0
    wide = int(round(math.sqrt(total))) if total else 0
    if wide < 1:
        return [0.0] * n
    return [float(cov[i * (wide + 1)]) if i * (wide + 1) < total else 0.0
            for i in range(min(n, wide))]


def _header(M, t: float, frame: str):
    h = M["Header"](frame_id=frame)
    h.stamp = M["Time"](sec=int(t), nanosec=int((t % 1.0) * 1e9))
    return h


def _set_pose(M, pose_msg, x, y, yaw) -> None:
    pose_msg.position.x, pose_msg.position.y = float(x), float(y)
    qx, qy, qz, qw = yaw_to_quat(yaw)
    pose_msg.orientation.x, pose_msg.orientation.y = qx, qy
    pose_msg.orientation.z, pose_msg.orientation.w = qz, qw


def to_ros(M, kind: str, payload):
    """Dataclass -> ROS-Meldung. `M` from `load_msgs()` (oder ein Test-Attrappen-Wörterbuch)."""
    if kind == "twist":
        return M["Twist"](linear=M["Vector3"](x=payload.vx, y=payload.vy),
                          angular=M["Vector3"](z=payload.omega))
    if kind == "wheels":
        return M["Float64MultiArray"](data=[float(w) for w in payload])
    if kind == "mission" or kind in ("robots", "world", "task", "config", "kfinfo"):
        return M["String"](data=str(payload))
    if kind == "clock":
        return M["Clock"](clock=M["Time"](sec=int(payload), nanosec=int((payload % 1) * 1e9)))
    if kind == "odom":
        m = M["Odometry"]()
        m.header = _header(M, payload.t, FRAME["odom"][0])
        m.child_frame_id = FRAME["odom"][1]
        _set_pose(M, m.pose.pose, payload.x, payload.y, payload.theta)
        m.pose.covariance = cov36((0.02, 0.02, 1e6, 1e6, 1e6, 0.05))
        m.twist.twist.linear.x, m.twist.twist.linear.y = payload.vx, payload.vy
        m.twist.twist.angular.z = payload.omega
        m.twist.covariance = cov36((0.05, 0.05, 1e6, 1e6, 1e6, 0.02))
        return m
    if kind in ("gps", "truth"):
        m = M["PoseStamped"]()
        m.header = _header(M, payload.t if hasattr(payload, "t") else 0.0, FRAME[kind][0])
        _set_pose(M, m.pose, payload.x, payload.y, payload.theta)
        return m
    if kind == "kf":
        m = M["PoseWithCovarianceStamped"]()
        m.header = _header(M, payload.t, FRAME["kf"][0])
        _set_pose(M, m.pose.pose, payload.x, payload.y, payload.theta)
        # ROS legt die Pose-Kovarianz als [x, y, z, roll, pitch, yaw] ab — unten diagonal
        m.pose.covariance = cov36((payload.sx ** 2, payload.sy ** 2, 1e-12, 1e-12, 1e-12,
                                   payload.sth ** 2))
        return m
    if kind == "imu":
        m = M["Imu"]()
        m.header = _header(M, payload.t, FRAME["imu"][0])
        qx, qy, qz, qw = yaw_to_quat(0.0)
        m.orientation.x, m.orientation.y = qx, qy
        m.orientation.z, m.orientation.w = qz, qw
        m.orientation_covariance = cov36((IMU_COV["orientation"],) * 6)
        m.angular_velocity.x, m.angular_velocity.y = payload.gx, payload.gy
        m.angular_velocity.z = payload.gz
        m.angular_velocity_covariance = cov36((IMU_COV["angular"],) * 6)
        m.linear_acceleration.x, m.linear_acceleration.y = payload.ax, payload.ay
        m.linear_acceleration.z = payload.az
        m.linear_acceleration_covariance = cov36((IMU_COV["linear"],) * 6)
        return m
    if kind == "scan":
        m = M["LaserScan"]()
        m.header = _header(M, payload.t, FRAME["scan"][0])
        n = max(len(payload.ranges) - 1, 1)
        m.angle_min, m.angle_max = payload.angle_min, payload.angle_min + payload.angle_increment * n
        m.angle_increment, m.scan_time = payload.angle_increment, 0.02
        m.time_increment = m.angle_increment / max(360 * 10, 1)
        m.range_min, m.range_max = payload.range_min, payload.range_max
        # Ein Laser meldet "kein Echo" als Reichweite, nicht als unendlich.
        m.ranges = [payload.range_max if r != r or r == math.inf or r > payload.range_max
                    else float(r) for r in payload.ranges]
        m.intensities = []
        return m
    raise ValueError(f"unbekannter Msg-Typ {kind}")


def from_ros(kind: str, msg):
    """ROS-Meldung -> Dataclass, damit dieselbe Leserseite in ROS und im Stub gilt."""
    if kind == "twist":
        return Twist(msg.linear.x, msg.linear.y, msg.angular.z)
    if kind == "wheels":
        return [float(v) for v in msg.data]
    if kind in ("task", "mission", "robots", "world", "config", "kfinfo"):
        return msg.data
    if kind == "odom":
        p, v = msg.pose.pose, msg.twist.twist
        return Odom(_zeit(msg.header), p.position.x, p.position.y,
                    quat_to_yaw(p.orientation), v.linear.x, v.linear.y, v.angular.z)
    if kind in ("gps", "truth"):
        p = msg.pose.position
        return Gps(_zeit(msg.header), p.x, p.y, quat_to_yaw(msg.pose.orientation))
    if kind == "kf":
        p, cov = msg.pose.pose, cov_diag(msg.pose.covariance)
        return Kf(_zeit(msg.header), p.position.x, p.position.y, quat_to_yaw(p.orientation),
                  math.sqrt(max(cov[0], 0.0)), math.sqrt(max(cov[1], 0.0)),
                  math.sqrt(max(cov[5], 0.0)))
    if kind == "imu":
        g, a = msg.angular_velocity, msg.linear_acceleration
        return Imu(_zeit(msg.header), a.x, a.y, a.z, g.x, g.y, g.z)
    if kind == "scan":
        return Scan(_zeit(msg.header), msg.angle_min, msg.angle_increment, msg.range_min,
                    msg.range_max, list(msg.ranges))
    raise ValueError(f"unbekannter Abo-Typ {kind}")


def _zeit(header) -> float:
    """Header-Stempel in Sekunden (Simulationszeit, wenn use_sim_time)."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def spawn_handler(engine) -> callable:
    """Handler für spawn_robot: SpawnError wird zur Meldung, nie zur Ausnahme."""
    def handle(req: dict) -> dict:
        try:
            r = engine.spawn(str(req.get("name", "")), str(req.get("variant", "")))
            return {"success": True, "message": "ok", "index": r.spec.index,
                    "color": r.spec.color, "marker": r.spec.marker, "variant": r.spec.variant,
                    "x": r.pose.x, "y": r.pose.y, "theta": r.pose.theta}
        except Exception as exc:
            return {"success": False, "message": str(exc)}
    return handle


def despawn_handler(engine) -> callable:
    def handle(req: dict) -> dict:
        name = str(req.get("name", ""))
        ok = engine.despawn(name)
        return {"success": ok, "message": f"'{name}' entfernt" if ok else f"'{name}' nicht gefunden"}
    return handle


# ------------------------------------------------------------------ der echte ROS-2-Bus


class RclpyBus:
    """Oberflächen-kompatibel zu `stub.StubBus` (pub/publish/sub/sub_topic/last/service/call/spin/ok)."""

    def __init__(self, node_name: str = "mecanum_sim", M: dict | None = None):
        import rclpy
        from rclpy.node import Node
        self.rclpy, self.M = rclpy, M or load_msgs()
        if self.M is None:
            raise RuntimeError("rclpy ist da, aber die Message-Klassen nicht — ROS korrekt sourcen?")
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.name, self._pubs, self._last, self._srv, self._cli = node_name, {}, {}, {}, {}
        self._abos, self._iface = set(), _spawn_iface(self.M)
        log.info("ROS-Knoten '%s' (%s)", node_name, "SpawnRobot-Interface" if self._iface
                 else "Spawn-Fallback: JSON-Handshake")

    # ------------------------------------------------------------------ publisher-seitig

    def pub(self, kind: str, robot: str | None = None):
        key = (kind, robot)
        if key not in self._pubs:
            self._pubs[key] = self.node.create_publisher(self.M[KIND_MSG[kind]],
                                                         topic(kind, robot), 10)
        p, M = self._pubs[key], self.M

        def send(payload):
            p.publish(to_ros(M, kind, payload))
        return send

    def publish(self, name: str, payload) -> None:
        """Themenname statt kind — den brauchen nur die Sonderfälle (Handshake-Antwort)."""
        self.pub_string(name, str(payload))

    # ------------------------------------------------------------------ subscriber-seitig

    def sub(self, kind: str, robot: str | None, cb) -> None:
        """Abo mit echtem Meldungs­typ; `cb` bekommt wieder die Dataclass aus types.py."""
        self._subscribe(KIND_MSG[kind], topic(kind, robot),
                        lambda msg, k=kind, c=cb: self._recv(topic(k, robot), c, from_ros(k, msg)))

    def sub_topic(self, name: str, cb) -> None:
        """Abo auf ein String-Thema; `cb` bekommt den Klartext (JSON oder Auftragstitel)."""
        self._subscribe("String", name, lambda msg, n=name, c=cb: self._recv(n, c, msg.data))

    def _subscribe(self, cls_key: str, name: str, wrap) -> None:
        self._abos.add(name)
        self.node.create_subscription(self.M[cls_key], name, wrap, 10)

    def _recv(self, name, cb, value) -> None:
        self._last[name] = (value, time.monotonic())
        try:
            cb(value)
        except Exception:
            log.exception("Subscriber auf %s", name)

    def last(self, kind: str, robot: str | None = None) -> tuple:
        """Letzter Wert + Alter in s. Beim ersten Hinschauen wird das Thema nebenbei abonniert —
        sonst wäre `last()` in ROS für immer leer, weil der Leser selbst nie abonniert hat."""
        name = topic(kind, robot)
        if name not in self._abos:
            self._subscribe(KIND_MSG[kind], name,
                            lambda msg, n=name, k=kind: self._recv(n, lambda p: None, from_ros(k, msg)))
        got = self._last.get(name)
        return (got[0], time.monotonic() - got[1]) if got else (None, 1e9)

    # ------------------------------------------------------------------------- services

    def service(self, name: str, handler) -> None:
        self._srv[name] = handler
        if name.endswith(("spawn_robot", "despawn_robot")):
            if self._iface:
                srv = self._iface
                def on(req, resp, handler=handler):
                    out = handler({"name": req.name, "variant": getattr(req, "variant", "")})
                    resp.success = bool(out.get("success"))
                    resp.message = str(out.get("message", ""))
                    resp.index = int(out.get("index", 0))
                    resp.color = str(out.get("color", ""))
                    resp.marker = str(out.get("marker", ""))
                    resp.x = float(out.get("x", 0.0))
                    resp.y = float(out.get("y", 0.0))
                    resp.theta = float(out.get("theta", 0.0))
                    return resp
                self.node.create_service(srv, name, lambda req, resp, f=on: f(req, resp))
            else:
                self.node.create_subscription(
                    self.M["String"], name + "/request",
                    lambda msg, n=name, h=handler: self._handshake(n, h, msg), 10)
        else:
            from std_srvs.srv import Trigger
            self.node.create_service(Trigger, name,
                                     lambda req, resp, h=handler: _trigger(resp, h, name))

    def _handshake(self, name, handler, msg) -> None:
        try:
            req = json.loads(msg.data)
        except ValueError:
            req = {"name": msg.data}
        out = dict(handler(req) or {})
        out["id"] = req.get("id")
        self.pub_string(name + "/result", json.dumps(out))

    def pub_string(self, name: str, text: str) -> None:
        if name not in self._pubs:
            self._pubs[name] = self.node.create_publisher(self.M["String"], name, 10)
        self._pubs[name].publish(self.M["String"](data=text))

    def call(self, name: str, req: dict, timeout: float = 2.0) -> dict:
        """Service-Aufruf (typend) oder Handshake — aufrufen, solange nichts anderes spinnt."""
        if name in self._srv:                               # derselbe Prozess: direkt
            return self._srv[name](dict(req or {}))
        if self._iface:
            client = self._cli.get(name) or self.node.create_client(self._iface, name)
            self._cli[name] = client
            if not client.wait_for_service(timeout_sec=1.0):
                return {"success": False, "message": f"Service {name} nicht erreichbar"}
            r = self._iface.Request()
            r.name, r.variant = str(req.get("name", "")), str(req.get("variant", ""))
            future = client.call_async(r)
            return self._await(future, lambda res: {"success": bool(res.success),
                                                    "message": res.message, "index": res.index,
                                                    "color": res.color, "marker": res.marker,
                                                    "x": res.x, "y": res.y,
                                                    "theta": res.theta}, timeout)
        got = {}
        mine = f"{time.monotonic():.6f}"

        def on_result(msg, mine=mine, got=got):
            try:
                out = json.loads(msg.data)
            except ValueError:
                return
            if out.get("id") == mine:
                got.update(out)

        self.node.create_subscription(self.M["String"], name + "/result", on_result, 10)
        self.pub_string(name + "/request", json.dumps({**req, "id": mine}))
        ende = time.monotonic() + timeout
        while not got and time.monotonic() < ende:
            self.spin(0.02)
        return got or {"success": False, "message": f"keine Antwort von {name}"}

    def _await(self, future, deutung, timeout):
        ende = time.monotonic() + timeout
        while not future.done() and time.monotonic() < ende and self.rclpy.ok():
            self.spin(0.02)
        if not future.done():
            return {"success": False, "message": "Zeit abgelaufen"}
        return deutung(future.result())

    # ------------------------------------------------------------------ lebenszyklus

    def spin(self, timeout: float = 0.01) -> None:
        if self.rclpy.ok():
            self.rclpy.spin_once(self.node, timeout_sec=min(max(timeout, 0.0), 0.05))

    def ok(self) -> bool:
        return self.rclpy.ok()

    def shutdown(self) -> None:
        try:
            self.node.destroy_node()
            if self.rclpy.ok():
                self.rclpy.shutdown()
        except Exception:
            pass


def _trigger(resp, handler, name):
    out = handler({}) or {}
    resp.success = bool(out.get("success", True))
    resp.message = str(out.get("message", ""))
    return resp


def _spawn_iface(M):
    """Das kleine Eigeninterface, falls gebaut — sonst None (dann JSON-Handshake)."""
    try:
        from mecanum_lab_interfaces.srv import SpawnRobot
        return SpawnRobot
    except ImportError:
        return None


def make_bus(kind: str = "auto", node_name: str = "mecanum_sim"):
    """"auto": echtes ROS, wenn vorhanden, sonst None (= Aufrufer nimmt stub). None ist hier
    ein Ergebnis, kein Fehler: so bleibt `./lab run` ohne ROS installierbar."""
    if kind == "stub":
        return stub.get_bus()
    if load_msgs() is None:
        if kind == "ros":
            raise RuntimeError("kein ROS 2 (rclpy) gefunden — bitte source /opt/ros/$ROS_DISTRO/setup.bash")
        return None
    try:
        return RclpyBus(node_name)
    except Exception as exc:
        if kind == "ros":
            raise
        log.warning("ROS nicht nutzbar (%s) — In-Prozess-Bus.", exc)
        return None
