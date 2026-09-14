"""ROS 2 bridge: the same bus interface as `stub.StubBus`, but with real topics.

Two things happen here and nowhere else:

1. **Type conversion.** Internally the simulator only works with dataclasses from
   `types.py` (Twist, Odom, Scan, Gps, String, four wheel speeds). Real ROS messages
   (`Odometry`, `LaserScan`, `PoseStamped`, …) are built here — and converted back
   when subscribing. That is why `engine.py` never sees ROS.
2. **Services.** ROS 2 has no built-in string service, `std_srvs` only knows
   Empty/SetBool/Trigger. `spawn_robot`/`despawn_robot` therefore use the small own
   interface `mecanum_lab_interfaces/srv/SpawnRobot`, **if** it was built; otherwise
   the bus falls back to a JSON handshake
   (`/sim/spawn_robot/request` -> `/sim/spawn_robot/result`). The caller does not
   notice, `handler(dict) -> dict` holds in both cases.

`make_bus("auto")` returns `None` when rclpy is missing — `robot_io` and `node` then
fall back to the in-process bus. Standard QoS (reliable) everywhere on purpose: a
best-effort publisher and a subscriber with default QoS never find each other, and
that is exactly the first frustrating experience of the lab course.
"""
import json
import logging
import math
import os
import threading
import time

from . import stub, tf_bcast
from .types import Gps, Imu, Kf, Link, Odom, Poi, Scan, SensorInfo, Twist, topic

log = logging.getLogger("mecanum.ros")

# kind -> (class in the M dict, direction); direction "p" = sim sends, "s" = sim receives
KIND_MSG = {"twist": "Twist", "wheels": "Float64MultiArray", "odom": "Odometry",
            "scan": "LaserScan", "gps": "PoseStamped", "truth": "PoseStamped",
            # The fix mirrored with its uncertainty, from the same payload and the same stamp — see
            # the comment at types.MSG_SPECS["gpscov"] for why /gps itself stays a PoseStamped.
            "gpscov": "PoseWithCovarianceStamped",
            "imu": "Imu", "kf": "PoseWithCovarianceStamped", "kfinfo": "String",
            "mission": "String", "robots": "String", "world": "String",
            "task": "String", "config": "String", "clock": "Clock", "poi": "String",
            "link": "String", "sensorinfo": "String"}
# Frame names live in tf_bcast: the same names in the message headers and in /tf, each
# carrying the robot as prefix. Nothing here invents frame names of its own.
# Which kind is *read* from which topic where they differ; publishing is unaffected. Reading the GPS
# fix off its covariance topic is why `rob.gps().sigma_xy` is never a silent 0.
READS = {"gps": "gpscov"}
# Placeholder uncertainty of the IMU assembly (diagonal), so RViz and rqt do not work
# with zero covariances. Anyone who wants tighter numbers: they belong in the filter,
# not in the driver — the filter knows its own state.
IMU_COV = {"angular": 1.0e-4, "linear": 4.0e-4, "orientation": 1.0e-4}
_M, _M_LOCK = None, threading.Lock()


def load_msgs(force: bool = False):
    """Import every ROS message class once; None when no ROS is present."""
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
                                           Quaternion, Transform, TransformStamped, Twist as T,
                                           Vector3)
            from nav_msgs.msg import Odometry
            from sensor_msgs.msg import Imu, LaserScan
            from std_msgs.msg import Float64MultiArray, String
            from rosgraph_msgs.msg import Clock
            try:
                from tf2_msgs.msg import TFMessage                # /tf and /tf_static
            except ImportError:
                TFMessage = None
            try:
                from rclpy.msg import Header                    # from ROS 2 Kilted
            except ImportError:
                from std_msgs.msg import Header                 # up to ROS 2 Jazzy
        except ImportError:
            return None
        _M = {"Time": Time, "Header": Header, "String": String, "Clock": Clock,
              "Twist": T, "Vector3": Vector3, "Pose": P, "Quaternion": Quaternion,
              "PoseStamped": PoseStamped, "PoseWithCovarianceStamped": PWCS,
              "Odometry": Odometry, "LaserScan": LaserScan, "Imu": Imu,
              "Float64MultiArray": Float64MultiArray, "Transform": Transform,
              "TransformStamped": TransformStamped, "TFMessage": TFMessage}
        return _M


# ------------------------------------------------------ pure conversion (testable without rclpy)


def yaw_to_quat(yaw: float) -> tuple:
    """Rotation about z (CCW) as a quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q) -> float:
    """Yaw angle from a quaternion — for received PoseStamped messages."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def cov36(diag: tuple) -> list:
    """Diagonal -> 6x6 covariance in row-major order (36 entries)."""
    m = [0.0] * 36
    for i, v in enumerate(diag[:6]):
        m[i * 7] = v
    return m


def cov_diag(cov, n: int = 6) -> list:
    """Read the diagonal out of a 6x6 (or 3x3) covariance — the inverse of cov36.

    Under ROS `cov` is a numpy-like array: `cov or []` would be an ambiguous truth value
    there (ValueError in the callback traceback), so the code always goes by its length.
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


def to_ros(M, kind: str, payload, robot: str | None = None, cfg: dict | None = None):
    """Dataclass -> ROS message. `M` from `load_msgs()` (or a test dummy dictionary).

    `robot` and `cfg` only decide the frame names (tf_bcast), never the content: the same
    message comes out as before, its header simply carries the robot prefix.
    """
    if kind == "twist":
        return M["Twist"](linear=M["Vector3"](x=payload.vx, y=payload.vy),
                          angular=M["Vector3"](z=payload.omega))
    if kind == "wheels":
        return M["Float64MultiArray"](data=[float(w) for w in payload])
    if kind == "mission" or kind in ("robots", "world", "task", "config", "kfinfo"):
        return M["String"](data=str(payload))
    if kind == "poi":
        # A stamp and a number, and still JSON on a String (CONTRACT §6.13): ROS 2 has no standard
        # message for a stamped scalar — `std_msgs/Float64` carries no header, and a header without a
        # value is not a reading — while this pattern needs no interface package between a student and
        # `ros2 topic echo`. Same reason as `/link` below, and the same reason `types.Poi` has exactly
        # these two fields: what is echoed is what the simulator holds.
        return M["String"](data=json.dumps({"t": payload.t, "intensity": payload.intensity}))
    if kind == "link":
        # /link is JSON on a String for the same reason as /poi (CONTRACT §6.14): a link budget has no
        # standard message, and a custom interface would put a build step in front of a student who
        # only wants to read a quality. All seven fields of types.Link, one object, echoable.
        return M["String"](data=json.dumps({"t": payload.t, "quality": payload.quality,
                                            "rssi_dbm": payload.rssi_dbm, "ap": list(payload.ap),
                                            "up": payload.up, "dropped": payload.dropped,
                                            "latency_ms": payload.latency_ms}))
    if kind == "sensorinfo":
        # The five numbers about the instruments, as JSON (CONTRACT §6.4): `PoseStamped`, `Imu` and
        # `LaserScan` have no field for a quality, a satellite count, a loss count, a latency or a
        # chip temperature, so without this message they would exist only in the window and in the
        # log — which is the one thing a student cannot check from a second terminal.
        return M["String"](data=json.dumps({"t": payload.t, "quality": payload.quality,
                                            "sats": payload.sats, "lost": payload.lost,
                                            "latency_ms": payload.latency_ms,
                                            "temp": payload.temp, "scan_gaps": payload.scan_gaps}))
    if kind == "clock":
        return M["Clock"](clock=M["Time"](sec=int(payload), nanosec=int((payload % 1) * 1e9)))
    if kind == "odom":
        m = M["Odometry"]()
        kopf, kind_frame = tf_bcast.header_frames("odom", robot, cfg)
        m.header = _header(M, payload.t, kopf)
        m.child_frame_id = kind_frame
        _set_pose(M, m.pose.pose, payload.x, payload.y, payload.theta)
        m.pose.covariance = cov36((0.02, 0.02, 1e6, 1e6, 1e6, 0.05))
        m.twist.twist.linear.x, m.twist.twist.linear.y = payload.vx, payload.vy
        m.twist.twist.angular.z = payload.omega
        m.twist.covariance = cov36((0.05, 0.05, 1e6, 1e6, 1e6, 0.02))
        return m
    if kind in ("gps", "truth"):
        m = M["PoseStamped"]()
        m.header = _header(M, payload.t if hasattr(payload, "t") else 0.0,
                           tf_bcast.frame_for(kind, robot, cfg))
        _set_pose(M, m.pose, payload.x, payload.y, payload.theta)
        return m
    if kind == "gpscov":
        # The fix of `/gps` again, with the R of that one emission. `sats` and `quality` have no
        # place in this message either — `/sensor/info` is where the instrument reports them.
        m = M["PoseWithCovarianceStamped"]()
        m.header = _header(M, payload.t, tf_bcast.frame_for("gps", robot, cfg))
        _set_pose(M, m.pose.pose, payload.x, payload.y, payload.theta)
        # ROS stores the pose covariance as [x, y, z, roll, pitch, yaw] — on the diagonal, squared
        m.pose.covariance = cov36((payload.sigma_xy ** 2, payload.sigma_xy ** 2, 1e-12, 1e-12,
                                   1e-12, payload.sigma_theta ** 2))
        return m
    if kind == "kf":
        m = M["PoseWithCovarianceStamped"]()
        m.header = _header(M, payload.t, tf_bcast.frame_for("kf", robot, cfg))
        _set_pose(M, m.pose.pose, payload.x, payload.y, payload.theta)
        # ROS stores the pose covariance as [x, y, z, roll, pitch, yaw] — on the diagonal
        m.pose.covariance = cov36((payload.sx ** 2, payload.sy ** 2, 1e-12, 1e-12, 1e-12,
                                   payload.sth ** 2))
        return m
    if kind == "imu":
        m = M["Imu"]()
        m.header = _header(M, payload.t, tf_bcast.frame_for("imu", robot, cfg))
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
        m.header = _header(M, payload.t, tf_bcast.frame_for("scan", robot, cfg))
        n = max(len(payload.ranges) - 1, 1)
        m.angle_min, m.angle_max = payload.angle_min, payload.angle_min + payload.angle_increment * n
        m.angle_increment, m.scan_time = payload.angle_increment, 0.02
        m.time_increment = m.angle_increment / max(360 * 10, 1)
        m.range_min, m.range_max = payload.range_min, payload.range_max
        # "No echo" has two dialects. The simulator has always spoken the first one — the missing beam comes
        # back as the laser's own range, which is what every laboratory here has measured since the start, so
        # it stays the default. The second is what `sensor_msgs/msg/LaserScan` documents and what a mapper
        # understands: infinity, „this beam left and did not come back". The difference is not cosmetic — a
        # beam beyond its `max_laser_range` is dropped by slam_toolbox rather than traced past, so with the
        # first dialect a hall that is open in every direction arrives as a map with one metre of floor on it.
        far = float("inf") if str(tf_bcast.cfg_get(cfg, "sensor.lidar.no_echo", "range_max")).lower() == "inf" \
            else payload.range_max
        m.ranges = [far if r != r or r == math.inf or r > payload.range_max
                    else float(r) for r in payload.ranges]
        m.intensities = []
        return m
    raise ValueError(f"unknown message type {kind}")


def from_ros(kind: str, msg):
    """ROS message -> dataclass, so the same reading side applies in ROS and in the stub."""
    if kind == "twist":
        return Twist(msg.linear.x, msg.linear.y, msg.angular.z)
    if kind == "wheels":
        return [float(v) for v in msg.data]
    if kind in ("task", "mission", "robots", "world", "config", "kfinfo"):
        return msg.data
    if kind == "poi":
        got = json.loads(msg.data)                       # the JSON form of to_ros(), see above
        return Poi(got.get("t", 0.0), got.get("intensity", 0.0))
    if kind == "link":
        got = json.loads(msg.data)                       # the JSON form of to_ros(), see above
        return Link(got.get("t", 0.0), got.get("quality", 1.0), got.get("rssi_dbm", 0.0),
                    tuple(got.get("ap") or (0.0, 0.0)), got.get("up", True),
                    got.get("dropped", 0), got.get("latency_ms", 0.0))
    if kind == "sensorinfo":
        got = json.loads(msg.data)                       # the JSON form of to_ros(), see above
        return SensorInfo(got.get("t", 0.0), got.get("quality", 0), got.get("sats", 0),
                          got.get("lost", 0), got.get("latency_ms", 0.0), got.get("temp", 0.0),
                          got.get("scan_gaps", 0))
    if kind == "odom":
        p, v = msg.pose.pose, msg.twist.twist
        return Odom(_stamp(msg.header), p.position.x, p.position.y,
                    quat_to_yaw(p.orientation), v.linear.x, v.linear.y, v.angular.z)
    if kind in ("gps", "truth"):
        p = msg.pose.position
        return Gps(_stamp(msg.header), p.x, p.y, quat_to_yaw(msg.pose.orientation))
    if kind == "gpscov":
        p, cov = msg.pose.pose, cov_diag(msg.pose.covariance)
        return Gps(_stamp(msg.header), p.position.x, p.position.y, quat_to_yaw(p.orientation),
                   sigma_xy=math.sqrt(max(cov[0], 0.0)),
                   sigma_theta=math.sqrt(max(cov[5], 0.0)))
    if kind == "kf":
        p, cov = msg.pose.pose, cov_diag(msg.pose.covariance)
        return Kf(_stamp(msg.header), p.position.x, p.position.y, quat_to_yaw(p.orientation),
                  math.sqrt(max(cov[0], 0.0)), math.sqrt(max(cov[1], 0.0)),
                  math.sqrt(max(cov[5], 0.0)))
    if kind == "imu":
        g, a = msg.angular_velocity, msg.linear_acceleration
        return Imu(_stamp(msg.header), a.x, a.y, a.z, g.x, g.y, g.z)
    if kind == "scan":
        return Scan(_stamp(msg.header), msg.angle_min, msg.angle_increment, msg.range_min,
                    msg.range_max, list(msg.ranges))
    raise ValueError(f"unknown subscription type {kind}")


def _stamp(header) -> float:
    """Message stamp in seconds (simulation time when use_sim_time is set)."""
    return header.stamp.sec + header.stamp.nanosec * 1e-9


def spawn_handler(engine) -> callable:
    """Handler for spawn_robot: a SpawnError becomes a message, never an exception.

    The `message` is a sentence rather than `ok`, because on `/sim/spawn_next` (a Trigger, §4) it is
    the only channel there is: the caller needs the name that was chosen and where the robot stands.
    """
    def handle(req: dict) -> dict:
        try:
            r = engine.spawn(str(req.get("name", "")), str(req.get("variant", "")))
            return {"success": True,
                    "message": f"spawned \'{r.spec.name}\' ({r.spec.variant}, {r.spec.color}) at "
                               f"({r.pose.x:.2f}, {r.pose.y:.2f}, "
                               f"{math.degrees(r.pose.theta):.0f} deg)",
                    "name": r.spec.name, "index": r.spec.index,
                    "color": r.spec.color, "marker": r.spec.marker, "variant": r.spec.variant,
                    "x": r.pose.x, "y": r.pose.y, "theta": r.pose.theta}
        except Exception as exc:
            return {"success": False, "message": str(exc)}
    return handle


def despawn_handler(engine, last: bool = False) -> callable:
    """Remove one robot; with `last`, an empty request means the robot that joined last."""
    def handle(req: dict) -> dict:
        name = str(req.get("name", ""))
        if not name and last:
            robot = engine.last_spawn()
            if robot is None:
                return {"success": False, "message": "no robot in this run"}
            name = robot.spec.name
        ok = engine.despawn(name)
        return {"success": ok, "message": f"'{name}' removed" if ok else f"'{name}' not found"}
    return handle


# ---------------------------------------------------------------------- the real ROS 2 bus


class RclpyBus:
    """Same interface as `stub.StubBus` (pub/publish/sub/sub_topic/last/service/call/spin/ok)."""

    def __init__(self, node_name: str = "mecanum_sim", M: dict | None = None,
                 cfg: dict | None = None):
        import rclpy
        from rclpy.node import Node
        self.rclpy, self.M = rclpy, M or load_msgs()
        if self.M is None:
            raise RuntimeError("rclpy is there but the message classes are not — source ROS correctly?")
        if not rclpy.ok():
            rclpy.init()
        self.node = Node(node_name)
        self.cfg = cfg or {}
        self.name, self._pubs, self._last, self._srv, self._cli = node_name, {}, {}, {}, {}
        self._subscribed, self._iface = set(), _spawn_iface(self.M)
        log.info("ROS node '%s' (%s)", node_name, "SpawnRobot interface" if self._iface
                 else "spawn fallback: JSON handshake")

    def enable_tf(self, engine, cfg: dict | None = None) -> None:
        """/tf and /tf_static: map -> <robot>/odom -> <robot>/base_link -> laser, imu_link.

        Only RViz and the visualisation tools need this; the engine and every topic of the
        contract stay exactly as they are.
        """
        self.cfg = cfg or self.cfg or getattr(engine, "cfg", {}) or {}
        tf_bcast.attach(self.node, engine, self.cfg, self.M)

    # ---------------------------------------------------------------- publisher side

    def pub(self, kind: str, robot: str | None = None):
        key = (kind, robot)
        if key not in self._pubs:
            self._pubs[key] = self.node.create_publisher(self.M[KIND_MSG[kind]],
                                                         topic(kind, robot), 10)
        p, M = self._pubs[key], self.M

        if kind == "gps":                               # every `/gps` also appears on `/gps_cov`
            cov = self.pub("gpscov", robot)             # with the σ the emission was drawn with
        else:
            cov = None

        def send(payload):
            p.publish(to_ros(M, kind, payload, robot, self.cfg))
            if cov is not None:
                cov(payload)
        return send

    def publish(self, name: str, payload) -> None:
        """Topic name instead of kind — only the special cases need it (handshake reply)."""
        self.pub_string(name, str(payload))

    # ---------------------------------------------------------------- subscriber side

    def sub(self, kind: str, robot: str | None, cb) -> None:
        """Subscription with the real message type; `cb` gets the dataclass from types.py again.

        A GPS fix is read on `/gps_cov` and not on `/gps`: the `PoseStamped` on `/gps` has no place for
        the σ of its emission, and `rob.gps()` must answer the same `types.Gps` — σ included — whether
        or not ROS is sourced. `/gps` itself is still published exactly as it was, so a solution that
        subscribes to it by name is unaffected.
        """
        gelesen = READS.get(kind, kind)
        self._subscribe(KIND_MSG[gelesen], topic(gelesen, robot),
                        lambda msg, k=kind, g=gelesen, c=cb:
                        self._recv(topic(k, robot), c, from_ros(g, msg)))

    def sub_topic(self, name: str, cb) -> None:
        """Subscription to a string topic; `cb` gets the plain text (JSON or task name)."""
        self._subscribe("String", name, lambda msg, n=name, c=cb: self._recv(n, c, msg.data))

    def _subscribe(self, cls_key: str, name: str, wrap) -> None:
        self._subscribed.add(name)
        self.node.create_subscription(self.M[cls_key], name, wrap, 10)

    def _recv(self, name, cb, value) -> None:
        self._last[name] = (value, time.monotonic())
        try:
            cb(value)
        except Exception:
            log.exception("subscriber on %s", name)

    def last(self, kind: str, robot: str | None = None) -> tuple:
        """Last value + age in s. The first look subscribes to the topic on the side —
        otherwise `last()` would stay empty forever in ROS, since the reader never subscribed."""
        name = topic(kind, robot)
        if name not in self._subscribed:
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
        """Service call (typed) or handshake — call directly while nothing else is spinning."""
        if name in self._srv:                               # same process: call directly
            return self._srv[name](dict(req or {}))
        if self._iface:
            client = self._cli.get(name) or self.node.create_client(self._iface, name)
            self._cli[name] = client
            if not client.wait_for_service(timeout_sec=1.0):
                return {"success": False, "message": f"service {name} not reachable"}
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
        return got or {"success": False, "message": f"no answer from {name}"}

    def _await(self, future, decode, timeout):
        ende = time.monotonic() + timeout
        while not future.done() and time.monotonic() < ende and self.rclpy.ok():
            self.spin(0.02)
        if not future.done():
            return {"success": False, "message": "timed out"}
        return decode(future.result())

    # ------------------------------------------------------------------ lifecycle

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
    """The small own interface, if it was built — otherwise None (then JSON handshake)."""
    try:
        from mecanum_lab_interfaces.srv import SpawnRobot
        return SpawnRobot
    except ImportError:
        return None


def make_bus(kind: str = "auto", node_name: str = "mecanum_sim", cfg: dict | None = None):
    """"auto": real ROS when available, otherwise None (= caller takes stub). None is a
    result here, not an error: that is what keeps `./lab run` installable without ROS."""
    if kind == "stub":
        return stub.get_bus()
    if load_msgs() is None:
        if kind == "ros":
            raise RuntimeError("no ROS 2 (rclpy) found — please source /opt/ros/$ROS_DISTRO/setup.bash")
        return None
    try:
        return RclpyBus(node_name, cfg=cfg)
    except Exception as exc:
        if kind == "ros":
            raise
        log.warning("ROS not usable (%s) — using in-process bus.", exc)
        return None
