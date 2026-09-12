"""Minimal TF tree of the simulator — plain `tf2_msgs/msg/TFMessage`, no tf2_ros needed.

    map                      the world of worlds/<name>.txt, origin bottom left
     └─ <robot>/odom         every tf.rate s; identity unless tf.map_to_odom: truth
         └─ <robot>/base_link    the odometry the students actually get on /<robot>/odom
             ├─ <robot>/laser        static, offset in tf.mount
             └─ <robot>/imu_link     static, offset in tf.mount

Two decisions are worth knowing before changing anything here:

* **Why the robot name in front of the frame.** TF names are global in a ROS graph, and in
  the lab 25 robots drive in the same hall. Without the prefix would every robot claim the
  frame `odom` for itself and RViz could no longer assign a scan to a robot. Topic names and
  `/sim/robots` stay as they are — only the headers gain the prefix.
* **Why `map -> odom` is the identity by default.** Then `map -> base_link` is exactly the
  drifting odometry, which is the point of the Kalman lab: the students close that gap with
  GPS and IMU themselves instead of getting the answer from the simulator. The tutor view of
  the exact pose is one setting away (`--set tf.map_to_odom=truth`) and does not replace
  `/<robot>/truth`.

RViz works with this: Fixed Frame `map`, the rest follows from /tf and /tf_static.
"""
import math

from .types import cfg_get, wrap_angle

# message kind -> which of the robot frames its header carries
KIND_FRAME = {"odom": "odom", "scan": "laser", "imu": "imu", "gps": "map", "truth": "map",
              "kf": "map"}


def frames(name: str, cfg: dict | None = None) -> dict:
    """Frame names of one robot, with the robot as prefix unless tf.namespaces is false."""
    pr = f"{name}/" if name and cfg_get(cfg or {}, "tf.namespaces", True) else ""
    return {"map": "map", "odom": pr + "odom", "base": pr + "base_link",
            "laser": pr + "laser", "imu": pr + "imu_link"}


def frame_fuer(kind: str, name: str = "", cfg: dict | None = None) -> str:
    """Header frame of a message kind — the one place where headers get their names."""
    return frames(name, cfg)[KIND_FRAME[kind]]


def header_frames(kind: str, name: str = "", cfg: dict | None = None) -> tuple:
    """(header frame, child frame id) as ros_bridge needs it for Odometry."""
    f = frames(name, cfg)
    return (f["odom"], f["base"]) if kind == "odom" else (frame_fuer(kind, name, cfg), None)


# --------------------------------------------------------------------- Baum aus Posen bauen


def _kopf(M, t: float, frame: str):
    kopf = M["Header"](frame_id=frame)
    kopf.stamp = M["Time"](sec=int(t), nanosec=int((t % 1.0) * 1e9))
    return kopf


def _trans(M, t: float, parent: str, child: str, x: float, y: float, yaw: float, z: float = 0.0):
    """One TransformStamped; rotation is yaw only, the sim is a 2D world."""
    from .ros_bridge import yaw_to_quat                     # late: ros_bridge imports this here
    tf = M["TransformStamped"]()
    tf.header = _kopf(M, t, parent)
    tf.child_frame_id = child
    tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = \
        float(x), float(y), float(z)
    qx, qy, qz, qw = yaw_to_quat(yaw)
    tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
    tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
    return tf


def static_tree(M, engine, cfg: dict, t: float) -> list:
    """base_link -> laser and base_link -> imu_link, mounted as tf.mount says."""
    haltungen = cfg_get(cfg, "tf.mount") or {}
    sending = []
    for robot in engine.robots.values():
        f = frames(robot.spec.name, cfg)
        for kind, achse in (("laser", "laser"), ("imu", "imu")):
            x, y, z = (list(haltungen.get(achse) or [0.0, 0.0, 0.0]) + [0.0, 0.0, 0.0])[:3]
            sending.append(_trans(M, t, f["base"], f[kind], x, y, 0.0, z))
    return sending


def dynamic_tree(M, engine, cfg: dict, t: float) -> list:
    """map -> odom (see module docstring) and odom -> base_link from the odometry."""
    wahrheit = str(cfg_get(cfg, "tf.map_to_odom", "odom")).lower() == "truth"
    sending = []
    for robot in engine.robots.values():
        f = frames(robot.spec.name, cfg)
        odo = robot.odom or robot.pose                        # at spawn both are identical
        sending.append(_trans(M, t, f["odom"], f["base"], odo.x, odo.y, odo.theta))
        x, y, yaw = 0.0, 0.0, 0.0
        if wahrheit:                                          # map->odom closes the drift
            wahr, theta = robot.pose, wrap_angle(robot.pose.theta - odo.theta)
            cos, sin = math.cos(theta), math.sin(theta)
            x = wahr.x - (cos * odo.x - sin * odo.y)
            y = wahr.y - (sin * odo.x + cos * odo.y)
            yaw = theta
        sending.append(_trans(M, t, "map", f["odom"], x, y, yaw))
    return sending


def attach(node, engine, cfg: dict, M) -> None:
    """Publish /tf and /tf_static from this node; the engine and its topics stay untouched."""
    if not cfg_get(cfg, "tf.enabled", True) or M.get("TFMessage") is None:
        return
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    dynamik = node.create_publisher(M["TFMessage"], "tf", 10)
    statik = node.create_publisher(
        M["TFMessage"], "tf_static",
        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
    gesehen, meldung = set(), M["TFMessage"]

    def tick():
        zeit = float(engine.t)
        if set(engine.robots) != gesehen:                     # new robot -> new static branches
            gesehen.clear()
            gesehen.update(engine.robots)
            statik.publish(meldung(transforms=static_tree(M, engine, cfg, zeit)))
        dynamik.publish(meldung(transforms=dynamic_tree(M, engine, cfg, zeit)))

    tick()                                                    # static transforms without delay
    node.create_timer(1.0 / float(cfg_get(cfg, "tf.rate", 20.0) or 20.0), tick)
