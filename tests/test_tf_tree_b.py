"""Tests for the TF tree of the simulator — pure functions, no rclpy, no display.

What has to hold here is what RViz cannot tell you: the frames in /tf must be exactly the
frames in the message headers, the tree must be connected (map -> odom -> base_link ->
laser), and the default must not give the ground truth away to the Kalman lab.
"""
from types import SimpleNamespace

import pytest

from mecanum_lab import ros_bridge, tf_bcast
from mecanum_lab.types import Odom, Pose


# ------------------------------------------------------------------ fake message classes


class V:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Header:
    def __init__(self, frame_id=""):
        self.frame_id, self.stamp = frame_id, None


class TSt:
    def __init__(self):
        self.header, self.child_frame_id, self.transform = Header(), "", SimpleNamespace(
            translation=V(), rotation=V(0.0, 0.0, 1.0))


class TFMsg:
    def __init__(self, transforms=()):
        self.transforms = list(transforms)


class Attr:
    """Message dummy that grows any attribute on first use — enough for a conversion test."""

    def __getattr__(self, name):
        wert = Attr()
        self.__dict__[name] = wert
        return wert


M = {"Header": Header, "Time": lambda sec=0, nanosec=0: SimpleNamespace(sec=sec, nanosec=nanosec),
     "Vector3": V, "Quaternion": V, "TransformStamped": TSt, "TFMessage": TFMsg,
     "LaserScan": Attr, "Odometry": Attr, "PoseStamped": Attr, "Imu": Attr,
     "PoseWithCovarianceStamped": Attr,
     "String": lambda data="": SimpleNamespace(data=data)}

CFG = {"tf": {"enabled": True, "rate": 20.0, "map_to_odom": "odom", "namespaces": True,
              "mount": {"laser": [0.1, 0.0, 0.05], "imu": [0.0, 0.0, 0.2]}}}


def bot(name="alice", pose=(1.0, 2.0, 0.5), odom=(0.9, 2.1, 0.45)):
    return SimpleNamespace(spec=SimpleNamespace(name=name),
                           pose=Pose(*pose), odom=Odom(1.0, *odom, 0.0, 0.0, 0.0))


def engine(*robots):
    return SimpleNamespace(robots={r.spec.name: r for r in robots})


# ------------------------------------------------------------------------- frame names


def test_frames_carry_the_robot_name():
    f = tf_bcast.frames("alice", CFG)
    assert f["map"] == "map" and f["odom"] == "alice/odom"
    assert f["base"] == "alice/base_link" and f["laser"] == "alice/laser"
    assert f["imu"] == "alice/imu_link"


def test_frames_without_prefix_when_namespaces_are_off():
    f = tf_bcast.frames("alice", {"tf": {"namespaces": False}})
    assert (f["odom"], f["base"], f["laser"]) == ("odom", "base_link", "laser")


@pytest.mark.parametrize("kind,erwartet", [("scan", "alice/laser"), ("imu", "alice/imu_link"),
                                           ("odom", "alice/odom"), ("gps", "map"),
                                           ("truth", "map"), ("kf", "map")])
def test_header_frames_match_the_tree(kind, erwartet):
    assert tf_bcast.frame_fuer(kind, "alice", CFG) == erwartet


def test_odometry_reports_parent_and_child():
    assert tf_bcast.header_frames("odom", "alice", CFG) == ("alice/odom", "alice/base_link")


def test_every_kind_in_the_contract_has_a_frame():
    for kind in ("odom", "scan", "gps", "truth", "imu", "kf"):
        assert tf_bcast.frame_fuer(kind, "bob", CFG)          # KeyError here means: no frame
        assert tf_bcast.frame_fuer(kind, "bob", CFG).endswith(tf_bcast.frame_fuer(kind, "", CFG))


# ------------------------------------------------------------------------- the tree itself


def test_static_tree_mounts_laser_and_imu_on_base_link():
    sending = tf_bcast.static_tree(M, engine(bot()), CFG, 3.0)
    paare = {(t.header.frame_id, t.child_frame_id) for t in sending}
    assert paare == {("alice/base_link", "alice/laser"), ("alice/base_link", "alice/imu_link")}
    laser = next(t for t in sending if t.child_frame_id.endswith("laser"))
    assert (laser.transform.translation.x, laser.transform.translation.z) == (0.1, 0.05)


def test_static_tree_grows_with_every_robot():
    sending = tf_bcast.static_tree(M, engine(bot("alice"), bot("bob")), CFG, 1.0)
    kinder = {t.child_frame_id for t in sending}
    assert {"alice/laser", "bob/laser", "bob/imu_link"} <= kinder


def test_dynamic_tree_publishes_map_and_odometry():
    sending = tf_bcast.dynamic_tree(M, engine(bot()), CFG, 5.0)
    paare = {(t.header.frame_id, t.child_frame_id) for t in sending}
    assert paare == {("map", "alice/odom"), ("alice/odom", "alice/base_link")}
    odom_tf = next(t for t in sending if t.child_frame_id == "alice/base_link")
    assert (odom_tf.transform.translation.x, odom_tf.transform.translation.z) == (0.9, 0.0)
    assert odom_tf.header.stamp.sec == 5


def test_map_to_odom_is_the_identity_by_default():
    """map -> base_link has to be the drifting odometry, not the truth. Lab 2 is their job."""
    sending = tf_bcast.dynamic_tree(M, engine(bot()), CFG, 1.0)
    karte = next(t for t in sending if t.child_frame_id == "alice/odom")
    assert (karte.transform.translation.x, karte.transform.translation.y) == (0.0, 0.0)
    assert karte.transform.rotation.w == pytest.approx(1.0)


def test_truth_mode_moves_the_odometry_frame():
    """With tf.map_to_odom=truth the composed map->base_link equals the exact pose."""
    cfg = {"tf": {"map_to_odom": "truth", "namespaces": True}}
    robot = bot(pose=(3.0, 1.0, 1.0), odom=(2.0, 0.5, 0.4))
    sending = tf_bcast.dynamic_tree(M, engine(robot), cfg, 2.0)
    karte = next(t for t in sending if t.child_frame_id == "alice/odom")
    basis = next(t for t in sending if t.child_frame_id == "alice/base_link")
    import math
    dreh = 2 * math.acos(min(1.0, karte.transform.rotation.w))
    yaw = dreh if karte.transform.rotation.z >= 0 else -dreh
    x = karte.transform.translation.x + (math.cos(yaw) * basis.transform.translation.x
                                         - math.sin(yaw) * basis.transform.translation.y)
    y = karte.transform.translation.y + (math.sin(yaw) * basis.transform.translation.x
                                         + math.cos(yaw) * basis.transform.translation.y)
    assert x == pytest.approx(3.0, abs=1e-9) and y == pytest.approx(1.0, abs=1e-9)


def test_missing_odometry_falls_back_to_the_true_pose():
    robot = bot()
    robot.odom = None
    sending = tf_bcast.dynamic_tree(M, engine(robot), CFG, 0.5)
    basis = next(t for t in sending if t.child_frame_id == "alice/base_link")
    assert basis.transform.translation.x == pytest.approx(1.0)


def test_two_robots_never_share_a_frame():
    sending = (tf_bcast.static_tree(M, engine(bot("alice"), bot("bob")), CFG, 1.0)
               + tf_bcast.dynamic_tree(M, engine(bot("alice"), bot("bob")), CFG, 1.0))
    kinder = [t.child_frame_id for t in sending]
    assert len(kinder) == len(set(kinder)), "every TF child must be unique in the graph"
    assert sum(1 for k in kinder if k.endswith("laser")) == 2


# ------------------------------------------------------------- headers and messages agree


def test_message_headers_use_the_same_frames_as_the_tree():
    from mecanum_lab.types import Scan
    robot = bot()
    scan = Scan(t=1.0, angle_min=0.0, angle_increment=0.1, range_min=0.05, range_max=8.0,
                ranges=[1.0] * 10)
    msg = ros_bridge.to_ros(M, "scan", scan, "alice", CFG)
    assert msg.header.frame_id == "alice/laser"
    odom_msg = ros_bridge.to_ros(M, "odom", robot.odom, "alice", CFG)
    assert (odom_msg.header.frame_id, odom_msg.child_frame_id) == ("alice/odom", "alice/base_link")
    gps_msg = ros_bridge.to_ros(M, "gps", robot.pose, "alice", CFG)
    assert gps_msg.header.frame_id == "map"


def test_frames_of_messages_without_a_prefix_stay_as_they_were():
    from mecanum_lab.types import Scan
    scan = Scan(t=0.0, angle_min=0.0, angle_increment=0.1, range_min=0.05, range_max=8.0,
                ranges=[1.0] * 4)
    assert ros_bridge.to_ros(M, "scan", scan).header.frame_id == "laser"
