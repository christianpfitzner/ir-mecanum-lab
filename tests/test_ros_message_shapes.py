"""A covariance the bridge writes must be as long as the message field declares it to be.

The story: `to_ros()` filled the three covariance fields of `sensor_msgs/msg/Imu` with 36 numbers,
while that message declares `float64[9]` for each of them — an attitude, a body rate and an
acceleration have three axes, six belongs to a pose. On ROS 2 Humble the generated setter counts the
entries and answers a 36-field list with an `AssertionError` *inside the publisher*, so
`ros2 launch mecanum_lab lab.launch.py` died with the simulator three seconds into its first IMU
message. Two things let that through the gate: the ROS steps of `tools/check.sh` echoed `/odom` and
`/scan` and never `/imu`, and the rclpy of a recent release copies such a list without counting —
so the same 36 is green on the development machine and fatal in the lab room.

Hence the length is pinned twice: once as plain numbers a bare `pytest` run checks without any ROS,
and once against the message classes of whatever ROS this shell happens to have sourced. The second
check asks a **fresh** message how long its fields are and never reads back what was assigned —
reading back is exactly the check that passes on the permissive rclpy.
"""
import math
import re
from types import SimpleNamespace

import pytest

from mecanum_lab import ros_bridge
from mecanum_lab.types import Gps, Imu, Kf, Link, Odom, Poi, Scan, SensorInfo, Twist

CFG = {"tf": {"namespaces": True}}

# What the definitions in the message packages say, as numbers: a pose and a twist covariance are
# 6x6, the three covariances of an Imu are 3x3 (see the docstring of ros_bridge.cov9).
DECLARED = {("odom", "pose.covariance"): 36, ("odom", "twist.covariance"): 36,
            ("gpscov", "pose.covariance"): 36, ("kf", "pose.covariance"): 36,
            ("imu", "orientation_covariance"): 9,
            ("imu", "angular_velocity_covariance"): 9,
            ("imu", "linear_acceleration_covariance"): 9}


# ------------------------------------------------------------------ message dummies (no rclpy)


class Attr:
    """Message dummy that grows any attribute on first use (same trick as tests/test_tf_tree_b.py)."""

    def __getattr__(self, name):
        value = Attr()
        self.__dict__[name] = value
        return value


M = {"Header": lambda frame_id="": SimpleNamespace(frame_id=frame_id, stamp=None),
     "Time": lambda sec=0, nanosec=0: SimpleNamespace(sec=sec, nanosec=nanosec),
     "Imu": Attr, "Odometry": Attr, "PoseStamped": Attr, "PoseWithCovarianceStamped": Attr,
     "LaserScan": Attr, "Twist": Attr, "Vector3": Attr, "Quaternion": Attr, "String": Attr}

FIX = {"imu": Imu(t=1.0, ax=0.1, ay=0.0, az=9.81, gz=0.2),
       "odom": Odom(t=1.0, x=1.0, y=2.0, theta=0.5, vx=0.5, vy=0.0, omega=0.1),
       "gpscov": Gps(t=1.0, x=1.0, y=2.0, theta=0.5, sigma_xy=0.3, sigma_theta=0.05),
       "kf": Kf(t=1.0, x=1.0, y=2.0, theta=0.5, sx=0.2, sy=0.3, sth=0.04)}


def follow(msg, dotted: str):
    """`"pose.covariance"` out of a message, without every test writing its own walk."""
    for step in dotted.split("."):
        msg = getattr(msg, step)
    return msg


def diagonal(cov):
    n = int(round(math.sqrt(len(cov))))
    return [float(cov[i * (n + 1)]) for i in range(n)]


def off_diagonal(cov):
    n = int(round(math.sqrt(len(cov))))
    return [float(v) for i, v in enumerate(cov) if i // n != i % n]


# ---------------------------------------------------------------------- the pure conversions


def test_the_diagonal_lands_where_row_major_expects_it_and_in_the_length_asked_for():
    assert len(ros_bridge.cov36((1.0,) * 6)) == 36
    assert diagonal(ros_bridge.cov36((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))) == \
        pytest.approx([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    nine = ros_bridge.cov9(1.0e-4)
    assert len(nine) == 9, "an Imu covariance is three by three, see the docstring of cov9"
    assert diagonal(nine) == pytest.approx([1.0e-4] * 3)
    assert off_diagonal(nine) == pytest.approx([0.0] * 6)


@pytest.mark.parametrize("kind,field,want",
                         [(k, f, w) for (k, f), w in sorted(DECLARED.items())])
def test_the_covariance_of_every_message_has_the_length_that_message_declares(kind, field, want):
    got = follow(ros_bridge.to_ros(M, kind, FIX[kind], "alice", CFG), field)
    assert len(got) == want, f"{kind} {field}: {len(got)} entries, the message holds {want}"


def test_the_imu_fills_its_three_covariances_and_nothing_else():
    """Placeholder numbers so RViz and rqt do not divide by zero — on the diagonal of each 3x3."""
    msg = ros_bridge.to_ros(M, "imu", FIX["imu"], "alice", CFG)
    for field, key in (("orientation_covariance", "orientation"),
                       ("angular_velocity_covariance", "angular"),
                       ("linear_acceleration_covariance", "linear")):
        cov = getattr(msg, field)
        assert diagonal(cov) == pytest.approx([ros_bridge.IMU_COV[key]] * 3), field
        assert off_diagonal(cov) == pytest.approx([0.0] * 6), field
    assert (msg.angular_velocity.z, msg.linear_acceleration.z) == (0.2, 9.81)


def test_the_pose_covariances_are_still_the_36_they_have_always_been():
    """The sigma of one emission, squared, on the diagonal of a 6x6 (CONTRACT-KF section 8.2)."""
    gps = ros_bridge.to_ros(M, "gpscov", FIX["gpscov"], "alice", CFG)
    pose = follow(gps, "pose.covariance")
    assert diagonal(pose)[0] == pytest.approx(0.3 ** 2)                  # x
    assert diagonal(pose)[1] == pytest.approx(0.3 ** 2)                  # y
    assert diagonal(pose)[5] == pytest.approx(0.05 ** 2)                 # yaw
    assert diagonal(ros_bridge.to_ros(M, "kf", FIX["kf"], "alice", CFG)
                   .pose.covariance)[:2] == pytest.approx([0.2 ** 2, 0.3 ** 2])
    odom = ros_bridge.to_ros(M, "odom", FIX["odom"], "alice", CFG)
    assert odom.pose.covariance == ros_bridge.cov36((0.02, 0.02, 1e6, 1e6, 1e6, 0.05))


# --------------------------------------------------- the same claim against a real ROS 2 install

# One payload per kind the bridge can put on a topic; the values do not matter, the types do.
PAYLOADS = {"twist": Twist(0.5, 0.0, 0.1), "wheels": [1.0, 2.0, 3.0, 4.0],
            "odom": FIX["odom"], "scan": Scan(t=1.0, angle_min=0.0, angle_increment=0.1,
                                              range_min=0.05, range_max=8.0, ranges=[2.0] * 5),
            "gps": FIX["gpscov"], "gpscov": FIX["gpscov"], "truth": FIX["gpscov"],
            "imu": FIX["imu"], "kf": FIX["kf"], "kfinfo": "nees 1.2", "mission": "v1",
            "robots": "{}", "world": "{}", "task": "t", "config": "{}", "clock": 1.5,
            "poi": Poi(t=1.0, intensity=3.5),
            "link": Link(t=1.0, quality=0.9, rssi_dbm=-61.0, ap=(1.0, 2.0)),
            "sensorinfo": SensorInfo(t=1.0, quality=1, sats=5, lost=2, latency_ms=30.0,
                                     temp=27.0, scan_gaps=1)}

FIXED_ARRAY = re.compile(r"^[a-z0-9_/]+\[(\d+)]$")


def shapes(msg, fresh, path=""):
    """`(path, entries on the wire, entries declared)` for every fixed-size field of `msg`."""
    out = []
    for field, declared in fresh.get_fields_and_field_types().items():
        fixed = FIXED_ARRAY.match(declared)
        if fixed:
            out.append((path + field, len(getattr(msg, field)), int(fixed.group(1))))
        elif "/" in declared and not declared.startswith("sequence<"):
            sub = getattr(fresh, field)
            if hasattr(type(sub), "get_fields_and_field_types"):      # a nested message, one level in
                out += shapes(getattr(msg, field), sub, path + field + ".")
    return out


@pytest.mark.skipif(ros_bridge.load_msgs() is None, reason="no ROS 2 in this shell")
def test_every_fixed_size_field_holds_what_the_sourced_message_declares():
    """Checked from the message classes, so no future field shape can quietly outgrow the bridge.

    On a permissive rclpy the wrong length is accepted on the way in and comes back unchanged, which
    is how 36 entries for a `double[9]` reached a Humble laboratory from a machine whose own ROS
    found nothing to complain about. Comparing against a freshly built message of the same type is
    the one comparison that answers the same question on every ROS 2.
    """
    M_real = ros_bridge.load_msgs()
    checked = 0
    for kind, payload in sorted(PAYLOADS.items()):
        msg = ros_bridge.to_ros(M_real, kind, payload, "alice", CFG)
        fresh = M_real[ros_bridge.KIND_MSG[kind]]()
        for path, got, want in shapes(msg, fresh):
            assert got == want, f"/{kind} -> {path}: {got} entries, {want} declared"
            checked += 1
    assert checked >= len(DECLARED), (
        f"only {checked} fixed-size fields seen — the sweep stopped checking, not the bridge")
