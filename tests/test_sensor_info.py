"""`/<robot>/sensor/info`: the five numbers about the instruments, on a topic of their own.

`quality`, `sats` and the lost count have no field in `geometry_msgs/msg/PoseStamped`, the chip
temperature has none in `sensor_msgs/msg/Imu`, and a beam that came back as nothing is
`range_max` in `LaserScan` (CONTRACT §6.4). So over ROS the state of the instruments used to exist
only in the window and in the log — which is the one place a second terminal cannot reach. These
tests pin that the message says the same numbers the readout line says, at the rate of the slowest
instrument, and that over ROS it rides a `std_msgs/msg/String` like `/poi` and `/link` do.
"""
import math
from types import SimpleNamespace

import pytest

from mecanum_lab import overlays, ros_bridge
from mecanum_lab.engine import SimEngine
from mecanum_lab.logbook import COLUMNS
from mecanum_lab.types import Twist, load_config, topic
from mecanum_lab.worlds import load_world

STEP = 0.02


def driven(seconds=6.0, cfg=None, seed=3, kinds=("sensorinfo",)):
    """One straight-ish drive with every delivered message of `kinds`, in the order they arrived."""
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=seed)
    eng.spawn("muster")
    got = []
    for i in range(int(seconds / STEP)):
        eng.set_cmd_vel("muster", Twist(vx=0.4, omega=0.1 * (1 if i % 100 < 50 else -1)))
        eng.step(eng.sub_step)
        got += [(eng.t, kind, robot, payload) for kind, robot, payload in eng.drain()
                if kind in kinds]
    return eng, got


def before(got, kind, index):
    """The last message of `kind` that had already arrived when `got[index]` went out."""
    return next(payload for t, k, robot, payload in reversed(got[:index]) if k == kind)


def test_the_answer_comes_from_the_instruments_not_from_their_last_message():
    """Quality, satellites, losses, latency, temperature, gaps — each from the thing that knows it.

    The scan is compared against the scan that had arrived when the message went out, not against
    whatever the robot measured since: `/sensor/info` is a snapshot at its own `t`, and the LIDAR is
    four times faster than it.
    """
    eng, got = driven(kinds=("sensorinfo", "scan"))
    assert got, "the topic is published without any demo config too"
    index = max(range(len(got)), key=lambda i: got[i][0] if got[i][1] == "sensorinfo" else -1)
    t, kind, robot, info = got[index]
    quality, sats, lost = eng.gps_health(robot)
    assert (info.quality, info.sats, info.lost) == (quality, sats, lost)
    assert info.temp == pytest.approx(eng.robots[robot].inertial.temp)
    assert info.scan_gaps == before(got, "scan", index).missing
    assert info.latency_ms == pytest.approx(1000.0 * eng._gps.latency)


def test_it_is_published_at_the_rate_of_the_slowest_instrument():
    """`gps.rate`, because nothing in the message changes faster — and the log is 10 Hz anyway."""
    cfg = load_config(None, {"gui": False, "gps": {"rate": 5.0}})
    eng, got = driven(seconds=6.0, cfg=cfg)
    assert 25 <= len(got) <= 35, f"5 Hz for 6 s, measured {len(got)}"


def test_echo_shows_what_the_readout_line_shows():
    """The point of the message: one `ros2 topic echo` and the window's sentence are the same facts."""
    eng, got = driven(seconds=4.0)
    info = got[-1][3]
    robot = eng.robots["muster"]
    rend = SimpleNamespace(engine=eng, s=50.0)                 # what _gps_view asks of the renderer
    line = " ".join(text for text, _colour in overlays.sensor_readout(rend, robot))
    assert f"q{info.quality} {info.sats} sats" in line
    assert f"{info.temp:.1f} °C" in line
    assert info.scan_gaps == 0 or f"lidar {info.scan_gaps} beams" in line


def test_the_same_five_numbers_are_log_columns():
    """Topic and log are the same facts seen during and after the run (CONTRACT §6.4, §6.10)."""
    for column in ("q_gps", "sats_gps", "lost_gps", "temp_imu", "noecho_scan"):
        assert column in COLUMNS


@pytest.mark.skipif(ros_bridge.load_msgs() is None, reason="no ROS 2 in this shell")
def test_over_ros_it_is_one_json_object_on_a_string():
    """Round trip: the dataclass out, the string on the wire, the dataclass back."""
    from mecanum_lab.types import SensorInfo
    info = SensorInfo(t=1.5, quality=1, sats=4, lost=2, latency_ms=35.0, temp=27.5, scan_gaps=7)
    msg = ros_bridge.to_ros(ros_bridge.load_msgs(), "sensorinfo", info, "alice", load_config())
    back = ros_bridge.from_ros("sensorinfo", msg)
    assert back == info
    assert "sensor/info" in topic("sensorinfo", "alice")
