"""Tests for how a missing lidar echo is put into a `LaserScan`.

Two dialects, one default: the simulator has always reported a beam that did not come back as the laser's
own `range_max`, and the laboratories measure that. `sensor_msgs/msg/LaserScan` documents infinity for it,
which is the only thing a mapper can read as "open beyond". The switch is `sensor.lidar.no_echo`.
"""
import math
from types import SimpleNamespace

from mecanum_lab import ros_bridge
from mecanum_lab.types import Scan


class Header:
    def __init__(self, frame_id=""):
        self.frame_id, self.stamp = frame_id, None


class Attr:
    """Message dummy that grows any attribute on first use — enough for a conversion test."""

    def __getattr__(self, name):
        value = Attr()
        self.__dict__[name] = value
        return value


M = {"Header": Header, "Time": lambda sec=0, nanosec=0: SimpleNamespace(sec=sec, nanosec=nanosec),
     "LaserScan": Attr}

CFG = {"tf": {"namespaces": True}}

ECHOES = [2.0, math.inf, float("nan"), 8.0, 12.0]      # measured, missing, broken, at the limit, beyond


def dialect(ranges):
    """`open` for a beam that is not a distance any more, the number for one that is."""
    return ["open" if math.isinf(r) else r for r in ranges]


def scan():
    return Scan(t=0.0, angle_min=0.0, angle_increment=0.1, range_min=0.05, range_max=8.0, ranges=ECHOES)


def test_a_missing_echo_is_the_laser_range_as_long_as_that_is_what_the_laboratories_measure():
    assert dialect(ros_bridge.to_ros(M, "scan", scan(), "alice", CFG).ranges) \
        == [2.0, 8.0, 8.0, 8.0, 8.0]


def test_the_other_dialect_leaves_a_missing_echo_as_infinity():
    """A beam of exactly 8.0 m is a measurement at the limit, not a missing one, and stays a number."""
    cfg = {**CFG, "sensor": {"lidar": {"no_echo": "inf"}}}
    assert dialect(ros_bridge.to_ros(M, "scan", scan(), "alice", cfg).ranges) \
        == [2.0, "open", "open", 8.0, "open"]
