"""The header of the students' own estimate: which moment it names, and in which frame.

Neither field is visible from a grade, and both decide whether a working filter can be seen at all. The
stamp used to be a hard-wired `Kf(t=0.0, …)` in `robot_io.send_kf`, which on the ROS bus is a pose at the
epoch: RViz draws a `PoseWithCovariance` through TF, a TF lookup at 0.0 is out of the cache from the first
second of a run on, and the display's message filter drops the stream — `Message Filter dropping message:
frame 'map' at time 0.000 … because the queue is full`, an empty estimate panel, and a group that believes
its filter is broken while it publishes at 30 Hz. The frame used to be guessed: a controller is a *second*
process with its own config file, so under `tf.tree: slam` — where the simulator's ground frame is `hall` —
its `kf/pose` arrived stamped `map`, a frame no transform of that run ever reaches. Grading never noticed,
because the grader measures the numbers and counts arrivals (`engine.set_kf` stamps the arrival into `Kf.t`
for the HUD), so both defects survived on the only topic of this experiment that a student writes.

Checked without rclpy on the message dummies of `tests/test_ros_message_shapes.py`'s pattern, plus the stub
bus: what is asserted here is the content of two header fields, not the DDS underneath them.
"""
import json
from types import SimpleNamespace

import pytest

from mecanum_lab import ros_bridge, tf_bcast
from mecanum_lab.engine import SimEngine
from mecanum_lab.robot_io import RobotIO
from mecanum_lab.stub import StubBus
from mecanum_lab.types import Gps, Kf, Odom, Pose, Rect, Scan, World, cfg_get, load_config, topic

# The tree of `tf:=localizer` in the ohm exercise: the mapper owns `map -> <robot>/odom`, so the
# simulator calls its ground frame `hall` (tf_bcast.GROUND) and `map` is not in the graph at all.
SLAM = {"tf": {"tree": "slam", "namespaces": True, "rate": 20.0, "map_to_odom": "identity"}}
SIM_TREE = {"tf": {"tree": "sim", "namespaces": True}}


class V:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x, self.y, self.z = x, y, z


class Attr:
    """Message dummy that grows any attribute on first use (tests/test_ros_message_shapes.py)."""

    def __getattr__(self, name):
        value = Attr()
        self.__dict__[name] = value
        return value


M = {"Header": lambda frame_id="": SimpleNamespace(frame_id=frame_id, stamp=None),
     "Time": lambda sec=0, nanosec=0: SimpleNamespace(sec=sec, nanosec=nanosec),
     "PoseWithCovarianceStamped": Attr, "Pose": Attr, "Point": V, "Quaternion": V,
     "Vector3": V, "String": lambda data="": SimpleNamespace(data=data)}


def bus_with(robot: str, *measurements):
    """A stub bus with the sim's measurements already on their topics."""
    bus = StubBus()
    for kind, payload in measurements:
        bus.publish(topic(kind, robot), payload)
    return bus


# -------------------------------------------------------------------------------- the stamp on `kf/pose`

def test_the_estimate_is_stamped_with_the_measurement_it_was_predicted_from():
    """`t=0.0` was a pose at the epoch; the odometry's stamp is the moment this pose describes."""
    rob = RobotIO("alice", bus=bus_with("alice", ("odom", Odom(t=4.02, x=1.0, y=2.0, theta=0.3,
                                                               vx=0.5, vy=0.0, omega=0.1))))
    rob.send_kf(1.04, 2.02, 0.31, 0.1, 0.1, 0.05)
    assert rob.bus.last("kf", "alice")[0].t == pytest.approx(4.02)


def test_the_odometry_is_preferred_over_a_newer_scan_and_the_others_are_the_fallback():
    """The transform above the odometry exists at the *odometry's* stamp — chasing the newest stamp
    would ask every TF consumer to extrapolate into a future the simulator has not published."""
    rob = RobotIO("alice", bus=bus_with("alice",
                                        ("odom", Odom(t=4.02, x=1.0, y=2.0, theta=0.3,
                                                      vx=0.5, vy=0.0, omega=0.1)),
                                        ("scan", Scan(t=4.06, ranges=[1.0] * 4)),
                                        ("gps", Gps(t=3.9, x=1.0, y=2.0, theta=0.3,
                                                    sigma_xy=0.3, sigma_theta=0.02))))
    rob.send_kf(1.0, 2.0, 0.3)
    assert rob.last_stamp() == pytest.approx(4.02)
    rob2 = RobotIO("bob", bus=bus_with("bob", ("scan", Scan(t=7.1, ranges=[1.0] * 4)),
                                       ("gps", Gps(t=6.9, x=1.0, y=2.0, theta=0.3,
                                                   sigma_xy=0.3, sigma_theta=0.02))))
    assert rob2.last_stamp() == pytest.approx(7.1), "a task without odometry still gets a real stamp"


def test_a_stamp_of_one_own_wins_and_nothing_received_is_still_zero():
    rob = RobotIO("alice", bus=bus_with("alice", ("odom", Odom(t=4.02, x=1.0, y=2.0, theta=0.3,
                                                               vx=0.5, vy=0.0, omega=0.1))))
    rob.send_kf(1.0, 2.0, 0.3, t=9.5)
    assert rob.bus.last("kf", "alice")[0].t == pytest.approx(9.5), "the caller knows its own moment"
    quiet = RobotIO("carol", bus=StubBus())
    assert quiet.last_stamp() == 0.0
    quiet.send_kf(1.0, 2.0, 0.3)
    assert quiet.bus.last("kf", "carol")[0].t == 0.0, \
        "a node that has received nothing says so rather than inventing a moment"


def test_the_call_signature_of_a_student_node_stays_the_call_signature_it_had():
    """`t` is the last keyword of `send_kf`, after `info`: every positional call of every template
    (and `student/kf_template.py` in the other repos) keeps meaning exactly what it meant."""
    rob = RobotIO("alice", bus=bus_with("alice", ("odom", Odom(t=1.0, x=0.0, y=0.0, theta=0.0))))
    rob.send_kf(1.0, 2.0, 0.3, 0.1, 0.1, 0.05, {"neff": 400})
    kf = rob.bus.last("kf", "alice")[0]
    assert (kf.x, kf.sx, kf.sth) == (1.0, 0.1, 0.05)
    assert json.loads(rob.bus.last("kfinfo", "alice")[0]) == {"neff": 400}


# --------------------------------------------------------------- the bridge: header in, header out

def test_the_bridge_writes_the_stamp_into_the_header_of_the_message():
    msg = ros_bridge.to_ros(M, "kf", Kf(t=12.34, x=1.0, y=2.0, theta=0.5, sx=0.2, sy=0.3, sth=0.04),
                            "alice", SIM_TREE)
    assert msg.header.stamp.sec == 12
    assert msg.header.stamp.nanosec == pytest.approx(340_000_000, abs=1_000), \
        "sim seconds to the millisecond, with the nano-second rounding the shared `_header()` does " \
        "(int(t % 1.0 * 1e9)) — the same arithmetic every other kind of this graph is stamped with"
    assert msg.header.frame_id == "map", "the simulator's own tree calls the hall `map`"


def test_the_estimate_is_stamped_in_the_frame_the_run_uses():
    """The same payload, the tree of `tf:=localizer`: `hall`, because that is the root of that graph."""
    msg = ros_bridge.to_ros(M, "kf", Kf(t=1.0, x=1.0, y=2.0, theta=0.0, sx=0.1, sy=0.1, sth=0.02),
                            "alice", SLAM)
    assert msg.header.frame_id == "hall"
    assert "map" not in (tf_bcast.frames("alice", SLAM).values(),), "…and `map` really is absent there"


# ------------------------------------------------------------- /sim/config carries the tree it runs

def test_the_running_config_names_the_frame_of_the_hall():
    """A node may not guess the ground frame, so the run has to say it (engine.config_json)."""
    cfg = load_config(None, {"gui": False, "tf": {"tree": "slam"}})
    hall = World(name="flat", walls=[Rect(0, 0, 10, 10)], spawns=[Pose(1.0, 1.0, 0.0)])
    eng = SimEngine(hall, cfg, seed=1)
    profile = json.loads(eng.config_json())
    assert cfg_get(profile, "tf.tree") == "slam", \
        "the ground frame of a run is a fact of the run, not of the caller's config file"
    assert cfg_get(profile, "seed") == eng.seed and "pois" not in profile


def test_the_bus_adopts_the_tree_of_the_run_and_never_overrides_what_it_was_given():
    """`RclpyBus` builds every header out of `self.cfg`, so `self.cfg` has to be this run's."""
    def adopt(cfg, text):
        bus = SimpleNamespace(cfg=cfg)
        ros_bridge.RclpyBus._adopt_tf(bus, text)
        return bus.cfg

    running = json.dumps({"odom": {"rate": 50.0}, "tf": SLAM["tf"]})
    assert cfg_get(adopt({}, running), "tf.tree") == "slam", "a controller learns the tree it drives in"
    untouched = adopt({"tf": {"tree": "sim"}}, running)
    assert untouched["tf"]["tree"] == "sim", "a bus built with a config knows better than a topic"
    given = {"gui": False}
    adopt(given, running)
    assert "tf" not in given, "the caller's dict stays the caller's — the bus gets a new one"
    for junk in ("", "not json", "[]", "[1, 2]", json.dumps({"odom": {}}), None, 7):
        assert adopt({}, junk) == {}, "no answer, no tf block or no dict changes nothing"


def test_a_node_reading_its_own_profile_gets_the_frame_of_the_run():
    """The whole chain that was broken, one link each side of the sentence.

    `/sim/config` carries the block (the test above), a bus adopts it (`_adopt_tf`, the ROS door), and a
    node that only ever calls `sensor_profile()` — which is what `ohm_localization/view.py` falls back on
    when its launch names no frame, and what a student writes against — reads the same answer. Both used
    to get a profile with no `tf` in it, whatever the run was, and `tf_bcast` then named the header out of
    its own default. The stub bus has no header to get wrong, which is exactly why no grade and no
    `./lab run` ever saw this: only the ROS door builds a header, and only a TF consumer reads one.
    """
    rob = RobotIO("alice", bus=StubBus())
    rob.bus.publish(topic("config"), json.dumps({"odom": {"rate": 50.0}, "tf": SLAM["tf"]}))
    assert tf_bcast.frame_for("kf", "alice", rob.sensor_profile()) == "hall"
    quiet = RobotIO("bob", bus=StubBus())
    assert tf_bcast.frame_for("kf", "alice", quiet.sensor_profile()) == "map", \
        "no simulator answering is the `sim` default, said out loud rather than by accident — in a `slam` " \
        "graph that is the wrong name forever, which is the failure the two tests above exist for"
