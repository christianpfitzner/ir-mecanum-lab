"""The GPS fix brings its own uncertainty: σ on the message, a covariance topic on the bus.

`GpsSensor` has always known the σ of every emission — a `gps.zones` shadow multiplies it, and the
delay ring keeps a fix waiting — and then threw the number away. Everything a filter could read
came from `config/default.json`: the *setting*, which is wrong for every fix measured in a shadow and
wrong for every delayed fix after the robot has moved on. These tests pin the σ to the emission that
was drawn with it (`types.Gps.sigma_xy`, `sigma_theta`) and to the ROS mirror of that message.
"""
import csv
import os
import tempfile
from types import SimpleNamespace

import pytest

from mecanum_lab import overlays, ros_bridge, sensors
from mecanum_lab.engine import SimEngine
from mecanum_lab.logbook import Logbook
from mecanum_lab.types import Gps, Pose, Twist, load_config
from mecanum_lab.worlds import load_world


def sensor(**cfg) -> sensors.GpsSensor:
    return sensors.GpsSensor(sensors.Noise(5), cfg)


def test_the_fix_carries_the_sigma_it_was_drawn_with():
    """A fix says how uncertain it is, without the reader having to look the number up somewhere else."""
    fix = sensor(sigma_xy=0.3, sigma_theta=0.05).fix(Pose(3.0, 2.0, 0.4), t=1.0)
    assert (fix.sigma_xy, fix.sigma_theta) == (0.3, 0.05)


def test_a_fix_from_a_shadow_carries_the_inflated_sigma_and_the_settings_do_not():
    """Under a shelf the sky is worse, and only the message knows it: scale 3 means σ 0.9 in there.

    The configured `sensor.sigma_xy` stays 0.3 — that is the setting, and `/sensor/info` reports it.
    The two numbers disagreeing is not a bug to harmonise away, it is the whole lesson: the filter has
    to weight the reading it is holding, not the settings file of a device it is not standing in.
    """
    schatten = dict(name="rack", rect=[2.0, 1.0, 4.0, 3.0], sigma_scale=3)
    empfaenger = sensor(sigma_xy=0.3, sigma_theta=0.05, zones=[schatten])
    assert empfaenger.fix(Pose(6.0, 2.0), t=1.0).sigma_xy == 0.3          # open sky: the setting
    assert empfaenger.fix(Pose(3.0, 2.0), t=2.0).sigma_xy == pytest.approx(0.9)
    assert empfaenger.sigma_xy == 0.3, "the receiver's own setting must not drift with the place"


def test_a_delayed_fix_carries_the_sigma_of_where_it_was_measured():
    """Transport delay is the case that makes this a correctness question rather than a nicety.

    The robot is driven from open sky into the shadow while `delay_ticks = 2` fixes are still on the
    wire. The fix that arrives just after the robot crossed the line was *measured* outside, so it
    carries σ 0.3; a reader that asks the sensor for σ at delivery time gets 0.9 and would trust a
    wide reading too much — and the same reader two emissions later distrusts a narrow one that has
    since become true. The σ belongs to the moment of measurement, so it travels in the message.
    """
    schatten = dict(name="rack", rect=[10.0, 0.0, 16.0, 6.0], sigma_scale=3)
    empfaenger = sensor(sigma_xy=0.3, sigma_theta=0.05, delay_ticks=2, zones=[schatten])
    assert empfaenger.fix(Pose(1.0, 2.0), t=1.0) is None                  # ring still filling
    assert empfaenger.fix(Pose(2.0, 2.0), t=2.0) is None
    ueber_der_schwelle = empfaenger.fix(Pose(12.0, 2.0), t=3.0)            # measured in the open
    assert ueber_der_schwelle.sigma_xy == 0.3, "a fix from the open must not be dressed in shadow σ"
    assert empfaenger.fix(Pose(12.0, 2.0), t=4.0).sigma_xy == 0.3
    assert empfaenger.fix(Pose(12.0, 2.0), t=5.0).sigma_xy == pytest.approx(0.9)


# --------------------------------------------------------------------- the mirror on the ROS side


class Attr:
    """Message dummy that grows any attribute on first use — enough for a conversion test."""

    def __getattr__(self, name):
        value = Attr()
        self.__dict__[name] = value
        return value


class Header:
    def __init__(self, stamp=None, frame_id=""):
        self.stamp, self.frame_id = stamp or SimpleNamespace(sec=0, nanosec=0), frame_id


M = {"Header": Header, "Time": lambda sec=0, nanosec=0: SimpleNamespace(sec=sec, nanosec=nanosec),
     "Vector3": lambda x=0.0, y=0.0, z=0.0: SimpleNamespace(x=x, y=y, z=z),
     "Quaternion": lambda x=0.0, y=0.0, z=0.0, w=1.0: SimpleNamespace(x=x, y=y, z=z, w=w),
     "PoseStamped": Attr, "PoseWithCovarianceStamped": Attr,
     "String": lambda data="": SimpleNamespace(data=data)}
CFG = {"tf": {"enabled": True, "namespaces": True}}


def fix() -> Gps:
    return Gps(t=1.5, x=2.0, y=3.0, theta=0.25, quality=1, sats=5,
               sigma_xy=0.9, sigma_theta=0.04)


def test_the_covariance_topic_is_the_same_fix_squared():
    """`/gps_cov` is the fix with an R: the diagonal is that emission's σ², nothing else is filled.

    ROS orders a pose covariance as [x, y, z, roll, pitch, yaw] in a 36-field row-major matrix. The
    sim draws x and y from one σ and yaw from another and has no cross terms, so twelve of the thirty-
    six diagonal slots carry a stand-in (z, roll, pitch are not measured at all) and all 30 off-
    diagonal slots are exactly 0.0 — a filter that reads a cross term as information reads a lie.
    """
    msg = ros_bridge.to_ros(M, "gpscov", fix(), "alice", CFG)
    cov = list(msg.pose.covariance)
    assert len(cov) == 36
    assert (cov[0], cov[7], cov[35]) == pytest.approx((0.81, 0.81, 0.0016))
    assert (cov[14], cov[21], cov[28]) == (1e-12, 1e-12, 1e-12)
    assert all(cov[k] == 0.0 for k in range(36) if k not in (0, 7, 14, 21, 28, 35))
    assert (msg.pose.pose.position.x, msg.pose.pose.position.y) == (2.0, 3.0)


def test_the_covariance_topic_carries_the_frame_and_the_stamp_of_its_gps():
    """Two messages about one measurement must not disagree about where and when it was taken.

    The frame is asked from `tf_bcast` for kind `gps`, not for `gpscov`, and the stamp is the emission
    stamp — the pairing a subscriber does by stamp only works while both messages say 1.5 s.
    """
    cov_msg, gps_msg = (ros_bridge.to_ros(M, kind, fix(), "alice", CFG)
                        for kind in ("gpscov", "gps"))
    assert cov_msg.header.frame_id == gps_msg.header.frame_id
    assert ros_bridge._stamp(cov_msg.header) == pytest.approx(1.5)


def test_a_subscriber_asks_for_the_gps_that_carries_its_sigma():
    """`rob.gps()` must not answer σ 0 merely because ROS is in the game.

    The bus is built without `__init__` here (that would want a sourced rclpy and a live node); what is
    under test is one decision: which topic a `gps` subscription lands on, and what the callback hands
    back. It lands on `/gps_cov` — the only GPS topic that can carry an R — and still stores its answer
    under `/alice/gps`, because that is the key `age("gps")` counts on.
    """
    bus = object.__new__(ros_bridge.RclpyBus)
    bus.M, bus._last, bus.cfg = M, {}, CFG
    abonniert = []
    bus._subscribe = lambda cls_key, name, wrap: abonniert.append((cls_key, name, wrap))
    gefangen = []
    bus.sub("gps", "alice", gefangen.append)
    assert [a[:2] for a in abonniert] == [("PoseWithCovarianceStamped", "/alice/gps_cov")]
    abonniert[0][2](ros_bridge.to_ros(M, "gpscov", fix(), "alice", CFG))
    ankunft = bus._last["/alice/gps"][0]
    assert (ankunft.x, ankunft.sigma_xy) == (2.0, pytest.approx(0.9))


def test_reading_the_covariance_topic_back_gives_the_same_sigmas():
    """Round trip: a node that subscribes to `/gps_cov` ends up with the same `Gps` as the sim sent."""
    msg = ros_bridge.to_ros(M, "gpscov", fix(), "alice", CFG)
    zurueck = ros_bridge.from_ros("gpscov", msg)
    assert (zurueck.x, zurueck.y) == (2.0, 3.0)
    assert (zurueck.sigma_xy, zurueck.sigma_theta) == pytest.approx((0.9, 0.04))


# ------------------------------------------------------------- the two places a human sees the σ


class SimpleRenderer:
    """The bit of `Renderer` that `overlays.sensor_readout()` looks at: an engine and a config."""

    def __init__(self, engine):
        self.engine, self.cfg = engine, engine.cfg


def test_the_window_shows_the_sigma_that_arrived():
    """The readout line is where a student without `ros2 topic echo` sees the number — show the σ of
    the fix that came in, not the one in the settings."""
    cfg = load_config(None, {"gui": False})
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=1)
    robot = eng.spawn("muster")
    eng._gps.sigma_xy, eng._gps.sigma_theta = 0.3, 0.05
    for _ in range(int(3.0 * cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.4))
        eng.step(eng.sub_step)
    assert robot.gps is not None, "no GPS arrived in three seconds"
    line = next(t for t, _c in overlays.sensor_readout(SimpleRenderer(eng), robot)
                if t.startswith("gps "))
    assert "σ0.30" in line, line
    eng._gps.sigma_xy = 0.0                          # a noiseless receiver has nothing to report
    for _ in range(int(1.0 * cfg["rate"])):
        eng.step(eng.sub_step)
    line = next(t for t, _c in overlays.sensor_readout(SimpleRenderer(eng), robot)
                if t.startswith("gps "))
    assert "σ" not in line, f"a receiver with σ 0 prints zeros: {line}"


def test_the_log_keeps_the_sigma_of_the_last_fix(tmp_path):
    """The CSV is how a wrong R is found afterwards: `sigma_gps` is the arrival, not the setting."""
    cfg = load_config(None, {"gui": False})
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=1)
    eng.spawn("muster")
    pfad = os.path.join(tempfile.mkdtemp(dir=str(tmp_path)), "lauf.csv")
    buch = Logbook(eng, pfad, interval=0.1)
    buch.tap("gps", "muster", Gps(t=1.0, x=1.0, y=2.0, sigma_xy=0.9, sigma_theta=0.04))
    buch.tick()
    buch.close()
    with open(pfad, encoding="utf-8") as fh:
        header, *zeilen = list(csv.reader(fh, delimiter=";"))
    assert "sigma_gps" in header and "sigmatheta_gps" in header, \
        f"the log lost the σ columns: {header}"
    assert header.index("sigmatheta_gps") == header.index("sigma_gps") + 1, \
        "the two σ columns belong next to each other"
    row = dict(zip(header, zeilen[0]))
    assert (row["sigma_gps"], row["sigmatheta_gps"]) == ("0.9000", "0.0400"), \
        "the two columns hold the σ of the fix, four decimals like every other column"
