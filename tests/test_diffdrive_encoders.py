"""The differential drive (`--variant diff`) and the wheel encoders (`/<robot>/joint_states`).

Three promises are pinned here. The differential drive is the textbook one (v and omega from the two
wheels, no strafing, slots 2 and 3 of `wheel_speeds` ignored). The encoders are off by default and,
when on, move no other sensor's random stream — the graded runs are calibrated on those streams.
`/joint_states` survives the ROS conversion. The lab that uses both is its own repository
(`ohm-odometry`); its tests live there.
"""
import math
from types import SimpleNamespace

import pytest

from mecanum_lab import diffdrive, ros_bridge
from mecanum_lab.engine import SimEngine
from mecanum_lab.types import Joints, load_config
from mecanum_lab.worlds import load_world



def engine(encoders: bool, seed: int = 3):
    cfg = load_config(None, {"world": "open", "gui": False, "debug_truth": True,
                             "encoders": {"enabled": encoders}})
    return SimEngine(load_world("open", cfg=cfg), cfg, seed=seed)


def drive(eng, name, wheels, seconds):
    rob = eng.robots[name]
    rob.mode = "wheels"
    out = []
    while eng.t < seconds:
        rob.wheel_cmd, rob.t_cmd = list(wheels), eng.t
        eng.step(1.0 / 50)
        out += eng.drain()
    return out


def test_differential_drive_is_the_textbook_model():
    eng = engine(True)
    rob = eng.spawn("dd", "diff")
    g = rob.chassis.geometry
    drive(eng, "dd", [4.0, 6.0, 99.0, -99.0], 2.0)      # slots 2 and 3 must not matter
    assert rob.twist.vx == pytest.approx(g.r / 2 * (4.0 + 6.0))
    assert rob.twist.vy == pytest.approx(0.0, abs=1e-12)
    assert rob.twist.omega == pytest.approx(g.r / g.track * (6.0 - 4.0))
    assert diffdrive.inverse_kinematics(g, 0.25, 1 / 3) == pytest.approx([4.0, 6.0])


def test_cmd_vel_on_a_differential_drive_drops_the_sideways_part():
    eng = engine(False)
    rob = eng.spawn("dd", "diff")
    rob.chassis.set_twist(0.2, 0.3, 0.0)
    assert rob.chassis.cmd == pytest.approx([4.0] * 4)


def test_encoders_are_off_by_default():
    assert not load_config()["encoders"]["enabled"]
    eng = engine(False)
    eng.spawn("m1", "stock")
    assert not [k for k, _, _ in drive(eng, "m1", [3.0] * 4, 1.0) if k == "joints"]


def test_encoders_do_not_move_the_other_random_streams():
    streams = []
    for on in (False, True):
        eng = engine(on)
        eng.spawn("m1", "stock")
        msgs = drive(eng, "m1", [3.0, -2.0, 1.0, 4.0], 3.0)
        streams.append([(k, vars(p)) for k, _, p in msgs if k in ("odom", "imu", "scan")])
    assert streams[0] == streams[1]


def test_joint_states_count_ticks_per_driven_wheel():
    eng = engine(True)
    eng.spawn("dd", "diff")
    eng.spawn("mm", "stock")
    eng.robots["mm"].mode = "wheels"
    joints = {}
    for kind, robot, payload in drive(eng, "dd", [5.0, 5.0, 0, 0], 2.0):
        if kind == "joints":
            joints[robot] = payload
    assert joints["dd"].name == ["wheel_left", "wheel_right"]
    assert joints["mm"].name == ["wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"]
    tick = 2 * math.pi / 1024
    left = joints["dd"].position[0]
    assert abs(left / tick - round(left / tick)) < 1e-6        # whole ticks
    assert 8.0 < left < 10.0                                    # ~5 rad/s for 2 s, minus the lag


def test_joint_states_survive_the_ros_conversion():
    msg_cls = lambda **kw: SimpleNamespace(**kw)                # noqa: E731
    M = {"JointState": msg_cls,
         "Header": lambda frame_id="": SimpleNamespace(frame_id=frame_id, stamp=None),
         "Time": lambda sec=0, nanosec=0: SimpleNamespace(sec=sec, nanosec=nanosec)}
    sent = Joints(t=2.5, name=["wheel_left", "wheel_right"], position=[1.0, 2.0],
                  velocity=[0.5, -0.5])
    msg = ros_bridge.to_ros(M, "joints", sent, robot="dd", cfg={})
    assert msg.header.frame_id.endswith("base_link")
    back = ros_bridge.from_ros("joints", msg)
    assert back == sent
