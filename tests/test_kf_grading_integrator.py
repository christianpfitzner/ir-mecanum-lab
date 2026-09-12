"""Tests for the KF grading in mecanum_lab/grade.py (Experiment 2, integrator).

No physics is needed here: the grader only reads topics, so we feed it truth, gps and
kf/pose as functions of time and check its verdict. That is exactly why grading in the
stub run is identical to running under ROS.
"""
import json
import math

import pytest

from mecanum_lab.grade import Grader, drive_duration, drive_command
from mecanum_lab.stub import StubBus
from mecanum_lab.types import Gps, Kf, Pose, Twist

TASK = {
    "id": "kf_test", "title": "K9 — Graded task", "points": 30, "experiment": 2, "kind": "kf",
    "timeout": 14.0, "warmup": 1.0, "sensor": "gps", "estimate": "kf",
    "drive": [{"vx": 0.5, "duration": 8.0}], "repeat": 1,
    "rmse_max": 0.40, "improvement_min": 1.5, "rate_min": 5.0, "contacts_max": 0,
}


def drive_as_time(t: float) -> float:
    """x path of the grader's command (vx = 0.5 m/s), for truth/gps/kf on the same path."""
    return 0.5 * t


def run_through(task, estimate_error=0.10, gps_error=0.70, sx=0.20, sy=0.20,
               outage_from=None, seconds=14.0, dt=0.02, profile=None):
    """Grade against synthetic topics: truth, GPS with error, estimate with error."""
    bus = StubBus("test")
    bew = Grader("alice", task["id"], bus, {"tasks": [task], "order": [task["id"]]},
                 world_info={"name": "arena", "size": [24, 16], "goal": None,
                             "spawns": [[1, 1, 0]], "walls": 4}).start()
    if profile is not None:
        bus.publish("/sim/config", json.dumps(profile))
    t = 0.0
    while not bew.tick(dt) and t < seconds:
        t += dt
        truth_pose = drive_as_time(t)
        bus.publish("/alice/truth", Pose(truth_pose, 0.0, 0.0, t))
        bus.publish("/sim/robots", json.dumps([{"name": "alice", "index": 0, "contacts": 0,
                                                "distance": truth_pose, "mission": "running",
                                                "mode": "pass-through", "task": task["id"]}]))
        gps_frisch = outage_from is None or t < outage_from
        if gps_frisch:
            mess = Gps(t=t, x=truth_pose + gps_error, y=0.5 * gps_error, theta=0.0)
            bus.publish("/alice/gps", mess)
        bus.publish("/alice/kf/pose", Kf(t=t, x=truth_pose + estimate_error, y=0.1 * estimate_error,
                                         theta=0.0, sx=sx, sy=sy))
    return bew.report()["tasks"][0]


# ---------------------------------------------------------------- command sequence


def test_drive_command_counts_and_repeats():
    task = {"drive": [{"vx": 0.4, "duration": 3.0}, {"vx": 0.0, "omega": 0.5, "duration": 2.0}],
               "repeat": 2, "timeout": 40}
    assert drive_duration(task) == pytest.approx(10.0)
    assert drive_command(task, 1.0) == pytest.approx(Twist(vx=0.4))
    assert drive_command(task, 4.0).omega == pytest.approx(0.5)
    assert drive_command(task, 6.0) == pytest.approx(Twist(vx=0.4))     # second round


def test_drive_command_without_segments_just_drives():
    task = {"vx": 0.3, "timeout": 12.0}
    assert drive_duration(task) == pytest.approx(11.0)
    assert drive_command(task, 3.0) == pytest.approx(Twist(vx=0.3))


def test_cov_diag_also_reads_multi_value_fields_as_under_ros():
    """ROS delivers the covariance as a numpy-like array — `cov or []` would fail there."""
    from mecanum_lab.ros_bridge import cov_diag

    class Field(list):
        def __bool__(self):                          # numpy behaves the same way
            raise ValueError("The truth value of an array is ambiguous")

    vier_x_vier = Field([1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4])
    assert cov_diag(vier_x_vier, n=4) == [1.0, 2.0, 3.0, 4.0]
    assert cov_diag(None) == [0.0] * 6


def test_sine_shaped_yaw_rate_superposes_the_base_value():
    task = {"drive": [{"vx": 0.5, "omega": 0.2, "sine": 0.1, "frequency": 0.5,
                          "duration": 6.0}]}
    bei_null = drive_command(task, 0.0).omega
    bei_halber_periode = drive_command(task, 1.0).omega
    assert bei_null == pytest.approx(0.2, abs=1e-9)
    assert bei_halber_periode == pytest.approx(0.2, abs=1e-6)     # sin(pi) = 0
    assert drive_command(task, 0.5).omega == pytest.approx(0.3, abs=1e-6)


# -------------------------------------------------------------------------- grading


def test_good_estimate_passes_and_reports_every_number():
    result = run_through(dict(TASK), sx=0.10, sy=0.10)
    mess = result["measured"]
    assert result["passed"], result["reason"]
    assert mess["rmse"] == pytest.approx(0.10, abs=0.02)
    assert mess["rmse_gps"] == pytest.approx(0.78, abs=0.05)
    assert mess["improvement"] > 1.5 and mess["rate_hz"] >= 5.0
    assert 0.4 < mess["nees"] < 1.2, mess["nees"]      # sx=sy=0.1 at 0.1 m error -> ~0.5
    assert result["points"] == 30


def test_filter_that_only_repeats_the_measurement_fails_on_improvement():
    result = run_through(dict(TASK), estimate_error=0.70, sy=0.2)
    assert not result["passed"]
    assert "improvement" in result["reason"]
    assert result["points"] == 0.0


def test_dishonest_covariance_is_caught_by_the_nees():
    scharf = run_through(dict(TASK, nees=[0.5, 3.0]), estimate_error=0.10, sx=0.01, sy=0.01)
    assert not scharf["passed"] and "NEES" in scharf["reason"]
    assert scharf["measured"]["nees"] > 3.0


def test_gps_outage_without_fusion_fails():
    """After t = 6 s no fixes arrive: without countermeasures the error keeps growing."""
    task = dict(TASK, timeout=14.0, outage={"duration_min": 2.0, "error_max": 0.5})
    gut = run_through(task, estimate_error=0.10, outage_from=6.0)
    assert gut["measured"]["outage_duration"] > 2.0, "GPS outage not detected"
    assert gut["passed"], gut["reason"]
    schlampig = run_through(task, estimate_error=0.90, outage_from=6.0)
    assert not schlampig["passed"] and "GPS outage" in schlampig["reason"]


def test_no_estimate_is_a_clear_reason_text():
    bus = StubBus("test")
    bew = Grader("alice", TASK["id"], bus,
                 {"tasks": [TASK], "order": [TASK["id"]]}).start()
    t = 0.0
    while not bew.tick(0.02) and t < 14.0:
        t += 0.02
        bus.publish("/alice/truth", Pose(0.5 * t, 0.0, 0.0))
    result = bew.report()["tasks"][0]
    assert not result["passed"]
    assert "kf/pose" in result["reason"]


def test_lower_k3_limit_stays_below_the_measured_scatter():
    """The floor is 0.05 because paced runs measure NEES 0.09 … 0.65 for the reference filter.

    With the old 0.15 one of those runs — RMSE 0.039 m, 17x improvement over GPS — lost 20
    points for being cautious. Tightening this number again without new measurements is the
    kind of change that turns a grade into a lottery; the upper bound is the one that matters.
    """
    from mecanum_lab import tasks
    k3 = [a for a in tasks.load_tasks()["tasks"] if a["id"] == "kf_kovarianz"][0]["nees"]
    assert k3[0] <= 0.09, k3
    assert k3[1] >= 3.0, k3


def test_check_profile_deviation_is_reported():
    profile = {"gps": {"rate": 5.0, "sigma_xy": 0.5}}
    expected = dict(TASK, sim={"gps": {"rate": 5.0, "sigma_xy": 0.5}})
    with_profile = run_through(expected, profile=profile)
    assert with_profile["passed"], with_profile["reason"]
    anders = run_through(expected, profile={"gps": {"rate": 20.0, "sigma_xy": 5.0}})
    assert not anders["passed"] and "test profile" in anders["reason"]


def test_missing_truth_is_a_tutor_error_not_a_student_error():
    bus = StubBus("test")
    bew = Grader("alice", TASK["id"], bus,
                 {"tasks": [TASK], "order": [TASK["id"]]}).start()
    for _ in range(900):
        if bew.tick(0.02):
            break
    result = bew.report()["tasks"][0]
    assert not result["passed"] and "samples" in result["measured"]


# ------------------------------------------------------------ world for the tasks

def _world(task, world=None):
    """Which arena does `./lab` pick when the task recommends a world?"""
    import types
    from mecanum_lab import node
    args = types.SimpleNamespace(world=world, task=task)
    return node.cfg_get_world({"world": "maze"}, args, {})


def_test = _world        # Pytest only collects test_* names; this one is a helper


def test_experiment_1_gets_production_and_experiment_2_arena():
    # config/tasks.json says it; ignoring that grades a square run inside the maze arena
    # and counts wall contacts as lab work.
    assert _world("v1") == "production"
    assert _world("kf_alle") == "arena"
    assert _world("alle") == "production"
    assert _world(None) == "maze"                      # without a task the config default applies
    assert _world("v1", world="track") == "track"      # an explicit choice wins


def test_world_of_the_task_also_applies_without_auto():
    assert _world("kf_gps", world="auto") == "arena"
    assert _world("quadrat") == "production"


def test_run_loop_keeps_the_robot_list_current():
    """/sim/robots is the grader's only source for mission_state and distance.

    The engine only pushes the list on spawn and reset. A run that never refreshes it
    grades a drive whose path stays 0 for the whole task — and does not even notice that
    the students' mission finished long ago.
    """
    from mecanum_lab import node, worlds
    from mecanum_lab.engine import SimEngine
    from mecanum_lab.robot_io import robot_info
    from mecanum_lab.types import DEFAULT_CONFIG

    bus = StubBus()
    eng = SimEngine(worlds.load_world("arena"), dict(DEFAULT_CONFIG), seed=5)
    eng.spawn("karl")
    node.subscribe(bus, eng, "karl")
    bus.pub("twist", "karl")(Twist(vx=0.5, vy=0.0, omega=0.0))
    node.run_loop(eng, bus, None, [], 0.6)
    info = robot_info(bus, "karl")
    assert info.get("distance", 0) > 0.05, info
    assert "mission" in info and "contacts" in info, info
