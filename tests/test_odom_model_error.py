"""Odometry that is wrong the way real odometry is wrong: model error, not noise (`odom.geometry`).

Why this needed a switch at all: `SimEngine` handed the `OdometrySensor` the *true* chassis
`Geometry`, so the commonest field error — a wheel radius that is not what the drawing says, a
lever arm measured in the wrong unit — could not be expressed. Noise averages out over a drive;
model error scales with the distance driven and is exactly what T2 and T3 make students fight.

The default (`odom.geometry` empty) must keep the old numbers: the second-to-last test measures
it, because the graded thresholds in config/tasks.json are calibrated on correct wheel constants.
"""
import math

from mecanum_lab.engine import SimEngine
from mecanum_lab.types import Twist, cfg_get, load_config
from mecanum_lab.worlds import load_world

NAME = "muster"
RADIUS_ERROR = 1.05
LEVER_ERROR = 0.97
FULL_TURN = 2 * math.pi


def config_file(name: str) -> str:
    """Path to a file in config/ — the demo file is part of what this test protects."""
    import os
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config", name)


def driving_engine(geometry: dict | None = None, world: str = "arena",
                   path: str | None = None, seed: int = 2) -> SimEngine:
    """One robot, with the odometry integrator believing `geometry` instead of the truth."""
    overrides = {"gui": False}
    if geometry is not None:
        overrides["odom"] = {"geometry": geometry}
    cfg = load_config(path, overrides)
    eng = SimEngine(load_world(world, cfg=cfg), cfg, seed=seed)
    eng.spawn(NAME)
    return eng


def drive(eng: SimEngine, twist: Twist, seconds: float) -> None:
    """Hold one body speed for `seconds` of simulation time (the watchdog needs the refresh)."""
    for _ in range(int(seconds / eng.sub_step)):
        eng.set_cmd_vel(NAME, twist)
        eng.step(eng.sub_step)


def measured(eng: SimEngine) -> tuple:
    """(true displacement, odometry displacement, true yaw, odometry yaw) since the spawn pose."""
    r = eng.robots[NAME]
    home = eng.world.spawn_pose(r.spec.index)
    return (math.dist((home.x, home.y), (r.pose.x, r.pose.y)),
            math.dist((home.x, home.y), (r.odom.x, r.odom.y)),
            r.pose.theta, r.odom.theta)


def yaw_error(estimate: float, truth: float) -> float:
    """estimate - truth, normalised to (-pi, pi] — one circle rotated is a small number."""
    return (estimate - truth + math.pi) % FULL_TURN - math.pi


def test_wheel_radius_scale_reports_the_path_too_long():
    """Wheels 5 % bigger than believed -> 5 % more metres on the odometer, nothing else."""
    eng = driving_engine({"wheel_radius_scale": RADIUS_ERROR})
    drive(eng, Twist(vx=0.3), 6.0)
    truth, estimate, _, _ = measured(eng)
    assert truth > 1.5                                  # it really drove
    assert abs(estimate / truth - RADIUS_ERROR) < 0.002     # 5 %, and nothing but the 5 %


def test_lever_scale_leaves_a_full_circle_rotated():
    """A lever arm 3 % short turns every wheel-speed difference into 3 % too much yaw rate."""
    eng = driving_engine({"lever_scale": LEVER_ERROR})
    drive(eng, Twist(omega=0.5), FULL_TURN / 0.5)      # one commanded revolution
    _, _, theta_truth, theta_odom = measured(eng)
    assert abs(theta_truth) < 0.06                      # the truth is back where it started
    expected = FULL_TURN * (1.0 / LEVER_ERROR - 1.0)    # 0.194 rad = 11 deg for one round
    assert abs(abs(yaw_error(theta_odom, theta_truth)) - expected) < 0.03
    assert abs(yaw_error(theta_odom, theta_truth)) > 0.1     # visible without knowing the formula


def test_scale_xy_stretches_the_path_but_not_the_turn():
    """Wrong radius, honest gyro: the metres scale, the yaw rate stays right."""
    eng = driving_engine({"scale_xy": 1.1})
    drive(eng, Twist(vx=0.3), 4.0)
    truth, estimate, _, _ = measured(eng)
    assert abs(estimate / truth - 1.1) < 0.002
    eng = driving_engine({"scale_xy": 1.1})
    drive(eng, Twist(omega=0.5), FULL_TURN / 0.5)
    _, _, theta_truth, theta_odom = measured(eng)
    assert abs(yaw_error(theta_odom, theta_truth)) < 0.02    # scale_xy must not touch omega


def test_bias_xy_shifts_the_reported_pose_by_a_constant():
    """A wrong odom origin is not drift: the whole reported world moves, the drive stays fine."""
    eng = driving_engine({"bias_xy": [0.3, -0.1]})
    drive(eng, Twist(vx=0.3), 2.0)
    r = eng.robots[NAME]
    assert abs((r.odom.x - 0.3) - r.pose.x) < 0.02
    assert abs((r.odom.y + 0.1) - r.pose.y) < 0.02


def test_the_demo_file_drifts_visibly_on_a_straight_drive():
    """config/demo_odom_error.json has to produce what its comment promises the student."""
    eng = driving_engine(world="track", path=config_file("demo_odom_error.json"))
    assert cfg_get(eng.cfg, "odom.geometry.wheel_radius_scale") == 1.05
    for _ in range(int(60.0 / eng.sub_step)):           # until 12 m of lane are driven
        eng.set_cmd_vel(NAME, Twist(vx=0.3))
        eng.step(eng.sub_step)
        if eng.robots[NAME].distance >= 12.0:
            break
    r = eng.robots[NAME]
    drift = math.dist((r.odom.x, r.odom.y), (r.pose.x, r.pose.y))
    assert r.distance > 11.5, f"the lane is shorter than the demo claims: {r.distance}"
    assert 0.5 < drift < 0.7, f"ghost drift {drift:.2f} m on {r.distance:.1f} m — not 5 % of it"


def test_without_odom_geometry_the_numbers_are_the_ones_the_tasks_are_calibrated_on():
    """Default config -> the integrator believes the true geometry (thresholds stay valid)."""
    eng = driving_engine()
    assert cfg_get(eng.cfg, "odom.geometry") == {}
    drive(eng, Twist(vx=0.3), 6.0)
    truth, estimate, theta_truth, theta_odom = measured(eng)
    assert abs(estimate - truth) < 0.02                 # only sensor noise, no model error
    assert abs(yaw_error(theta_odom, theta_truth)) < 0.02


def test_a_test_profile_keeps_the_believed_geometry():
    """set_sensor_profile() rebuilds the odometer; the model error must not be lost on the way."""
    eng = driving_engine({"wheel_radius_scale": RADIUS_ERROR})
    drive(eng, Twist(vx=0.3), 3.0)
    eng.set_sensor_profile({"gps": {"sigma_xy": 0.42}})     # e.g. the profile of a KF task
    r = eng.robots[NAME]
    assert abs(r.odometer.g.r - r.chassis.geometry.r * RADIUS_ERROR) < 1e-9
    # Every profile switch restarts the odometer at the current pose (engine note), so what has to
    # scale is the leg measured *from there*, not the whole drive. One step first, so `r.odom` is
    # a message from the rebuilt sensor and not the last one from the old.
    eng.step(eng.sub_step)
    base_truth, base_odom = (r.pose.x, r.pose.y), (r.odom.x, r.odom.y)
    assert math.dist(base_truth, base_odom) < 0.02      # the reset is to the truth, not to 0
    drive(eng, Twist(vx=0.3), 6.0)
    truth_leg = math.dist(base_truth, (r.pose.x, r.pose.y))
    odom_leg = math.dist(base_odom, (r.odom.x, r.odom.y))
    assert abs(odom_leg / truth_leg - RADIUS_ERROR) < 0.002
