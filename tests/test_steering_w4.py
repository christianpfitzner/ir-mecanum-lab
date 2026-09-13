"""Tests for the second drive train — `mecanum_lab/steering.py` and its example controller.

What is checked here is a *different kinematic world*, not a worse mecanum robot: this car has one
driven axle, one steered axle and one degree of freedom in its velocity. So the tests ask the
questions a student has to answer about it:

* nothing ever moves sideways — the body-frame lateral component is zero, integrated;
* the radius that comes out is `L / tan(delta)`, measured with a tape and not from the model;
* the rack cannot jump: a step command becomes a ramp of `steer_rate` degrees per second;
* `vy` is dropped with one sentence and the car keeps driving;
* the odometry integrates the steering angle it *believes*, so a wrong end stop returns as yaw drift;
* and the example controller gets its rounded rectangle and its parking spot without a scratch.

Everything runs on the in-process bus, without ROS and without a window:
SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests/test_steering_w4.py -q
"""
import importlib.util
import math
import os

import pytest

from mecanum_lab import node, physics, sensors, steering
from mecanum_lab.engine import SimEngine
from mecanum_lab.stub import StubBus
from mecanum_lab.types import Pose, Rect, Twist, VARIANTS, World, cfg_get, load_config, wrap_angle
from mecanum_lab.worlds import load_world

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
EXAMPLE = os.path.join(ROOT, "student", "steering_example.py")
DT = 1.0 / 50.0


def open_world(size: float = 600.0) -> World:
    """A world without walls: a 1 m car driving 6 m circles needs more floor than `arena` has."""
    return World(name="open", cell=1.0, walls=[], spawns=[Pose(size / 2, size / 2, 0.0)],
                 size=(size, size))


def walled_world() -> World:
    """The same floor, but with a wall 5 m in front of the spawn, facing east."""
    mid = 300.0
    return World(name="box", cell=1.0, walls=[Rect(mid + 5.0, mid - 20.0, mid + 10.0, mid + 20.0)],
                 spawns=[Pose(mid, mid, 0.0)], size=(2 * mid, 2 * mid))


def car_engine(extra: dict | None = None, world: World | None = None, variant="steering"):
    """One simulation with one robot of the given variant, spawned at the world's spawn pose."""
    cfg = load_config(None, {"gui": False, "rate": 50, **(extra or {})})
    eng = SimEngine(world or open_world(), cfg, seed=1)
    eng.spawn("car", variant)
    return eng, eng.robots["car"]


def drive(eng, seconds, vx, omega=0.0, vy=0.0) -> tuple:
    """Drive one command for `seconds`; return both heading changes, unwrapped.

    Unwrapped on purpose: a test that drives 40 s at 30 degrees ends up several circles further
    round, and comparing two `wrap_angle` values would call that a match.
    """
    chassis, odometer = eng.robots["car"].chassis, eng.robots["car"].odometer
    yaw, odo_yaw = 0.0, 0.0
    was, was_odo = chassis.pose.theta, odometer.pose.theta
    for _ in range(round(seconds / DT)):
        eng.set_cmd_vel("car", Twist(vx=vx, vy=vy, omega=omega))    # cmd_vel ages after 0.35 s
        eng.step(DT)
        now, now_odo = chassis.pose.theta, odometer.pose.theta
        yaw += math.atan2(math.sin(now - was), math.cos(now - was))
        odo_yaw += math.atan2(math.sin(now_odo - was_odo), math.cos(now_odo - was_odo))
        was, was_odo = now, now_odo
    return yaw, odo_yaw


def quiet_car(steer_max_scale: float = 1.0):
    """A car whose sensors are honest except for the steering angle its own model believes."""
    return car_engine({"odom": {"steer_max_scale": steer_max_scale, "sigma_wheel": 0.0,
                                "sigma_xy": 0.0, "sigma_theta": 0.0, "bias_omega": 0.0}})


def measured_radius(eng, degrees, v=0.5, settle=4.0) -> float:
    """Geometric radius of a steady turn: the chord of a half turn is twice the radius.

    Not v/omega — that would compare the model with itself. The half turn is measured between two
    poses in the world, so what meets `L / tan(delta)` is what a tape measure would find.
    """
    omega = v * math.tan(math.radians(degrees))
    drive(eng, settle, v, omega)                                   # settle the rack and the drive
    x0, y0 = eng.robots["car"].chassis.pose.x, eng.robots["car"].chassis.pose.y
    turned, prev = 0.0, eng.robots["car"].chassis.pose.theta
    for _ in range(6000):                                          # 120 s is enough for a half turn
        eng.set_cmd_vel("car", Twist(vx=v, omega=omega))
        eng.step(DT)
        now = eng.robots["car"].chassis.pose.theta
        turned += math.atan2(math.sin(now - prev), math.cos(now - prev))
        prev = now
        if turned >= math.pi:
            break
    return math.dist((x0, y0), (eng.robots["car"].chassis.pose.x, eng.robots["car"].chassis.pose.y)) / 2


# ------------------------------------------------------------------ the two drive trains apart


def test_the_variant_name_picks_its_own_drive_train():
    """`steering` is a second world, and only `steering`: without --variant nothing changes."""
    assert steering.is_steering("steering") and steering.is_steering("steering-big")
    assert not steering.is_steering("") and not steering.is_steering("stock")
    assert not any(steering.is_steering(name) for name in VARIANTS)   # the auto cycle stays mecanum
    eng, car = car_engine()
    eng.spawn("mec", "")
    assert isinstance(car.chassis, steering.Chassis)
    assert isinstance(car.odometer, steering.Odometry)
    assert isinstance(eng.robots["mec"].chassis, physics.Chassis)
    assert isinstance(eng.robots["mec"].odometer, sensors.OdometrySensor)
    assert not isinstance(eng.robots["mec"].odometer, steering.Odometry)


def test_the_geometry_is_the_config_with_the_angles_in_degrees():
    """The config speaks degrees, the model works in radians, R_min follows from both."""
    cfg = cfg_get(load_config(), "steering")
    g = steering.make_geometry(cfg, "steering")
    assert g.wheel_base == pytest.approx(1.0) and g.track == pytest.approx(0.62)
    assert g.steer_max == pytest.approx(math.radians(32.0))
    assert g.steer_rate == pytest.approx(math.radians(60.0))
    assert g.r_min == pytest.approx(1.0 / math.tan(math.radians(32.0)), abs=1e-4)      # 1.60 m
    big = steering.make_geometry(cfg, "steering-big")
    assert big.wheel_base > g.wheel_base and big.name == "steering-big"
    assert steering.make_geometry(cfg, "steering-unknown").wheel_base == g.wheel_base
    assert steering.make_geometry({}, "steering").r == pytest.approx(0.05)              # empty cfg
    assert g.lx == pytest.approx(g.wheel_base / 2) and g.ly == pytest.approx(g.track / 2)


def car_chassis_geom(car):
    """The view sizes the chassis from `chassis.geom` — an alias that has to be the geometry."""
    return car.chassis.geom is car.chassis.geometry


def test_the_view_sees_the_real_geometry_of_either_chassis():
    """`render._geom()` reads lx/ly/r/footprint_r from the chassis, so the car is drawn as a car."""
    eng, car = car_engine()
    eng.spawn("mec", "stock")
    assert car_chassis_geom(car) and car_chassis_geom(eng.robots["mec"])
    g = car.chassis.geom
    assert (g.lx, g.ly, g.r, g.footprint_r) == (pytest.approx(0.5), pytest.approx(0.31),
                                                pytest.approx(0.05), pytest.approx(0.62))


# ------------------------------------------------------------------------- no sideways motion


def test_the_body_never_moves_sideways():
    """vy is dropped, so the chord of every step must point along the car's own heading.

    Checked against the mid-step heading: a car on an arc covers a chord, and a chord is turned by
    half the yaw of that step — that is geometry, not a sideways slide. What has to be zero is the
    body-frame lateral component that remains.
    """
    eng, car = car_engine()
    worst_chord, worst_report = 0.0, 0.0
    for i in range(600):
        vx = 0.6 * math.sin(i / 37.0)
        omega = 0.9 * math.sin(i / 23.0) * math.cos(i / 11.0)      # no vy in the command at all
        th0, x0, y0 = car.chassis.pose.theta, car.chassis.pose.x, car.chassis.pose.y
        eng.set_cmd_vel("car", Twist(vx=vx, omega=omega))
        eng.step(DT)
        w, dx, dy = car.chassis.twist.omega, car.chassis.pose.x - x0, car.chassis.pose.y - y0
        chord = math.hypot(dx, dy)
        if chord > 1e-9:
            worst_chord = max(worst_chord, chord * abs(math.sin(math.atan2(dy, dx)
                                                                - (th0 + 0.5 * w * DT))))
        worst_report = max(worst_report, abs(car.chassis.twist.vy), abs(car.odom.vy))
    assert worst_report == 0.0                                      # not "small": exactly zero
    assert worst_chord < 1e-9                                       # no side slip anywhere


def test_a_straight_car_stays_on_its_straight_line():
    eng, car = car_engine()
    drive(eng, 6.0, 0.5)
    assert abs(car.chassis.pose.y - 300.0) < 1e-12                  # no lateral drift in 3 metres
    assert car.chassis.pose.x - 300.0 == pytest.approx(0.5 * 6.0, rel=0.05)


# --------------------------------------------------------------------------- the turning radius


@pytest.mark.parametrize("degrees", [14.0, 22.0, 30.0])
def test_the_turning_radius_is_l_over_tan_delta(degrees):
    """Within 5 % of the one formula this car is built on."""
    eng, car = car_engine()
    expected = car.chassis.geometry.wheel_base / math.tan(math.radians(degrees))
    assert measured_radius(eng, degrees) == pytest.approx(expected, rel=0.05)


def test_full_lock_is_the_minimum_turning_radius():
    """Demand 45 degrees and the rack stops at 32: the radius then is exactly R_min, no less."""
    eng, car = car_engine()
    assert measured_radius(eng, 45.0) == pytest.approx(car.chassis.geometry.r_min, rel=0.05)


# --------------------------------------------------------------------- the rack cannot teleport


def test_a_step_command_becomes_a_ramp():
    """Step response of the steering: steer_rate degrees per second, and not one step faster."""
    g = steering.make_geometry(cfg_get(load_config(), "steering"), "steering")
    eng, car = car_engine()
    per_step = g.steer_rate * DT                                    # 1.2 deg at 60 deg/s and 50 Hz
    demanded = g.steer_max + 0.2                                    # past the stop, on purpose
    omega = 0.5 * math.tan(demanded)
    steps, reached = 0, False
    while steps < 200:
        previous = car.chassis.delta
        eng.set_cmd_vel("car", Twist(vx=0.5, omega=omega))
        eng.step(DT)
        steps += 1
        assert abs(car.chassis.delta - previous) <= per_step * 1.001     # never faster than this
        if abs(car.chassis.delta) >= g.steer_max - 1e-9:
            reached = True
            break
    assert reached and car.chassis.delta == pytest.approx(g.steer_max)   # clamped at the stop
    assert steps == pytest.approx(g.steer_max / per_step, abs=1.5)       # 16 steps, not 1


def test_the_drive_speed_is_limited_too():
    eng, car = car_engine()
    drive(eng, 20.0, 5.0)                                           # commanded far above v_max
    assert car.chassis.v == pytest.approx(car.chassis.geometry.v_max, abs=1e-3)


# ------------------------------------------------------------------------------- vy is impossible


def test_strafe_is_dropped_once_and_the_car_keeps_driving(caplog):
    """"steering robot cannot strafe, vy=0.25 dropped" — said once, and then it drives anyway."""
    eng, car = car_engine()
    with caplog.at_level("WARNING", logger="mecanum.steering"):
        turned, _ = drive(eng, 4.0, 0.5, 0.0, vy=0.25)
    said = [rec.getMessage() for rec in caplog.records if "strafe" in rec.getMessage()]
    assert len(said) == 1, said                                     # one sentence, not 200
    assert "vy=0.25" in said[0]
    assert turned == pytest.approx(0.0)                             # straight ahead, not sideways
    assert car.chassis.pose.x > 300.0 + 1.0                         # and driving, not standing still
    assert car.chassis.pose.y == pytest.approx(300.0, abs=1e-9)     # never moved in y


def test_a_wheel_speed_command_is_one_drive_speed():
    """Four wheel speeds cannot steer this car: their mean becomes v, the rack stays put.

    This is the documented answer to a student who sends lab 1's inverse kinematics to a steering
    robot — a straight line at the average of the four instead of an unexplained motion.
    """
    eng, car = car_engine()
    g = car.chassis.geometry
    for _ in range(round(6.0 / DT)):
        eng.set_wheel_speeds("car", [12.0, 8.0, 12.0, 8.0])         # a mecanum turn command
        eng.step(DT)
    assert car.chassis.v_cmd == pytest.approx(g.r * 10.0)           # the mean, in m/s
    assert car.chassis.delta == pytest.approx(0.0)                  # nothing steered
    assert car.chassis.twist.vy == 0.0


# ------------------------------------------------------------------------- Ackermann geometry


def test_the_inner_front_wheel_turns_further():
    """Both front wheels turn about one point on the rear axle line, so they cannot be equal."""
    g = steering.make_geometry(cfg_get(load_config(), "steering"), "steering")
    assert steering.ackermann(g, 0.0) == [0.0, 0.0]
    left, right = steering.ackermann(g, g.steer_max)                # a left turn: left is inside
    assert left > g.steer_max > right > 0.0
    assert left == pytest.approx(-steering.ackermann(g, -g.steer_max)[1])      # mirrored
    half = g.track / 2
    assert math.tan(left) == pytest.approx(g.wheel_base / (g.r_min - half), abs=1e-9)
    assert math.tan(right) == pytest.approx(g.wheel_base / (g.r_min + half), abs=1e-9)


def test_the_four_wheel_speeds_are_the_bicycle_state_seen_from_the_four_wheels():
    """Straight: all four equal. In a turn: the outer wheel is fastest, the rear mean is exactly v."""
    g = steering.make_geometry(cfg_get(load_config(), "steering"), "steering")
    v = 0.5
    assert steering.wheel_speeds(g, v, 0.0) == pytest.approx([v / g.r] * 4)
    fl, fr, hl, hr = steering.wheel_speeds(g, v, g.steer_max)       # labels: VL, VR, HL, HR
    assert fr > fl > 0.0 and hr > hl > 0.0                          # outer faster than inner
    assert fr == pytest.approx(max([fl, fr, hl, hr]))               # outer front is fastest of all
    assert (hl + hr) / 2.0 == pytest.approx(v / g.r)                # the rear axle mean *is* v
    assert fl > hl                                                  # front axle, bigger circle


def test_the_engine_reports_the_steering_angles_apart_from_the_wheel_speeds():
    """/sim/robots keeps `wheels` a list of four speeds; the angles are the extra `steer_deg`."""
    eng, car = car_engine()
    drive(eng, 3.0, 0.5, 0.25)
    info = next(r for r in eng.robots_info() if r["name"] == "car")
    assert len(info["wheels"]) == 4 and len(info["steer_deg"]) == 2
    assert info["steer_deg"][0] > info["steer_deg"][1] > 0.0
    eng.spawn("mec", "stock")
    assert next(r for r in eng.robots_info() if r["name"] == "mec")["steer_deg"] == []
    assert car.chassis.wheel_headings == pytest.approx(
        [car.chassis.steer[0], car.chassis.steer[1], 0.0, 0.0])     # what the view turns the wheels by


# ---------------------------------------------------------------------------- the lying odometry


def test_with_the_right_numbers_the_odometry_is_exact():
    """An odometer that knows its car integrates the same arc: yaw and metres both come out right."""
    eng, car = quiet_car()
    turned, odo_turned = drive(eng, 40.0, 0.5, 0.5 * math.tan(math.radians(30.0)))
    assert abs(wrap_angle(turned)) > 1.0                            # several circles, sanity only
    assert turned - odo_turned == pytest.approx(0.0, abs=1e-9)      # the same yaw, step by step
    assert car.odom.x == pytest.approx(car.pose.x, abs=1e-6)        # and the same place
    assert car.odom.y == pytest.approx(car.pose.y, abs=1e-6)


def test_an_end_stop_10_percent_off_returns_as_yaw_drift():
    """`odom.steer_max_scale` is the model error a steering car cannot notice: no encoder, no check.

    The integrator replays the angle the rack was *told* to take, so believing 35.2 degrees where the
    car can do 32 makes every corner turn too far. That error is systematic — over a full circle at
    full lock it is tens of degrees of yaw and metres of place, and no amount of driving averages it
    out. The same command with the honest model is exact, which is what makes this a number.
    """
    honest, believing = [], []
    for scale, store in ((1.0, honest), (1.1, believing)):
        eng, car = quiet_car(scale)
        store.append(drive(eng, 20.0, 0.5, 0.5 * math.tan(math.radians(45.0))))   # full lock
        store.append(math.hypot(car.pose.x - car.odom.x, car.pose.y - car.odom.y))
    assert honest[0][0] - honest[0][1] == pytest.approx(0.0, abs=1e-9)
    assert believing[0][0] - believing[0][1] < -0.5                 # 30 deg of yaw too much
    assert believing[1] > 1.0                                       # and metres away from the truth


# ------------------------------------------------------------------- wheel against an obstacle


def test_against_a_wall_the_body_stops_and_the_wheels_do_not():
    """The mecanum slip rule, inherited: the pose stands still, the encoders keep lying."""
    eng, car = car_engine(world=walled_world())
    drive(eng, 30.0, 0.6)
    assert car.contacts == 1
    assert car.pose.x == pytest.approx(305.0 - car.chassis.geometry.footprint_r, abs=0.02)
    assert max(abs(w) for w in car.wheels) > 1.0                    # the wheels are still running
    assert car.odom.x > car.pose.x + 1.0                            # the odometer believes otherwise
    assert car.twist.vx == 0.0                                      # and the truth says: standing


def test_with_static_friction_the_odometry_stays_honest():
    eng, car = car_engine({"steering": {"slip": 0.0}}, world=walled_world())
    drive(eng, 30.0, 0.6)
    assert max(abs(w) for w in car.wheels) == pytest.approx(0.0)
    assert car.odom.x == pytest.approx(car.pose.x, abs=0.05)


# --------------------------------------------------------------------- the example controller


def example_module():
    """Load student/steering_example.py the way robot_io.serve() does — as a file, not a package."""
    spec = importlib.util.spec_from_file_location("steering_example_under_test", EXAMPLE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_example(monkeypatch, seconds: float = 60.0):
    """The whole route on the in-process bus, as fast as the CPU allows (fixed physics step).

    The same `--seconds 60` budget as the brief's command, only sooner: `--fixed-step` plus
    MECANUM_FAST leaves the wall clock out of the loop while the physics step stays 1/50 s, so this
    is the same measurement series as the real-time run.
    """
    monkeypatch.setenv("MECANUM_FAST", "1")
    cfg = load_config(None, {"gui": False, "world": "production"})
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=1)
    bus = StubBus()
    eng.spawn("car", "steering")
    node.subscribe(bus, eng, "car")
    node.wire_task(bus, eng, robot="car")
    track = []
    bus.sub("odom", "car", lambda o: track.append((o.x, o.y)))
    node.add_node(EXAMPLE, "car", bus)
    node.run_loop(eng, bus, None, (), seconds, hz=60.0, fixed_step=True)
    return eng, eng.robots["car"], track


def test_the_example_is_a_drive_and_not_a_task():
    """`serve()` runs drive(rob) with no task given; a steering car has no kinematics to pass on."""
    module = example_module()
    assert callable(getattr(module, "drive", None))
    assert not hasattr(module, "inverse_kinematics")


def test_the_example_rounds_the_rectangle_and_parks_without_a_scratch(monkeypatch):
    eng, car, track = run_example(monkeypatch)
    assert car.mission_state == "done", car.mission_state           # the route ended by itself, and
    assert eng.t == pytest.approx(60.0, abs=0.1)                    # in the middle of these 60 s
    assert car.contacts == 0, car.contacts
    assert car.distance > 15.0                                      # it really drove the lap
    assert min(p[0] for p in track) < 2.25 - 0.5                    # west of the spawn: it turned
    assert max(p[1] for p in track) > 2.75 + 3.0                    # and north: it rounded the box
    assert car.pose.x == pytest.approx(5.8, abs=0.6)                # the loading spot, in front of
    assert car.pose.y == pytest.approx(7.0, abs=0.6)                # the shelf at y = 8 m
    assert car.odom.vy == 0.0                                       # and never one metre sideways
