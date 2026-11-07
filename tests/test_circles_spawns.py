"""Tests for what a config adds to the grid: round obstacles, start poses with any heading, a world file.

All three are off by default, and the first test is the one that matters most: without them a hall is the
cached object itself and the scan stream is the one it always was. Used by the lab `ohm-polygon-map`.
"""
import math
import os

import pytest

from mecanum_lab import physics, sensors, worlds
from mecanum_lab.engine import SimEngine
from mecanum_lab.types import Circle, Pose, load_config

BOX = "#" * 20 + "\n" + ("#" + "." * 18 + "#\n") * 12 + "#" * 20 + "\n"      # 9 x 6 m inside


def box(tmp_path):
    path = tmp_path / "box.txt"
    path.write_text(BOX)
    return str(path)


def test_without_circles_or_spawns_the_world_is_the_cached_one():
    world = worlds.load_world("open")
    assert worlds.apply_config(world, load_config()) is world
    assert world.solids() is world.walls


def test_a_world_can_be_a_file_of_its_own(tmp_path):
    world = worlds.load_world(box(tmp_path))
    assert world.name == "box"
    assert world.size == (10.0, 7.0)


def test_a_beam_straight_at_a_circle_measures_centre_distance_minus_radius(tmp_path):
    world = worlds.apply_config(worlds.load_world(box(tmp_path)),
                                {"obstacles": {"circles": [[6.0, 3.5, 0.8]]}})
    lidar = sensors.Lidar(world, sensors.Noise(1), {"beams": 360, "sigma": 0.0, "range_max": 8.0})
    scan = lidar.scan(Pose(2.0, 3.5, 0.0))
    assert scan.ranges[0] == pytest.approx(4.0 - 0.8)
    assert scan.ranges[180] == pytest.approx(2.0 - 0.5)          # the wall behind, inner face at x = 0.5


def test_a_beam_past_the_circle_reaches_the_wall(tmp_path):
    world = worlds.apply_config(worlds.load_world(box(tmp_path)),
                                {"obstacles": {"circles": [[6.0, 3.5, 0.8]]}})
    lidar = sensors.Lidar(world, sensors.Noise(1), {"beams": 360, "sigma": 0.0, "range_max": 8.0})
    beam = 30                                                     # 30 degrees up, misses the circle
    assert lidar.scan(Pose(2.0, 3.5, 0.0)).ranges[beam] == pytest.approx(3.0 / math.sin(math.radians(30)))


def test_a_grazing_beam_on_a_circle_is_lost_like_a_grazing_wall(tmp_path):
    world = worlds.apply_config(worlds.load_world(box(tmp_path)),
                                {"obstacles": {"circles": [[6.0, 3.5, 0.8]]}})
    cfg = {"beams": 3600, "sigma": 0.0, "range_max": 8.0, "reflectivity_min": 0.25}
    scan = sensors.Lidar(world, sensors.Noise(1), cfg).scan(Pose(2.0, 3.5, 0.0))
    edge = math.degrees(math.asin(0.8 / 4.0))                     # the tangent beam
    near_edge = int(round((edge - 0.2) * 10))
    assert math.isinf(scan.ranges[near_edge])
    assert scan.ranges[0] == pytest.approx(3.2)


def test_a_robot_that_stands_gets_the_same_scans_with_the_cache(tmp_path):
    world = worlds.apply_config(worlds.load_world(box(tmp_path)),
                                {"obstacles": {"circles": [[6.0, 3.5, 0.8]]}})
    cfg = {"beams": 360, "sigma": 0.02, "range_max": 8.0}
    cached, fresh = sensors.Lidar(world, sensors.Noise(7), cfg), sensors.Lidar(world, sensors.Noise(7), cfg)
    poses = [Pose(2.0, 3.5, 0.3)] * 3 + [Pose(2.5, 3.0, 0.3)] * 2       # stands, moves, stands
    for pose in poses:
        fresh._cached = (None, None)                                  # casts every ray every time
        assert cached.scan(pose).ranges == fresh.scan(pose).ranges
    assert cached.scan(poses[0]).ranges != cached.scan(poses[0]).ranges   # the noise is new each time


def test_a_robot_is_pushed_out_of_a_circle():
    chassis = physics.Chassis(physics.make_geometry({}, ""), Pose(5.0, 5.0, 0.0))
    chassis.pose = Pose(5.5, 5.0, 0.0)
    chassis._collide([Circle(5.0, 5.0, 0.4)])
    assert math.hypot(chassis.pose.x - 5.0, chassis.pose.y - 5.0) == pytest.approx(0.4 + 0.21)
    assert chassis.contacts == 1


def test_spawns_from_the_config_carry_their_heading_and_survive_a_reset(tmp_path):
    cfg = load_config(overrides={"spawns": [[2.0, 1.5, 20.0]], "gui": False})
    eng = SimEngine(worlds.load_world(box(tmp_path)), cfg, seed=1)
    eng.spawn("muster")
    pose = eng.robots["muster"].chassis.pose
    assert (pose.x, pose.y, pose.theta) == pytest.approx((2.0, 1.5, math.radians(20.0)))
    eng.robots["muster"].chassis.pose = Pose(4.0, 4.0, 0.0)
    assert eng.reset_robot("muster")
    pose = eng.robots["muster"].chassis.pose
    assert (pose.x, pose.y, pose.theta) == pytest.approx((2.0, 1.5, math.radians(20.0)))


@pytest.mark.parametrize("extra, message", [
    ({"obstacles": {"circles": [[20.0, 3.0, 0.5]]}}, "outside the walls"),
    ({"obstacles": {"circles": [[3.0, 3.0, 0.0]]}}, "radius"),
    ({"obstacles": {"circles": [[3.0, 3.0, 0.5]]}, "spawns": [[3.1, 3.0, 0.0]]}, "inside the circle"),
])
def test_a_scenario_nobody_can_solve_is_refused(tmp_path, extra, message):
    with pytest.raises(ValueError, match=message):
        worlds.apply_config(worlds.load_world(box(tmp_path)), extra)


def test_the_default_config_has_no_circles_and_no_spawns():
    cfg = load_config()
    assert cfg["obstacles"] == {"circles": []} and cfg["spawns"] == []
    assert os.path.basename(worlds.load_world("open").name) == "open"
