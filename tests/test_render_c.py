"""Tests for the Pygame renderer — without agent A: a fake engine from SimpleNamespace.

That way render.py is checked only against types.py and the attributes promised in
the contract (engine.world, engine.robots, engine.t, engine.task). All of it headless.
"""
import contextlib
import math
import statistics
import time
from types import SimpleNamespace

import pygame
import pytest

from mecanum_lab import render
from mecanum_lab.types import (MARKERS, Odom, PALETTE, Pose, Rect, Robot, RobotSpec, Scan,
                               Twist, World)

CFG = {"width": 900, "height": 600, "gui_rate": 30,
       "robot": {"lx": 0.14, "ly": 0.13, "r": 0.05, "footprint_r": 0.21},
       "gui_style": {"wall": [0.34, 0.36, 0.42], "floor": [0.13, 0.14, 0.17],
                     "grid": True, "lidar_alpha": 70, "trail_len": 25}}


def make_world(w=6.0, h=4.0):
    return World(name="fake", cell=0.5,
                 walls=[Rect(0, 0, w, 0.2), Rect(0, h - 0.2, w, h), Rect(0, 0, 0.2, h),
                        Rect(w - 0.2, 0, w, h), Rect(2, 1.5, 3, 2.5)],
                 spawns=[Pose(1.0, 1.0, 0.0), Pose(1.0, 2.0, math.pi / 2)],
                 goal=Pose(4.5, 3.0, 0.0), markings=[(1, 3.5, 5, 3.5)], size=(w, h))


def make_scan(beams=360, value=1.2):
    ranges = [value if i % 7 else float("inf") for i in range(beams)]
    ranges[3] = float("nan")                            # must not escape the ranges
    return Scan(angle_min=0.0, angle_increment=2 * math.pi / beams, range_max=8.0,
                ranges=ranges)


def make_robot(name="alice", index=0, with_scan=True, with_odom=True, wheels=None):
    color, rgb = PALETTE[index % len(PALETTE)]
    return Robot(spec=RobotSpec(name=name, index=index, color=color, rgb=rgb,
                                marker=MARKERS[index % len(MARKERS)], variant="stock"),
                 chassis=None, pose=Pose(1.0 + 0.3 * index, 1.5, 0.4 * index),
                 twist=Twist(0.4, -0.1, 0.2),
                 wheels=wheels if wheels is not None else [3.0, -3.0, 3.0, -3.0],
                 mode=["pass-through", "wheels"][index % 2],
                 odom=None if not with_odom else Odom(x=1.0, y=1.5, theta=0.4, vx=0.4),
                 scan=None if not with_scan else make_scan(),
                 contacts=index % 3, distance=1.5 * index, mission_state="running")


def make_engine(robots=None, world=None, task="kinematik"):
    robots = robots if robots is not None else {"alice": make_robot()}
    return SimpleNamespace(world=world or make_world(), robots=robots, t=3.25,
                           task=task, drain=lambda: [])


@contextlib.contextmanager
def gui(engine=None, cfg=None):
    """Renderer; events are posted only after the constructor, which drains the queue."""
    rend = render.Renderer(engine or make_engine(), cfg or CFG)
    try:
        yield rend
    finally:
        rend.close()


def press(key):
    """Post a KEYDOWN. pygame.key.key_code knows 'space'/'escape' but not 'plus'."""
    name = {"plus": "+", "minus": "-"}.get(key, key)
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.key.key_code(name),
                                         mod=0, unicode=name))


# ------------------------------------------------------------------------ drawing


def test_draw_one_robot_keeps_engine_untouched():
    engine = make_engine()
    before = (engine.t, engine.robots["alice"].pose.x, len(engine.robots["alice"].scan.ranges))
    with gui(engine) as rend:
        assert rend.ok
        rend.draw(cap=False)
        rend.draw(cap=False)
    assert (engine.t, engine.robots["alice"].pose.x,
            len(engine.robots["alice"].scan.ranges)) == before


def test_draw_without_robots_and_without_task():
    with gui(make_engine(robots={}, task="")) as rend:
        rend.draw(cap=False)


def test_draw_eight_robots_two_hud_columns():
    bots = {f"robot{i}": make_robot(f"robot{i}", i) for i in range(8)}
    with gui(make_engine(bots)) as rend:
        for _ in range(3):
            rend.draw(cap=False)


def test_draw_handles_missing_sensors_and_empty_wheels():
    robot = make_robot("nackt", 3, with_scan=False, with_odom=False, wheels=[])
    robot.twist = Twist()
    with gui(make_engine({"nackt": robot})) as rend:
        rend.draw(cap=False)


@pytest.mark.parametrize("marker", MARKERS + ["unbekannt"])
def test_every_marker_shape_draws(marker):
    robot = make_robot("m", 0)
    robot.spec.marker = marker
    with gui(make_engine({"m": robot})) as rend:
        rend.draw(cap=False)


def test_degenerate_world_size_does_not_crash():
    world = make_world()
    world.size = (0.0, 0.0)
    world.goal = None
    world.markings, world.spawns = [], []
    with gui(make_engine(world=world)) as rend:
        rend.draw(cap=False)


def test_wheel_phase_follows_sim_time():
    with gui(make_engine({"alice": make_robot(wheels=[6.0, 6.0, 6.0, 6.0])})) as rend:
        rend.draw(cap=False)
        first = list(rend.phase["alice"])
        rend.engine.t += 0.5                            # only the engine tick drives the rollers
        rend.draw(cap=False)
        assert any(abs(b - a) > 1e-6 for a, b in zip(first, rend.phase["alice"]))


def test_trail_is_capped_at_configured_length():
    engine = make_engine()
    robot = engine.robots["alice"]
    with gui(engine) as rend:
        for i in range(80):
            robot.pose.x = 1.0 + 0.05 * i                 # far more points than trail_len
            engine.t += 0.1
            rend.draw(cap=False)
        assert len(rend.trails["alice"]) <= rend.trail_len


def test_trail_of_removed_robot_is_dropped():
    engine = make_engine()
    with gui(engine) as rend:
        rend.draw(cap=False)
        engine.robots.clear()
        engine.t += 0.1
        rend.draw(cap=False)
        assert "alice" not in rend.trails and "alice" not in rend.phase


def test_frame_time_for_eight_robots_with_lidar():
    bots = {f"robot{i}": make_robot(f"robot{i}", i) for i in range(8)}
    engine = make_engine(bots)
    with gui(engine) as rend:
        rend.draw(cap=False)                            # first frame warms up fonts and paths
        times = []
        for i in range(60):
            for robot in bots.values():
                robot.pose.x += 0.01
            engine.t += 0.033
            start = time.perf_counter()
            rend.draw(cap=False)
            times.append(time.perf_counter() - start)
    median = statistics.median(times) * 1000
    assert median < 30, f"frame time {median:.1f} ms over the 30 ms budget"


# ------------------------------------------------------------------- keyboard/state


def test_poll_reports_each_key_as_edge_only():
    with gui() as rend:
        press("l")
        press("space")
        flags = rend.poll()
        assert flags["toggle_lidar"] and flags["pause"] and flags["key"] == "space"
        assert rend.paused and not rend.show_scan
        assert rend.poll()["toggle_lidar"] is False     # no edge without a new key press


def test_toggles_and_zoom_and_grid():
    with gui() as rend:
        for key in ("t", "l", "plus", "plus", "minus"):
            press(key)
            rend.poll()
        assert not rend.show_trails and not rend.show_scan
        assert abs(rend.zoom - 1.25) < 1e-9
        press("l")
        press("t")
        rend.poll()
        assert rend.show_scan and rend.show_trails


def test_camera_keys_focus_and_reset():
    bots = {f"robot{i}": make_robot(f"robot{i}", i) for i in range(3)}
    engine = make_engine(bots)
    with gui(engine) as rend:
        press("3")
        assert rend.poll()["camera"] == 2 and rend.focus == 2
        press("0")
        assert rend.poll()["camera"] == 0 and rend.focus is None
        rend.focus = 1                                  # focus must not leave the world
        rend._cam()
        assert 0 <= rend.ox + rend.s * engine.world.size[0] <= rend.size[0]


def test_quit_key_clears_ok_and_close_is_idempotent():
    rend = render.Renderer(make_engine(), CFG)
    assert rend.ok is True
    press("escape")
    assert rend.poll()["quit"] is True
    assert rend.ok is False
    rend.close()
    rend.close()                                        # a second close is allowed
    assert rend.ok is False


def test_window_quit_event_ends_ok():
    rend = render.Renderer(make_engine(), CFG)
    pygame.event.post(pygame.event.Event(pygame.QUIT))
    assert rend.poll()["quit"] is True and rend.ok is False
    rend.close()


def test_caption_names_the_world():
    with gui(make_engine({"a": make_robot("a", 0), "b": make_robot("b", 1)})) as rend:
        rend.draw(cap=False)
        assert "fake" in pygame.display.get_caption()[0]
