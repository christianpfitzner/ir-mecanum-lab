"""Tests for camera and layer menu — headless, fake engine where possible.

The promise under test: the menu changes only what pygame draws. Publishing lives in node.py
and engine.py, so a hidden lidar layer must not remove a single message from the bus.
"""
import math
from types import SimpleNamespace

import pygame
import pytest

from mecanum_lab import cam
from mecanum_lab import render as R
from mecanum_lab.engine import SimEngine
from mecanum_lab.stub import get_bus
from mecanum_lab.types import (MARKERS, Odom, PALETTE, Pose, Rect, Robot, RobotSpec, Scan,
                               Twist, World, load_config)
from mecanum_lab.worlds import load_world

CFG = {"width": 900, "height": 600, "gui_rate": 30,
       "robot": {"lx": 0.14, "ly": 0.13, "r": 0.05, "footprint_r": 0.21},
       "gui_style": {"wall": [0.34, 0.36, 0.42], "floor": [0.13, 0.14, 0.17],
                     "void": [0.06, 0.065, 0.08], "trail_len": 25}}


def make_engine(roboten=None, w=8.0, h=6.0, t=1.0):
    world = World(name="fake", cell=0.5,
                 walls=[Rect(0, 0, w, .2), Rect(0, h - .2, w, h), Rect(0, 0, .2, h),
                        Rect(w - .2, 0, w, h), Rect(3.0, 2.0, 5.0, 3.0)],
                 spawns=[Pose(1.0, 1.0, 0.0), Pose(2.0, 1.0, 0.0)], goal=Pose(4.0, 3.0, 0.0),
                 markings=[(1, 3.5, 5, 3.5)], size=(w, h))
    color, rgb_value = PALETTE[0]
    robots = roboten or {"alice": Robot(
        spec=RobotSpec(name="alice", index=0, color=color, rgb=rgb_value, marker=MARKERS[0],
                       variant="stock"),
        chassis=None, pose=Pose(1.0, 1.0, 0.0), twist=Twist(0.4, 0.0, 0.1),
        wheels=[2.0, -2.0, 2.0, -2.0], mode="pass-through",
        odom=Odom(t=1.0, x=1.0, y=1.0, theta=0.0, vx=0.4),
        scan=Scan(t=1.0, angle_min=0.0, angle_increment=math.tau / 8, range_min=.05,
                  range_max=8.0, ranges=[1.5] * 8),
        contacts=0, distance=1.0, mission_state="running")}
    return SimpleNamespace(world=world, robots=robots, t=t, task="quadrat", drain=lambda: [])


@pytest.fixture
def rend():
    renderer = R.Renderer(make_engine(), CFG)
    try:
        yield renderer
    finally:
        renderer.close()


def click(pos, button=1):
    """Press and release one mouse button at `pos` (the panel reacts to the press)."""
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=pos, button=button,
                                         rel=(0, 0)))
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, pos=pos, button=button,
                                         rel=(0, 0)))


def drag(start, target):
    """Drag with the left button from `start` to `target`."""
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, pos=start, button=1, rel=(0, 0)))
    pygame.event.post(pygame.event.Event(pygame.MOUSEMOTION, pos=target, rel=(0, 0),
                                         buttons=(1, 0, 0)))
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONUP, pos=target, button=1, rel=(0, 0)))


def press_key(name):
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.key.key_code(name),
                                         mod=0, unicode=name))


# ------------------------------------------------------------------------------- camera


def test_camera_shows_the_whole_world_and_never_less_than_the_minimum_scale():
    k = cam.Camera((800, 600), (12.0, 9.0), px_per_meter_min=50)
    assert k.s >= 50 - 1e-9
    assert 0 <= k.px(0, 0)[0] and k.px(12.0, 0.0)[0] <= 800


def test_zoom_at_the_cursor_keeps_exactly_that_spot_on_screen():
    k = cam.Camera((800, 600), (12.0, 9.0), px_per_meter_min=50)
    zeiger = (620, 180)
    world_point = k.wx(*zeiger)
    k.zoom_to_at(zeiger, 1.6)
    nach_dem_zoom = k.px(*world_point)
    assert abs(nach_dem_zoom[0] - zeiger[0]) < 1.0
    assert abs(nach_dem_zoom[1] - zeiger[1]) < 1.0


def test_zoom_stays_inside_its_limits_and_f_resets_to_the_whole_world():
    k = cam.Camera((800, 600), (12.0, 9.0))
    for _ in range(40):
        k.zoom_to(1.25)
    assert k.zoom <= 20.0
    for _ in range(60):
        k.zoom_to(1 / 1.25)
    assert k.zoom >= 0.25
    k.center()
    assert (round(k.cx, 3), round(k.cy, 3)) == (6.0, 4.5)


def test_dragging_moves_the_world_with_the_mouse():
    k = cam.Camera((800, 600), (12.0, 9.0))
    k.zoom_to_at((400, 300), 4.0)
    marker_point = k.wx(250, 200)
    k.pan(250, -100)                                   # mouse goes right and up
    moved = k.px(*marker_point)
    assert abs(moved[0] - 500) < 1.0
    assert abs(moved[1] - 100) < 1.0            # up on screen means smaller y


def test_resize_grows_the_view_and_keeps_the_world_visible():
    k = cam.Camera((800, 600), (12.0, 9.0), px_per_meter_min=20)
    k.resize((1400, 900))
    assert k.size == (1400, 900)
    assert k.px(12.0, 0.0)[0] <= 1400 and k.px(0.0, 9.0)[1] >= 0


# -------------------------------------------------------- renderer uses camera and window


def test_window_resize_event_reaches_the_camera(rend):
    pygame.event.post(pygame.event.Event(pygame.VIDEORESIZE, w=1024, h=640, x=0, y=0))
    flags = rend.poll()
    assert rend.size == (1024, 640) and flags["resized"] == (1024, 640)
    rend.draw(cap=False)
    assert rend.ok


def test_mouse_wheel_zooms_and_drag_pans_in_the_renderer(rend):
    vorher = rend.cam.wx(300, 200)
    pygame.event.post(pygame.event.Event(pygame.MOUSEWHEEL, y=1, x=0, flipped=False))
    pygame.mouse.get_pos = lambda: (300, 200)
    rend.poll()
    assert rend.zoom > 1.0
    nachher = rend.cam.wx(300, 200)
    assert abs(nachher[0] - vorher[0]) < 0.01 and abs(nachher[1] - vorher[1]) < 0.01
    drag((420, 300), (200, 380))
    rend.poll()
    assert rend.cam.s > 0                                # still a usable scale after the drag


def test_void_outside_the_world_is_darker_than_the_floor_inside(rend):
    rend.cam.center()
    rend.draw(cap=False)
    innen = rend.px(1.5, 5.0)                                 # free floor, far from menu and HUD
    assert rend.screen.get_at((int(innen[0]), int(innen[1])))[:3] == rend.col_floor[:3]
    border = rend.px(0.0, 0.0)                                  # left edge of the world box
    aussen = (max(1, int(border[0]) - 8), int(rend.size[1] * .6))
    assert rend.screen.get_at(aussen)[:3] == rend.col_void[:3], \
        f"expected void at {aussen}, world starts at x={border[0]:.0f}"
    assert sum(rend.col_void) < sum(rend.col_floor)


def test_walls_are_filled_without_an_outline_of_another_colour(rend):
    """One wall, one colour: the old 1 px brighten read as a second, thinner wall."""
    rend.cam.center()
    rend.draw(cap=False)
    wand = next(w for w in rend.engine.world.walls
                if w.x0 > 1.0 and w.x1 < rend.engine.world.size[0] - 1.0)   # the inner wall
    proben = [rend.px((wand.x0 + wand.x1) / 2, (wand.y0 + wand.y1) / 2),
              rend.px(wand.x0 + 0.05, (wand.y0 + wand.y1) / 2),
              rend.px(wand.x1 - 0.05, wand.y0 + 0.05)]
    for point in proben:
        assert rend.screen.get_at((int(point[0]), int(point[1])))[:3] == rend.col_wall[:3]


# ---------------------------------------------------------------------------------- menu


def test_the_window_starts_without_the_raw_sensor_values(rend):
    """What a student sees on the first start: the robot and its estimate, not four kinds of noise.

    The measured layers are off because their dots explain nothing before the sensor behind them has
    been interpreted — and because they are the same numbers the readout line prints, the log writes
    and `ros2 topic echo` answers, none of which is switched off by any of this. What stays on is the
    chassis with its wheels, the estimate of experiment 2 and the things a hall contains.
    """
    for name in R.RAW_LAYERS:
        assert getattr(rend, "show_" + name) is False, f"{name} should start hidden"
    for name in ("wheels", "velocity", "goal", "markers", "hud", "kf"):
        assert getattr(rend, "show_" + name) is True, f"{name} belongs to the clean view"


def test_view_state_reads_the_config_and_names_unknown_layers_loudly():
    """`view` in the config: a whole profile at once, or single layers written over it."""
    clean = R.view_state({})
    assert clean["show_scan"] is False and clean["show_kf"] is True
    sensors = R.view_state({"view": {"profile": "sensors"}})
    assert all(sensors["show_" + name] for name in R.LAYER_NAMES), "'sensors' is all of them"
    from support_logging import logged                     # caplog dies once ROS touched logging
    with logged("mecanum.render") as records:
        one = R.view_state({"view": {"profile": "clean",
                                     "layers": {"scan": True, "goal": False, "nonsense": True}}})
    assert one["show_scan"] is True and one["show_goal"] is False, "layers win over the profile"
    assert "nonsense" not in str(one), "an unknown name must not become an attribute"
    assert any("nonsense" in r.getMessage() for r in records), "not swallowed silently"


def test_a_renderer_builds_the_view_the_config_asked_for():
    rend = R.Renderer(SimpleNamespace(world=SimpleNamespace(size=(6, 4), walls=[], markings=[],
                                                            goal=None, name="fake"),
                                     robots={}, t=0.0, task=""),
                      {**CFG, "view": {"profile": "sensors"}})
    try:
        assert all(getattr(rend, "show_" + name) for name in R.LAYER_NAMES)
    finally:
        rend.close()


def test_every_row_of_the_panel_switches_a_real_attribute(rend):
    for attribut, _text, _taste in R.menue.LAYERS:
        assert isinstance(getattr(rend, attribut), bool), attribut


def test_click_on_a_row_switches_only_that_layer(rend):
    rend.menu.open = True                                 # as after pressing m
    rend.draw(cap=False)                                  # lays out the panel rectangle
    before = {attribut: getattr(rend, attribut) for attribut, _t, _k in R.menue.LAYERS}
    row_rect = rend.menu.row_rect(0)                            # first row is the lidar scan
    assert before["show_scan"] is False, "the raw layers start hidden — see view_state()"
    click((row_rect.x + 20, row_rect.y + 8))
    flags = rend.poll()
    assert flags["menu"] == "show_scan"
    assert rend.show_scan is True
    assert {a: getattr(rend, a) for a, _t, _k in R.menue.LAYERS} == {**before,
                                                                     "show_scan": True}


def test_click_next_to_the_panel_does_not_switch_anything(rend):
    rend.menu.open = True
    rend.draw(cap=False)
    before = {attribut: getattr(rend, attribut) for attribut, _t, _k in R.menue.LAYERS}
    click((10, rend.size[1] - 10))
    assert rend.poll()["menu"] == ""
    assert {attribut: getattr(rend, attribut) for attribut in before} == before


def test_the_panel_starts_closed_and_covers_nothing_of_the_map(rend):
    """The goal often sits in the far corner — a panel there would hide the task."""
    assert rend.menu.open is False
    rend.cam.center()
    rend.draw(cap=False)
    assert rend.menu.rect.width < rend.size[0]            # would fit, but is not drawn


def test_m_shows_the_panel_and_hides_it_again(rend):
    assert not rend.menu.open
    press_key("m")
    assert rend.poll()["menu"] == "menu"
    assert rend.menu.open is True
    rend.draw(cap=False)
    press_key("m")
    rend.poll()
    assert rend.menu.open is False
    rend.draw(cap=False)


def test_hiding_every_layer_never_touches_the_engine_or_the_bus():
    """The lab promise: fewer pixels on screen, exactly the same messages on the wire."""
    cfg = load_config(None, {"world": "arena", "gui": True})
    cfg["gui_style"]["trail_len"] = 20
    eng = SimEngine(load_world("arena", cfg=cfg), cfg, seed=3)
    eng.spawn("alice")
    bus = get_bus()
    rend = R.Renderer(eng, cfg)
    try:
        arten = {art for art, _robot in _steps(eng, bus, 40)}
        sensoren = {"odom", "scan", "gps", "imu"}
        assert sensoren <= arten
        pose_vorher = (eng.robots["alice"].pose.x, eng.robots["alice"].pose.y, eng.t)
        for attribut, _text, _taste in R.menue.LAYERS:
            setattr(rend, attribut, False)
        rend.menu.open = False
        rend.draw(cap=False)
        erneut = {art for art, _robot in _steps(eng, bus, 40)}
        assert erneut & sensoren == arten & sensoren, \
            "hiding layers must not stop any sensor from publishing"
        assert bus.last("scan", "alice")[0] is not None
        assert bus.last("gps", "alice")[0] is not None
        assert (eng.robots["alice"].pose.x, eng.robots["alice"].pose.y) != pose_vorher[:2]
    finally:
        rend.close()


def _steps(eng, bus, n):
    """n simulation steps, every measurement onto the bus, one frame per 4th step."""
    gesendet = []
    for i in range(n):
        eng.set_cmd_vel("alice", Twist(0.5, 0.0, 0.2))
        eng.step(0.02)
        for art, roboter, payload in eng.drain():
            bus.pub(art, roboter)(payload)
            gesendet.append((art, roboter))
    return gesendet


def test_q_turns_instead_of_quitting_while_keyboard_driving(rend):
    """q is a driving key in teleop, so it must not close the window there — ESC always quits."""
    for teleop, quits in ((False, True), (True, False)):
        rend.teleop = teleop
        flagen = {"quit": False, "key": "", "camera": None, "menu": ""}
        rend._event_key(SimpleNamespace(key=pygame.K_q), flagen)
        assert flagen["quit"] is quits, f"teleop={teleop}"
        flagen["quit"] = False
        rend._event_key(SimpleNamespace(key=pygame.K_ESCAPE), flagen)
        assert flagen["quit"] is True
