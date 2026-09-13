"""Tests for the Pygame renderer — without agent A: a fake engine from SimpleNamespace.

That way render.py is checked only against types.py and the attributes promised in
the contract (engine.world, engine.robots, engine.t, engine.task). All of it headless.
"""
import contextlib
import math
import statistics
import unittest.mock as mock
import time
from types import SimpleNamespace

import pygame
import pytest

from mecanum_lab import keys, physics, render
from mecanum_lab.types import (MARKERS, Odom, PALETTE, Pose, Rect, Robot, RobotSpec, Scan,
                               Twist, World)
from support_contrast import leucht, ratio

CFG = {"width": 900, "height": 600, "gui_rate": 30,
       "robot": {"lx": 0.14, "ly": 0.13, "r": 0.05, "footprint_r": 0.21},
       "gui_style": {"wall": [0.34, 0.36, 0.42], "floor": [0.13, 0.14, 0.17],
                     "grid": True, "lidar_alpha": 70, "trail_len": 25}}


def make_world(w=6.0, h=4.0):
    return World(name="fake", cell=0.5,
                 walls=[Rect(0, 0, w, 0.2), Rect(0, h - 0.2, w, h), Rect(0, 0, 0.2, h),
                        Rect(w - 0.2, 0, w, h), Rect(2, 1.5, 3, 2.5)],
                 spawns=[Pose(1.0, 1.0, 0.0), Pose(1.0, 2.0, math.pi / 2)],
                 goal=Pose(4.5, 3.0, 0.0), size=(w, h))


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


def _to_world(theta: float, bx: float, by: float) -> tuple:
    """A body-frame direction of a robot at `theta` as a world direction.

    Written out here rather than taken from `render`: the right hand side is the definition of the
    frame the whole project documents — x forward, y to the left, theta counter-clockwise.
    """
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    return bx * cos_t - by * sin_t, bx * sin_t + by * cos_t


def _drawn_at(theta: float, bx: float, by: float) -> tuple:
    """And the same direction as the camera shows it: `px()` maps a world y downwards on the screen.

    This is the contract `render.screen()` has to satisfy, restated independently of it. Without this
    step a test of the drawing can only ever compare a drawing with the transform it was drawn by, and
    passes on a mirrored one.
    """
    wx, wy = _to_world(theta, bx, by)
    return wx, -wy


def _shift(x: float, y: float, theta: float, mount: tuple) -> tuple:
    """Where a body-frame point of a robot at `theta` lands in the world (the camera does the rest)."""
    dx, dy = _to_world(theta, *mount)
    return x + dx, y + dy


def _closest_to(axles: list, point: tuple) -> int:
    """Which wheel a drawn stripe belongs to: the one whose axle it started nearest to."""
    return min(range(len(axles)), key=lambda i: math.dist(axles[i], point))


def press(key):
    """Post a KEYDOWN. pygame.key.key_code knows 'space'/'escape' but not 'plus'."""
    name = {"plus": "+", "minus": "-"}.get(key, key)
    pygame.event.post(pygame.event.Event(pygame.KEYDOWN, key=pygame.key.key_code(name),
                                         mod=0, unicode=name))


def click(button: int, pos) -> None:
    """Post a MOUSEBUTTONDOWN at a screen position — `press` for the other hand."""
    pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=button, pos=tuple(pos)))


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
    with gui(make_engine(world=world)) as rend:
        rend.draw(cap=False)


def test_wheel_phase_follows_sim_time():
    with gui(make_engine({"alice": make_robot(wheels=[6.0, 6.0, 6.0, 6.0])})) as rend:
        rend.draw(cap=False)
        first = list(rend.phase["alice"])
        rend.engine.t += 0.5                            # only the engine tick drives the rollers
        rend.draw(cap=False)
        assert any(abs(b - a) > 1e-6 for a, b in zip(first, rend.phase["alice"]))


def test_every_wheel_is_drawn_at_its_axle_and_not_metres_away():
    """The wheels belong to the robot: no wheel may leave the footprint circle.

    The mount of a wheel used to be built from a pixel number that stood in a metre variable, so
    `px()` put the four tyres 10 m in front of and behind the chassis — on no screen, and the
    picture that made the wheel layer look "broken". The test asks the question a student would
    ask of the frame: is there a wheel where the wheels are?
    """
    lx, ly = CFG["robot"]["lx"], CFG["robot"]["ly"]
    reach = CFG["robot"]["footprint_r"]
    for sx, sy in render.CORNERS:
        assert abs(sx * lx) <= reach and abs(sy * ly) <= reach
    assert all(math.hypot(*m) <= reach for m in render.wheel_mounts(lx, ly))


def test_wheel_pixels_stay_inside_the_chassis_box():
    """Pixel check of the same promise: colour where the tyres are, floor in front of the robot."""
    robot = make_robot("alice", 0, with_scan=False)
    robot.pose = Pose(3.0, 2.0, 0.0)                    # straight ahead, x forward = screen right
    with gui(make_engine({"alice": robot})) as rend:
        rend.show_ghost = rend.show_zones = False
        for _ in range(3):                              # a frame, then the frame to look at
            rend.engine.t += 0.1
            rend.draw(cap=False)
        lx, ly, wr = (CFG["robot"][k] for k in ("lx", "ly", "r"))
        for mx, my in render.wheel_mounts(lx, ly):
            px, py = (int(round(v)) for v in rend.px(robot.pose.x + mx, robot.pose.y + my))
            assert rend.screen.get_at((px, py))[:3] != rend.col_floor, f"no wheel at {mx}, {my}"
        far = tuple(int(round(v)) for v in rend.px(robot.pose.x + 2.0, robot.pose.y))
        assert rend.screen.get_at(far)[:3] == rend.col_floor, "a wheel is drawn 2 m ahead"


def test_a_body_direction_becomes_the_direction_it_is_drawn_as():
    """`render.screen()` has to be the camera's own mapping, and the camera is defined in `px()`.

    The world is right-handed with y to the left; the screen is not, its y grows downwards, and
    `px()` turns a world position into a screen position with exactly one sign change. A *direction*
    needs the same sign change, which the helper that stood here for years left out: it rotated body
    vectors by `-theta`, a reflection composed with a rotation. The two agree on every vector along the
    body x-axis — the long axis of the chassis plate, the velocity arrow, the marker of a robot facing
    forward, and the direction the collision circle is measured in — which is why the difference survived as long as it
    did. The ones it got wrong were the diagonals: the roller axes of the wheels.
    """
    for theta in (0.0, 0.7, math.pi / 2, 2.3, -1.1):
        for vector in ((1, 0), (0, 1), (1, 1), (1, -1), (-1, 1), (0.3, 0.9)):
            world = _to_world(theta, *vector)
            want = (world[0], -world[1])
            got = render.screen(theta, *vector)
            assert got[0] == pytest.approx(want[0], abs=1e-9)
            assert got[1] == pytest.approx(want[1], abs=1e-9), \
                f"body direction {vector} at {math.degrees(theta):+.0f}° drawn at " \
                f"{math.degrees(math.atan2(got[1], got[0])):+.1f}°, belongs at " \
                f"{math.degrees(math.atan2(want[1], want[0])):+.1f}°"
    assert render.screen(0.0, 0, 1)[1] < 0, "left is up the screen for a robot that faces +x"


def test_the_nose_of_a_marker_points_where_that_robot_drives():
    """A marker says "this corner is the front of that robot", so its nose must lie on the heading.

    The marker angles used to be authored as *screen* angles of an unrotated robot, and the tip of the
    triangle sat at -90° — up the screen, which is the robot's left. A robot facing +x therefore wore a
    triangle pointing 90° off its heading while the white heading line beside it pointed forward: two
    heading indicators on one robot, telling two stories, in the picture a student uses to tell the
    robots apart. The shapes are authored in the body frame now (0° = forward, counter-clockwise), so
    every marker that has a corner forward has that corner on the heading at any heading.
    """
    nose_markers = ("triangle", "diamond", "pentagon", "star", "cross")   # the ones with a corner at 0°
    for marker in nose_markers:
        for theta in (0.0, 0.7, math.pi / 2, 2.3, -1.1):
            ahead = render.screen(theta, 1, 0)
            corners = render.shape(marker, (0.0, 0.0), 10.0, theta)
            best = max(corners, key=lambda p: (p[0] * ahead[0] + p[1] * ahead[1]) / math.hypot(*p))
            along = (best[0] * ahead[0] + best[1] * ahead[1]) / math.hypot(*best)
            assert along > 0.99, f"{marker} at {math.degrees(theta):+.0f}°: nose is {along:.2f} ahead"


def test_the_roller_strokes_are_drawn_on_the_free_glide_axis_of_their_own_wheel():
    """Which way the stripes on a tyre lean is the content of a mecanum drawing.

    A wheel is drawn as a rectangle along the direction it is mounted to roll in, with its rollers as
    stripes across the tread. Those stripes are the free-glide axis, and the wheel is driven along the
    direction perpendicular to it — which is what `physics.inverse_kinematics()` projects the body
    velocity onto. The stripes are therefore the one thing in the picture that says which diagonal this
    robot's layout follows, and the renderer's screen helper used to mirror every direction with a y
    component, enough to turn the X of the physics into an O: in the figure a student is meant to learn
    the kinematics from. A rectangle hides a mirror image of its own axes; a stripe does not.

    The stripe is taken from the drawing call rather than from the pixels, deliberately. A tyre is some
    20 px across, its three stripes are 2 px thick, and where they sit along the tread depends on the
    wheel phase and so on the frame time: an axis fitted from such ink sits within a few degrees of the
    truth at best and cannot tell a mirror image from a rounding error. What is checked instead is the
    segment the renderer asks pygame to draw, per wheel, at four headings — with the free-glide axis of
    every wheel derived from the kinematics and nothing copied from `render`. The right drawing and the
    mirrored one are 90° apart, so no tolerance here can blur the two together.
    """
    g = physics.Geometry()
    free_axes = []                                        # one perpendicular per wheel, from physics
    for i in range(4):
        vx = physics.inverse_kinematics(g, 1.0, 0.0, 0.0)[i] * g.r      # this wheel's speed at +vx
        vy = physics.inverse_kinematics(g, 0.0, 1.0, 0.0)[i] * g.r      # ... and at +vy
        assert (vx, vy) != (0.0, 0.0), f"wheel {i} is driven by nothing, so its stripe says nothing"
        free_axes.append((-vy, vx))                       # a quarter turn from the driven direction

    robot = make_robot("alice", 0, with_scan=False)
    stripe = render.mix(render.rgb(robot.spec.rgb), (1, 1, 1), .35)
    calls = []
    real_line = pygame.draw.line

    def spy(surface, color, start, end, width=1, **kw):
        calls.append((tuple(color)[:3], start, end))
        return real_line(surface, color, start, end, width, **kw)

    for theta in (0.0, 0.6, -1.2, 2.6):
        robot.pose = Pose(3.0, 2.0, theta)
        with gui(make_engine({"alice": robot})) as rend:
            axles = [rend.px(*_shift(3.0, 2.0, theta, m)) for m in
                     render.wheel_mounts(CFG["robot"]["lx"], CFG["robot"]["ly"])]
            with mock.patch.object(pygame.draw, "line", spy):
                calls.clear()
                rend.draw(cap=False)
            strokes = [(a, b) for colour, a, b in calls if colour == stripe]
            assert len(strokes) == 4 * render.WHEEL_STROKES, \
                f"{len(strokes)} roller strokes drawn, expected one per wheel per stripe"
            for i, free in enumerate(free_axes):
                want = _drawn_at(theta, *free)       # where the camera puts this wheel's stripes
                mine = [(a, b) for a, b in strokes if _closest_to(axles, a) == i]
                assert len(mine) == render.WHEEL_STROKES, f"wheel {i} owns {len(mine)} stripes"
                for (ax, ay), (bx, by) in mine:
                    gx, gy = bx - ax, by - ay
                    length = math.hypot(gx, gy)
                    assert length > 1, "a roller stroke of no length is drawn as a point"
                    along = (gx * want[0] + gy * want[1]) / (length * math.hypot(*want))
                    assert abs(along) > 0.94, f"wheel {i} at {math.degrees(theta):+.0f}°: roller drawn " \
                        f"at {math.degrees(math.atan2(gy, gx)):+.0f}° on the screen, its free-glide " \
                        f"axis is at {math.degrees(math.atan2(want[1], want[0])):+.0f}°"


def test_roller_axes_match_the_kinematics_that_drive_the_wheels():
    """The drawn rollers are the ones `physics.forward_kinematics()` assumes.

    A mecanum wheel drives only across its roller axis, so the velocity of its centre — with this
    one wheel turning and the robot free — has to stand on that axis. Measured against the module
    that the exercise is graded with, not against a comment: mirror the diagonals (the other
    arrangement, "O") and every dot product here comes out 1 instead of 0.
    """
    g = physics.Geometry()
    for index, (sx, sy) in enumerate(render.CORNERS):
        speeds = [0.0] * 4
        speeds[index] = 1.0
        vx, vy, omega = physics.forward_kinematics(g, speeds)
        point = (vx - omega * sy * g.ly, vy + omega * sx * g.ly)      # body frame, at the wheel
        axis = render.ROLLERS[index]
        both = math.hypot(*point) * math.hypot(*axis)
        assert abs((point[0] * axis[0] + point[1] * axis[1]) / both) < 0.02, index
        assert math.hypot(*point) > 1e-6, "a wheel that drives nothing explains nothing"


def test_a_hud_colour_written_0_1_is_not_the_black_it_looks_like():
    """pygame truncates a 0..1 colour instead of scaling it: `(1, .85, .3)` came out `(1, 0, 0)`.

    Both spaces are in use here — `types.PALETTE` and the literal HUD colours are 0..1, `rgb()` and
    `mix()` hand back 0..255 — so `to255()` is where the two meet, and `_text()` goes through it.
    A black that was meant as black is untouched by the conversion.
    """
    assert render.to255((1, .85, .3)) == (255, 216, 76)             # the amber of the goal
    assert render.to255((1, .92, .70)) == (255, 234, 178)           # the pointer coordinate
    assert render.to255(render.GREY) == render.GREY                 # 0..255 passes through
    assert render.to255((0, 0, 0)) == (0, 0, 0)                     # black on purpose, stays black


def test_the_goal_label_is_readable_on_the_floor():
    """Measured where the letters are, not where the colour claims to be bright.

    The word `goal` stands above its bullseye, on the floor — (33, 35, 43). The truncated amber
    reached 1.3:1 against that floor, which is not a dim label but no label: not one pixel of it
    stood 3:1 out from what it was painted over.
    """
    with gui() as rend:
        rend.draw(cap=False)
        ziel = rend.engine.world.goal
        px, py = rend.px(ziel.x, ziel.y)
        boden = rend.col_floor
        gemalt = [rend.screen.get_at((x, y))[:3]
                  for y in range(int(py) - 32, int(py) - 18)
                  for x in range(int(px) - 12, int(px) + 44)
                  if rend.screen.get_at((x, y))[:3] != tuple(boden[:3])]
        assert len(gemalt) > 20, "not a single pixel of the word 'goal' was drawn"
        beste = max(ratio(c, boden) for c in gemalt)
        assert beste >= 4.5, f"'goal' reaches only {beste:.2f}:1 against the floor"


def test_a_label_gets_a_dark_edge_because_no_robot_colour_survives_a_wall():
    """The map under a label is not its background — the dark edge behind it is.

    A wall is (86, 91, 107) and no colour of `types.PALETTE` reaches 4.5:1 on that: the name of a red
    robot was 1.98:1, and in a hall built out of walls a robot drives along them most of the time. So
    `_text()` darkens the pixels behind the letters before painting the letters — which is invisible
    on floor and void (dark on dark) and rescues every hue on a wall. Both halves are asserted: the
    edge has to be there, and the number has to come from it rather than from the hue.
    """
    rot = tuple(int(255 * v) for v in dict(PALETTE)["red"])
    with gui() as rend:
        for halo, hintergrund in ((False, rend.col_wall), (True, rend.col_void)):
            rend.screen.fill(rend.col_wall)
            breit = rend._text("alice", 60, 40, rot, halo=halo)
            feld = [rend.screen.get_at((x, y))[:3] for y in range(38, 58)
                    for x in range(58, 62 + int(breit))]
            dunkelster, hellster = min(feld, key=leucht), max(feld, key=leucht)
            assert tuple(dunkelster) == tuple(hintergrund[:3]), (
                f"halo={halo}: the darkest pixel is {tuple(dunkelster)}, expected the edge "
                f"{tuple(hintergrund[:3])} — the edge behind a label is what makes it readable")
            kontrast = ratio(hellster, dunkelster)
            if halo:
                assert kontrast >= 4.5, f"red on the edge reaches only {kontrast:.2f}:1"
            else:
                assert kontrast < 3.0, f"red on a wall alone should NOT reach 4.5:1 ({kontrast:.2f})"


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
        assert "alice" not in rend.ghost_trail, "the believed trail belongs to a robot that is gone"


def test_the_ghost_trail_follows_the_believed_pose_and_not_the_true_one():
    """Layer `o` used to show where odometry stands *now*; the drift it is about is a history.

    The same three rules as the truth trail, and for the same reasons: written while hidden (a line
    that starts at the moment the layer is switched on tells the student the error began now), the same
    spacing and cap (so what lies between the two lines is the odometry error and nothing else), and
    pruned with the robot.
    """
    engine = make_engine()
    robot = engine.robots["alice"]
    with gui(engine) as rend:
        for i in range(12):
            robot.pose.x = 1.0 + 0.05 * i                # the truth drives further than the odometer
            robot.odom.x = 1.0 + 0.04 * i
            engine.t += 0.1
            rend.draw(cap=False)
        geglaubt = rend.ghost_trail["alice"]
        assert geglaubt, "no ghost trail at all"
        assert geglaubt[-1] == pytest.approx((robot.odom.x, robot.odom.y)), "followed the truth"
        assert [p[0] for p in geglaubt] == sorted(p[0] for p in geglaubt), "grew backwards"
        assert rend.trails["alice"][-1][0] - geglaubt[-1][0] > 0.1, "both lines on top of each other"

        rend.show_ghost = False                          # hidden: the line stops, the history does not
        punkte = len(geglaubt)
        robot.odom.x += 0.5
        engine.t += 0.1
        rend.draw(cap=False)
        assert len(rend.ghost_trail["alice"]) == punkte + 1, "hidden layer threw the history away"

        for i in range(60):                              # and the cap holds, same as the truth trail
            robot.odom.x = 2.0 + 0.05 * i
            engine.t += 0.1
            rend.draw(cap=False)
        assert len(rend.ghost_trail["alice"]) == rend.trail_len


def test_the_believed_line_is_drawn_only_while_its_layer_is_on():
    """A layer that is switched off must leave no pixel behind — and one that is on must leave its own.

    The colour of the believed line is the robot colour pulled towards the floor, so it is not the
    ghost plate and not the truth trail: counting that exact colour over the screen is the difference
    between "the history is kept" and "something was painted", which the state dict alone cannot tell.
    """
    engine = make_engine()
    robot = engine.robots["alice"]
    with gui(engine) as rend:
        farbe = render.mix(render.rgb(robot.spec.rgb), rend.col_floor, .60)[:3]
        rend.show_ghost = True            # the clean view keeps the raw layers off, this one included
        for i in range(30):
            robot.pose.x = 1.0 + 0.06 * i
            robot.odom.x = 1.0 + 0.05 * i
            engine.t += 0.1
            rend.draw(cap=False)

        def spuren():
            return sum(1 for y in range(0, rend.size[1], 2) for x in range(0, rend.size[0], 2)
                       if rend.screen.get_at((x, y))[:3] == farbe)

        assert spuren() > 20, "the believed line is not drawn with the layer on"
        rend.show_ghost = False
        robot.odom.x += 0.6
        engine.t += 0.1
        rend.draw(cap=False)
        assert spuren() == 0, "the believed line is still painted with the layer off"


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
        was_scan = rend.show_scan
        press("l")
        press("space")
        flags = rend.poll()
        assert flags["toggle_lidar"] and flags["pause"] and flags["key"] == "space"
        assert rend.paused and rend.show_scan != was_scan    # one press, one flip — not a level
        assert rend.poll()["toggle_lidar"] is False          # no edge without a new key press


def test_toggles_and_zoom_and_grid():
    """A layer key flips its layer and nothing else; `+`/`-` are the zoom keys of the help line."""
    with gui() as rend:
        before = {attribute: getattr(rend, attribute) for attribute in keys.LAYER_ATTRIBUTES}
        for key in ("t", "l", "plus", "plus", "minus"):
            press(key)
            rend.poll()
        assert rend.show_trails is not before["show_trails"]
        assert rend.show_scan is not before["show_scan"]
        assert abs(rend.zoom - 1.25) < 1e-9, "two steps in and one out must leave 1.25"
        for key in ("l", "t"):                                # once more: back to where it started
            press(key)
            rend.poll()
        assert {attribute: getattr(rend, attribute) for attribute in before} == before


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


# --------------------------------------------------------------- right button: place a robot


def test_the_right_button_offers_every_robot_for_the_spot_under_the_pointer():
    """Right click opens the menu, a click on a row reports the placement — and moves nothing here.

    The window is a *view*: the assertion that matters most is the last one. Everything a robot is made
    of belongs to the engine, and a renderer that moved robots would be a second place that decides what
    a pose is — while `node.run_loop` is the one place that asks the engine for it.
    """
    bots = {name: make_robot(name, i) for i, name in enumerate(("alice", "bob"))}
    engine = make_engine(bots)
    with gui(engine) as rend:
        spot = (3.0, 3.0)                                   # open floor, between rack and goal
        click(3, rend.px(*spot))
        assert rend.poll()["teleport"] is None, "opening the menu is not placing anything"
        assert rend.pick.open and [row[0] for row in rend.pick.rows] == ["alice", "bob"]
        rend.draw(cap=False)                                # the menu is drawn, chips and all
        assert rend.pick.rect.right <= rend.size[0] and rend.pick.rect.bottom <= rend.size[1], \
            "the menu hangs over the edge of the window — a row off screen is a robot off limits"

        click(1, (rend.pick.rect.x + 10, rend.pick.row_rect(1).y + 5))    # the row of 'bob'
        reported = rend.poll()["teleport"]
        assert reported and reported[0] == "bob", reported
        assert (round(reported[1], 2), round(reported[2], 2)) == spot, "the spot of the right click"
        assert not rend.pick.open, "a menu that answered has nothing to say a second time"
        assert engine.robots["bob"].pose.x == pytest.approx(1.3), \
            "the renderer moved a robot — it is a view, not a second engine"


def test_the_right_button_stays_quiet_where_no_robot_can_stand():
    """On the rack and out in the void the menu does not appear: an offer that can only answer "no"."""
    with gui() as rend:
        for blocked in ((2.5, 2.0), (-1.0, 2.0), (3.0, -1.0)):
            click(3, rend.px(*blocked))
            flags = rend.poll()
            assert not rend.pick.open, f"a menu opened at {blocked}, which is no floor"
            assert flags["teleport"] is None


def test_escape_over_an_open_robot_menu_closes_it_instead_of_ending_the_run():
    """The hand that reaches for esc there has just decided against placing — the window stays open."""
    with gui() as rend:
        click(3, rend.px(4.5, 3.5))
        rend.poll()                                         # the click is worked off in the poll
        assert rend.pick.open
        press("escape")
        flags = rend.poll()
        assert flags["quit"] is False and flags["key"] == "", "esc closed the menu, not the run"
        assert not rend.pick.open
        press("escape")
        assert rend.poll()["quit"] is True, "the second esc is the one that means it"


def test_a_second_right_click_moves_the_menu_instead_of_needing_a_close_click():
    bots = {name: make_robot(name, i) for i, name in enumerate(("alice", "bob"))}
    with gui(make_engine(bots)) as rend:
        click(3, rend.px(1.0, 3.5))
        click(3, rend.px(5.0, 3.5))
        rend.poll()                                         # both clicks, in the order they came
        assert rend.pick.open
        click(1, (rend.pick.rect.x + 10, rend.pick.row_rect(0).y + 5))
        reported = rend.poll()["teleport"]
        assert round(reported[1], 2) == 5.0, "the menu is about the spot it was last opened at"


def test_forget_drops_the_drawn_history_of_one_robot_and_keeps_the_others():
    """After a placement the old part of the hall must not be drawn as if it had been driven."""
    bots = {name: make_robot(name, i) for i, name in enumerate(("alice", "bob"))}
    with gui(make_engine(bots)) as rend:
        for name in bots:
            rend.trails[name] = [(0.0, 0.0), (1.0, 1.0)]
            rend.ghost_trail[name] = [(0.0, 0.0)]
            rend.kf_trail[name] = [(0.0, 0.0)]
            rend.phase[name] = [0.0] * 4
        rend.forget("alice")
        for history in (rend.trails, rend.ghost_trail, rend.kf_trail, rend.phase):
            assert "alice" not in history and "bob" in history


def test_the_believed_line_is_dashed_and_the_true_line_is_not():
    """Which of the two similar lines is the belief must be readable without a legend.

    Counted as holes: the row the collected trail lies on is sampled pixel by pixel and the runs of ink
    are counted — a solid stroke answers 1, the dashed one answers several. Row and span are read back
    from the trail the renderer collected itself, because where that line sits in pixels is the
    renderer's business (and its cap decides how much of it is drawn). The truth line is checked in the
    same run to stay solid: two dashed lines would distinguish nothing, only swap the confusion.
    """
    def runs_of_ink(ghost):
        engine = make_engine()
        robot = engine.robots["alice"]
        with gui(engine) as rend:
            rend.show_ghost, rend.show_trails = ghost, not ghost
            for i in range(40):                       # 4 m straight ahead, pose and odometer together
                robot.pose.x = 1.0 + 0.1 * i
                robot.odom.x = 1.0 + 0.1 * i
                engine.t += 0.1
                rend.draw(cap=False)
            pts = rend.ghost_trail["alice"] if ghost else rend.trails["alice"]
            assert len(pts) > 2, "no trail was collected at all"
            x_lo, x_hi = sorted((rend.px(*pts[0])[0], rend.px(*pts[-1])[0]))
            y = int(round(rend.px(*pts[-1])[1]))
            x1 = int(x_hi - 14)                       # short of the marker drawn on the last pose
            x0 = int(x_lo + 0.6 * (x_hi - x_lo))
            ink = [rend.screen.get_at((x, y))[:3] != rend.col_floor for x in range(x0, x1)]
            return sum(1 for a, b in zip(ink, ink[1:]) if a and not b) + (1 if ink and ink[0] else 0)

    assert runs_of_ink(ghost=True) >= 3, "the believed trail is drawn solid again"
    assert runs_of_ink(ghost=False) == 1, "the truth trail is not the solid line any more"
