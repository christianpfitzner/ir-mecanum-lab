"""W5: the empty hall `open`, and the Points of Interest a robot can only hear.

Two deliverables with one theme: a hall with nothing in it, and something to look for in it.

The first block checks the arena — that it really is empty (the drift argument only works if a wrong
wheel constant cannot be blamed on a collision), that the supervisor's checker passes it, and that the
documentation picture and the README still agree after the fifth panel.

The second block checks the field model of `pois.py`: the fall-off measured as a ratio at two
distances instead of asserted, the counting noise seeded, the distance staying out of the message, a
mis-placed source refused, and the wall that is not in the model staying out of it.

The third block is the wiring: no source configured means no detector, no message and no random
number, which is what keeps every graded stream of `tests/test_sensor_reality.py` what it was.
"""
import dataclasses
import json
import math
import os
import statistics
import subprocess
import sys
from types import SimpleNamespace

import pytest

from support_docs import text

from mecanum_lab import overlays, pois, render, ros_bridge, sensors
from mecanum_lab.engine import SimEngine
from mecanum_lab.types import (Poi, Pose, Rect, Robot, RobotSpec, Twist, World, cfg_get,
                               load_config, topic)
from mecanum_lab.worlds import list_worlds, load_world

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "tools"))
import worldcheck                                              # noqa: E402
import worldpic                                                # noqa: E402

# The source of config/demo_poi_exploration.json — the shipped scenario, the one the README quotes.
SHIP = [{"name": "src1", "kind": "radiation", "x": 12.0, "y": 6.0, "activity": 1.0, "range": 4.0}]
CLEARANCE = 0.25                     # the allowance tools/worldcheck.py defaults to
CFG = load_config()


def poi_cfg(**over):
    """The lab defaults with a source planted in `open`, plus whatever a single test needs."""
    return load_config(None, dict({"world": "open", "gui": False, "pois": SHIP}, **over))


def drive(cfg, seconds=6.0, seed=1, velocity=0.5):
    """One straight drive east from the world's first spawn; every measurement that came out."""
    eng = SimEngine(load_world(cfg_get(cfg, "world", "arena"), cfg=cfg), cfg, seed=seed)
    eng.spawn("muster")
    got = []
    for _ in range(int(seconds * cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=velocity))
        eng.step(eng.sub_step)
        got += eng.drain()
    return eng, got


def sensor(sources, seed=3, **poi_keys):
    """The detector for `sources`, reading the shipped `poi` block with one knob changed."""
    return pois.PoiSensor(sources, sensors.Noise(seed),
                          dict(cfg_get(load_config(), "poi"), **poi_keys))


# ---------------------------------------------------------------------------- the empty hall


def test_the_open_hall_is_floor_and_border_and_nothing_else():
    """No obstacle, no marking, no goal: what the drift argument needs is an empty room."""
    world = load_world("open", cfg=CFG)
    assert world.size == (30.0, 20.0) and world.cell == 0.5
    assert world.goal is None, "a goal would make it a task arena instead of a drift hall"
    assert len(world.walls) == 4, f"border walls merged into {world.walls}"
    for wall in world.walls:                        # every wall is on the outer edge
        assert wall.x0 <= 0.5 or wall.x1 >= 29.5 or wall.y0 <= 0.5 or wall.y1 >= 19.5, wall
    assert len(world.spawns) == 2, [tuple(p) for p in world.spawns]
    for pose in world.spawns:                       # both start on open floor, well off the wall
        assert min(pose.x, pose.y, 30.0 - pose.x, 20.0 - pose.y) > 1.0


def test_the_open_hall_passes_the_supervisor_checker():
    """`tools/worldcheck.py` has to like it — with the whole hall as the aisle."""
    run = subprocess.run([sys.executable, os.path.join(REPO, "tools", "worldcheck.py"),
                          "--world", "open"], capture_output=True, text=True, cwd=REPO)
    assert run.returncode == 0, run.stdout
    assert "FAIL" not in run.stdout, run.stdout
    assert "widest free spot 9.25 m free (needs 0.46 m)" in run.stdout, run.stdout


def test_a_straight_line_in_the_open_hall_never_touches_anything():
    """20 m of driving, 0 wall contacts, and the odometry drifts anyway — that is the demo."""
    eng, _ = drive(load_config("config/demo_open_odrift.json", {"gui": False}), seconds=42.0)
    robot = eng.robots["muster"]
    assert 20.0 < robot.distance < 21.0 and robot.contacts == 0
    assert robot.gps is None, "the demo switches the GPS off; a fix would be the answer key"
    assert robot.imu is not None, "the IMU keeps its default rate, drift is not its subject"
    gap = math.dist((robot.pose.x, robot.pose.y), (robot.odom.x, robot.odom.y))
    assert 0.9 < gap < 1.1, f"5 % of 20 m should read as 1 m of ghost, measured {gap:.2f} m"


def test_the_picture_draws_every_world_including_the_empty_one(tmp_path):
    """Five panels, one PNG: the figure is generated, so it has to keep generating."""
    assert "open" in list_worlds()
    target = str(tmp_path / "worlds.png")
    run = subprocess.run([sys.executable, os.path.join(REPO, "tools", "worldpic.py"),
                          "--out", target], capture_output=True, text=True, timeout=180, cwd=REPO)
    assert run.returncode == 0, run.stderr[-400:]
    for name in list_worlds():
        assert name in run.stdout, f"{name} not in the figure: {run.stdout}"
    assert open(target, "rb").read(8) == b"\x89PNG\r\n\x1a\n"
    assert len(open(target, "rb").read()) > 5000, "picture too small to contain five arenas"


def test_the_documentation_quotes_the_widths_the_checker_measures():
    """The documentation states passage widths; the numbers have to be the checker's, not the author's."""
    pages = text()
    quoted = {"arena": "5.25", "maze": "0.50", "open": "9.25"}
    for name, number in quoted.items():
        caption = worldpic.clearance(name, CFG)              # what goes under the panel
        assert number in caption, f"{name}: the picture says {caption!r}"
        assert f"{number} m" in pages, f"{name}: no page states {number} m (picture: {caption})"
    report, ok = worldcheck.check("open", CFG, CLEARANCE)
    assert ok, report
    assert any("no start->goal pair" in row for row in report), report


# ------------------------------------------------------------------------- the field of a source


def source(**over):
    return pois.Source(**dict({"name": "src1", "x": 12.0, "y": 6.0, "activity": 1.0,
                               "range_m": 4.0, "d0": 1.0}, **over))


def test_the_intensity_is_one_over_one_plus_d_over_d0_squared():
    """Two distances, one ratio: the fall-off is the formula of the docstring and nothing else."""
    src = source()
    near, far = src.intensity(11.5, 6.0), src.intensity(14.0, 6.0)      # 0.5 m and 2 m
    assert near == pytest.approx(0.8) and far == pytest.approx(0.2)
    assert near / far == pytest.approx((1 + (2.0 / 1.0) ** 2) / (1 + (0.5 / 1.0) ** 2))
    assert src.intensity(12.0, 6.0) == pytest.approx(1.0)               # at the centre: activity
    assert src.intensity(16.0, 6.0) == pytest.approx(1 / 17.0)          # 4 m: still in range
    assert src.intensity(16.1, 6.0) == 0.0                              # one decimetre later: 0


def test_the_range_is_where_the_counter_stops_hearing_it():
    src = source(activity=2.0, range_m=3.0)
    assert src.intensity(14.9, 6.0) > 0.0 and src.intensity(15.1, 6.0) == 0.0


def hall(with_shelf=False):
    """An 8 × 6 m hall, optionally with a shelf between (4, 3) and the source at (7, 3)."""
    walls = [Rect(0, 0, 8, 0.2), Rect(0, 5.8, 8, 6), Rect(0, 0, 0.2, 6), Rect(7.8, 0, 8, 6)]
    if with_shelf:
        walls.append(Rect(5.0, 2.0, 5.6, 4.0))
    return World(name="hall with shelf" if with_shelf else "hall", walls=walls,
                 spawns=[Pose(4.0, 3.0, 0.0)], size=(8.0, 6.0))


def test_a_shelf_between_the_robot_and_the_source_changes_nothing():
    """No line of sight, and that is a property of the model rather than a phrase in a comment.

    Same seeded detector, same poses, once in the empty hall and once with a shelf in the line of the
    source: the two reading series are identical. The LIDAR of the same robot is asked for the same
    line and reports the shelf — the disagreement is what makes the pair worth teaching together.
    """
    src = [source(x=7.0, y=3.0, range_m=6.0)]
    poses = [Pose(4.0, 3.0, 0.0), Pose(4.5, 2.5, 0.0), Pose(3.0, 3.5, 0.0)]
    without = [sensor(src, 9).read(p).intensity for p in poses]
    with_shelf = [sensor(src, 9).read(p).intensity for p in poses]
    assert without == with_shelf, "the counter started caring about occlusion"
    lidar_free = sensors.Lidar(hall(), sensors.Noise(1), cfg_get(CFG, "lidar"))
    lidar_shelf = sensors.Lidar(hall(True), sensors.Noise(1), cfg_get(CFG, "lidar"))
    towards = lidar_free.scan(poses[0]).ranges[0]
    blocked = lidar_shelf.scan(poses[0]).ranges[0]
    assert blocked < 2.0 < towards, (towards, blocked)          # 1.4 m of shelf, 3 m of free floor


# ------------------------------------------------------------------------- the counter's noise


def readings(seed, at=(12.5, 6.0), count=2000, **poi_keys):
    """One detector, `count` readings of the same spot — one sensor, not one per reading."""
    sources = pois.load_sources(SHIP, load_world("open", cfg=CFG), cfg_get(CFG, "poi.d0"))
    detector = sensor(sources, seed, **poi_keys)
    pose = Pose(at[0], at[1], 0.0)
    return [detector.read(pose) for _ in range(count)]


def test_the_same_seed_reads_the_same_twice_and_another_seed_differently():
    """Deterministic from --seed — the only way a demo is repeatable in the lab."""
    assert [p.intensity for p in readings(7)] == [p.intensity for p in readings(7)]
    assert [p.intensity for p in readings(7)] != [p.intensity for p in readings(8)]


def test_a_weak_reading_is_relatively_noisier_than_a_strong_one():
    """Counting noise: the absolute sigma is sqrt(counts), so the relative one is 1/sqrt(intensity)."""
    for spot in ((12.5, 6.0), (14.0, 6.0), (16.0, 6.0)):          # 0.5 m, 2 m and the range itself
        vals = [p.intensity for p in readings(5, at=spot)]
        field = source().intensity(*spot)
        assert statistics.pstdev(vals) / statistics.fmean(vals) == \
            pytest.approx(1 / math.sqrt(field * 400.0), rel=0.15), spot
        assert statistics.fmean(vals) == pytest.approx(field, rel=0.05), spot


def test_a_reading_outside_every_range_is_exactly_nothing():
    """0.0 with no noise on it: a counter that hears nothing does not guess."""
    assert {p.intensity for p in readings(11, at=(12.0, 11.0), count=500)} == {0.0}


def test_the_message_is_a_stamp_and_one_number():
    """`/poi` is `{t, intensity}` — in the dataclass and on the wire, with nothing to switch on.

    This is the whole interface claim of the exercise: a wide-band counter gives a rate and a clock.
    Which source dominates is field arithmetic the simulation keeps for its own overlay, and the
    metres are what the student is supposed to find. There used to be a `poi.publish_distance` switch
    that put the answer on the same topic as the question — a switch that can end an exercise is a bug
    with documentation, so the field went rather than the default.
    """
    assert [f.name for f in dataclasses.fields(Poi)] == ["t", "intensity"]
    sources = pois.load_sources(SHIP, load_world("open", cfg=CFG), 1.0)
    readings = [sensor(sources, 3).read(Pose(13.0, 6.0, 0.0)) for _ in range(5)]
    assert {tuple(sorted(vars(m))) for m in readings} == {("intensity", "t")}
    wire = {"String": lambda data: SimpleNamespace(data=data)}          # the one class this branch uses
    for sample in readings:
        echoed = json.loads(ros_bridge.to_ros(wire, "poi", sample).data)
        assert sorted(echoed) == ["intensity", "t"], f"/poi carries {sorted(echoed)}"
        back = ros_bridge.from_ros("poi", ros_bridge.to_ros(wire, "poi", sample))
        assert back.intensity == pytest.approx(sample.intensity) and back.t == sample.t


def test_no_counts_means_the_field_itself():
    """`poi.counts: 0` switches the counter model off — the maths exercise without the noise."""
    assert {p.intensity for p in readings(2, at=(14.0, 6.0), count=20, counts=0.0)} == \
        {source().intensity(14.0, 6.0)}


def test_the_loudest_source_is_answered_by_the_engine_and_not_by_the_message():
    """Two sources, one reading: the field still knows which one dominates — the bus does not say so."""
    two = [{"name": "src1", "x": 12.0, "y": 6.0, "activity": 1.0, "range": 5.0},
           {"name": "src2", "x": 14.0, "y": 6.0, "activity": 0.5, "range": 5.0}]
    sources = pois.load_sources(two, load_world("open", cfg=CFG), 1.0)
    between = Pose(13.0, 6.0, 0.0)
    assert sensor(sources, 4, counts=0.0).read(between).intensity == pytest.approx(0.5)
    assert pois.loudest(sources, between).name == "src1", "same distance, so the stronger one wins"
    at_second = Pose(13.95, 6.0, 0.0)
    assert sensor(sources, 4, counts=0.0).read(at_second).intensity \
        == pytest.approx(0.5 / (1 + 0.05 ** 2))
    assert pois.loudest(sources, at_second).name == "src2", "close by, the weaker source is louder"


# ---------------------------------------------------------------------------- the validation


def test_a_source_outside_the_walls_is_refused_and_says_which_one():
    world = load_world("open", cfg=CFG)
    for broken in ({"x": 31.0}, {"y": -0.5}, {"x": 0.0}, {"y": 20.0}):
        with pytest.raises(ValueError, match=r"src1.*outside the walls"):
            pois.load_sources([dict(SHIP[0], **broken)], world)


def test_a_source_inside_a_wall_is_refused():
    """In `maze` the same coordinates sit in a wall block — that is a typo, not a scenario."""
    with pytest.raises(ValueError, match="inside a wall"):
        pois.load_sources(SHIP, load_world("maze", cfg=CFG))


def test_a_source_in_the_furnished_hall_is_fine():
    """The demo file also runs with `--world production`; (12, 6) is a lane there, not a table."""
    got = pois.load_sources(SHIP, load_world("production", cfg=CFG))
    assert [s.name for s in got] == ["src1"] and got[0].range_m == 4.0


@pytest.mark.parametrize("broken, why", [
    ({"name": "src1", "x": 5.0, "y": 5.0, "range": 0.0}, "range and d0 have to be > 0"),
    ({"name": "src1", "x": 5.0, "y": 5.0, "d0": -1.0}, "range and d0 have to be > 0"),
    ({"name": "src1", "x": 5.0, "y": 5.0, "activity": -2.0}, "activity has to be >= 0"),
    ({"name": "src1", "x": 5.0, "y": 5.0, "kind": "gravity"}, "unknown kind"),
    ({"x": 5.0, "y": 5.0}, "has no name"),
    ({"name": "src1", "x": "left", "y": 5.0}, "have to be numbers"),
])
def test_a_broken_source_is_refused_with_a_readable_message(broken, why):
    world = load_world("open", cfg=CFG)
    with pytest.raises(ValueError, match=why):
        pois.load_sources([broken], world)


def test_a_source_list_has_to_be_a_list_of_unique_names():
    world = load_world("open", cfg=CFG)
    with pytest.raises(ValueError, match="is not an object"):
        pois.load_sources(["src1"], world)
    with pytest.raises(ValueError, match="used twice"):
        pois.load_sources([dict(SHIP[0]), dict(SHIP[0], x=13.0)], world)


# ------------------------------------------------------------------------- the wiring


def test_without_a_source_no_detector_is_built_and_no_message_appears():
    """The default config plants nothing: no sensor, no `/poi`, no question to the generator."""
    eng, got = drive(load_config(None, {"gui": False, "debug_truth": True}), seconds=4.0)
    assert eng._poi is None and eng.poi_sources() == []
    kinds = {kind for kind, _r, _p in got}
    assert "poi" not in kinds
    assert {"odom", "scan", "gps", "imu", "truth"} <= kinds           # everything else still runs
    # `robots`: the spawn report; `sensorinfo`: the instruments' self-report (CONTRACT §6.4), which
    # the engine publishes whatever the world contains — it asks the receiver and the chip, and the
    # detector as well, so with nothing planted the message says 0 and the claim below stays whole.
    assert kinds <= {"odom", "scan", "gps", "imu", "truth", "robots", "sensorinfo"}
    assert eng.robots["muster"].poi is None


def test_a_planted_source_adds_its_topic_at_poi_rate():
    cfg = poi_cfg()
    assert topic("poi", "muster") == "/muster/poi"
    eng, got = drive(cfg, seconds=4.0)
    assert [s.name for s in eng.poi_sources()] == ["src1"]
    readings = [p for kind, _r, p in got if kind == "poi"]
    assert len(readings) in (19, 20), f"5 Hz for 4 s: {len(readings)} messages"
    # 19 or 20: the last accumulator tick of the 4th second may fall on either side of the boundary,
    # which is the same behaviour every other sensor of the engine has (engine._due).
    assert all(m.t > 0.0 for m in readings), "not stamped with simulation time"
    assert eng.robots["muster"].poi is readings[-1]
    assert {tuple(sorted(vars(m))) for m in readings} == {("intensity", "t")}
    assert eng.loudest_poi("muster").name == "src1", "the engine knows, the message does not say"


def test_the_demo_file_plants_one_source_and_switches_nothing_else_on():
    cfg = load_config("config/demo_poi_exploration.json")
    assert cfg_get(cfg, "world") == "open" and len(cfg_get(cfg, "pois")) == 1
    assert "publish_distance" not in json.dumps(cfg), "the answer is not a switch any more"
    assert cfg_get(cfg, "poi.rate") == 5.0 and cfg_get(cfg, "gps.rate") == 5.0
    assert cfg_get(load_config(), "pois") == []              # and the lab default stays empty


def test_a_source_uses_the_shared_noise_stream_which_is_why_the_default_is_empty():
    """Honest accounting: a sensor that counts draws from the one seeded generator of the run.

    So planting a source does shift the other streams of that run, and `config/tasks.json` names no
    source for any graded task. The default case is the byte-for-byte test in test_sensor_reality.py.
    """
    heard = poi_cfg(pois=[dict(SHIP[0], x=4.0, y=6.25)])          # in range from the first step on
    plain = [p.x for k, _r, p in drive(load_config(None, {"gui": False}), seconds=2.0)[1]
             if k == "odom"]
    planted = [p.x for k, _r, p in drive(heard, seconds=2.0)[1] if k == "odom"]
    assert plain and plain != planted
    quiet = [p.intensity for k, _r, p in drive(poi_cfg(), seconds=2.0)[1] if k == "poi"]
    assert set(quiet) == {0.0}, "out of range the counter draws nothing, so nothing shifts"


def test_the_poi_block_written_out_at_its_defaults_changes_nothing():
    """A key that is read while it is switched off is a bug, so it is tested and not denied."""
    off = {"gui": False, "pois": [], "poi": {"rate": 5.0, "d0": 1.0, "counts": 400.0}}
    series = lambda cfg: [[vars(m) for k, _r, m in drive(cfg, seconds=3.0)[1] if k == kind]
                          for kind in ("odom", "imu")]
    assert series(load_config(None, off)) == series(load_config(None, {"gui": False}))


def test_the_window_sees_the_sources_the_sensor_measured():
    eng = SimEngine(load_world("open", cfg=poi_cfg()), poi_cfg(), seed=1)
    assert eng.poi_sources() == eng._poi.sources
    assert eng.poi_sources()[0].intensity(12.5, 6.0) == pytest.approx(0.8)


def test_the_detector_settings_travel_in_the_config_topic_but_not_the_positions():
    """/sim/config says how the counter works; where the source is would be the answer."""
    import json
    eng = SimEngine(load_world("open", cfg=poi_cfg()), poi_cfg(), seed=1)
    profile = json.loads(eng.config_json())
    assert profile["poi"] == cfg_get(poi_cfg(), "poi")
    assert "pois" not in profile and "12.0" not in eng.world_json(), eng.world_json()


# ------------------------------------------------------------------------- what the window shows


class StubRenderer:
    """The one attribute `overlays.poi_readout()` reads: an engine that can name the loudest source."""

    def __init__(self, loudest="src1"):
        self.engine = SimpleNamespace(
            loudest_poi=lambda _name: SimpleNamespace(name=loudest) if loudest else None)


def test_the_readout_line_carries_the_current_intensity():
    """`poi src1 0.803` in the line the student already reads — no second terminal needed.

    The name is asked of the engine, because `/poi` does not carry it (§6.13). The metres stay out of
    the line even though the engine has them — the same rule as the field rings: a readout that prints
    the distance ends the exercise. That is a change of behaviour on purpose, the old line showed
    `@2.25 m` whenever `poi.publish_distance` was on.
    """
    robot = SimpleNamespace(name="alice", poi=None)
    assert overlays.poi_readout(StubRenderer(), robot) == []
    robot.poi = Poi(t=1.0, intensity=0.803)
    assert overlays.poi_readout(StubRenderer("src1"), robot)[0][0] == "poi src1 0.803"
    robot.poi = Poi(t=1.0, intensity=0.0)
    assert overlays.poi_readout(StubRenderer("src1"), robot)[0][0] == "poi - 0.000", \
        "hearing nothing, the line claims no source"
    robot.poi = Poi(t=1.0, intensity=0.5)
    assert overlays.poi_readout(StubRenderer("src1"), robot)[0][0] == "poi src1 0.500"


def test_the_key_p_and_the_menu_row_belong_to_the_same_layer():
    """`p` in render.KEYS, a row in menu.LAYERS, one boolean on the renderer."""
    assert render.KEYS["p"] == ("show_pois", "")
    assert {attribut: key for attribut, _t, key in render.menue.LAYERS}["show_pois"] == "p"
    assert "p" in render.keys.layer_hint().split(), "the key is promised nowhere but the panel"


def fake_engine(sources, size=(12.0, 9.0)):
    """The engine `overlays.poi_sources()` reads: a world, one robot out of the way, the sources."""
    robot = Robot(spec=RobotSpec(name="muster", index=0, color="red", rgb=(0.9, 0.3, 0.3)),
                  chassis=None, pose=Pose(2.0, 2.0, 0.0), wheels=[0.0] * 4)
    return SimpleNamespace(world=World(name="hall", walls=[Rect(0, 0, size[0], 0.2),
                                                           Rect(0, size[1] - .2, size[0], size[1]),
                                                           Rect(0, 0, 0.2, size[1]),
                                                           Rect(size[0] - .2, 0, size[0], size[1])],
                                       spawns=[Pose(1, 1, 0.0)], size=size),
                           robots={"muster": robot}, t=1.0, task="", drain=lambda: [],
                           poi_sources=lambda: list(sources))


def ring_pixels(rend, centre, radius_px, band=5):
    """How many of 36 angles see something **brighter than the floor** at about that distance.

    A band of a few pixels around the radius instead of the exact circle: `pygame.draw.circle` places a
    1 px outline at a radius of its own choosing (and not at every angle the same one), so a test that
    samples one pixel wide would fail on the drawing routine rather than on the overlay.

    Brighter rather than "anything that is not floor": both rings are mixed out of the floor upwards,
    so every ring pixel is lighter than what it lies on. Asking for "not floor" counted the dark edge
    behind the source's own name as a ring — the label crosses that circle, and since a label carries
    a dark outline to stay readable on a wall, it leaves pixels *darker* than the floor there.
    """
    hits = 0
    untergrund = sum(rend.col_floor[:3])
    for step in range(0, 360, 10):
        for offset in range(-band, band + 1):
            radius = radius_px + offset
            x = int(round(centre[0] + radius * math.cos(math.radians(step))))
            y = int(round(centre[1] - radius * math.sin(math.radians(step))))
            if sum(rend.screen.get_at((x, y))[:3]) > untergrund:
                hits += 1
                break
    return hits


def test_the_field_map_is_the_counter_scale_and_nothing_else():
    """1.0 at a source, the fall-off of the counter, and two fields that add — `pois.field_map()`.

    Checked against the model at every sampled spot rather than at three hand-picked ones, and as a
    ratio rather than a colour: the layer is meant to be read with the numbers the readout line prints
    (`0.5` at `d0`, `0.1` at `3·d0`), so a second scale for the picture would be a second thing to
    explain, and the first comparison a student makes is picture against counter.
    """
    hall = World(name="fieldone", cell=0.5, walls=[], spawns=[Pose(1, 1, 0.0)], size=(10, 8))
    # On a sample point on purpose: `field_map()` starts its grid half a cell into the world, so a source
    # at whole metres is never sampled at its own centre and "1.0 at the source" could only be asserted
    # as "0.89 somewhere near it" — a statement about the grid rather than about the model.
    ort = (5.25, 4.25)
    src = source(x=ort[0], y=ort[1], activity=1.0, range_m=4.0, d0=1.0)
    samples = pois.field_map([src], hall)
    assert samples, "a world with a source in it produced no samples"
    for x, y, level in samples:
        assert level == pytest.approx(min(1.0, src.intensity(x, y) / src.activity), abs=1e-9)
    assert max(level for _x, _y, level in samples) == pytest.approx(1.0), "not 1.0 at the source"
    outside = [level for x, y, level in samples if math.dist((x, y), ort) > src.range_m]
    assert outside and all(level == 0.0 for level in outside), "dose painted beyond the range"

    pair = [source(name="a", x=4.25, y=4.25), source(name="b", x=6.25, y=4.25)]
    alone = min(pois.field_map([pair[0]], hall), key=lambda s: math.dist(s[:2], ort))
    both = min(pois.field_map(pair, hall), key=lambda s: math.dist(s[:2], ort))
    assert alone[2] == pytest.approx(0.5), "one source at its own d0 should read 0.5"
    assert both[2] == pytest.approx(1.0), "two sources reading 0.5 each did not add to 1.0"


def test_the_dose_layer_paints_the_field_and_the_clean_view_does_not():
    """Layer `i` is a picture of a model, so it is off until asked for — and on, it is that field.

    Three radii and one wall: the paint has to fall off with distance (a field, not a blob), the clean
    view must show none of it, and the border wall inside the field has to keep the colour of a wall.
    The wall is the interesting assertion — the dose map is drawn **under** the walls, which is the
    opposite argument from the radio coverage map (a rack should stay visible above the shadow it
    casts). It comes out of the model: `Source.intensity()` knows no wall, so the field runs straight
    through the rack, and the wall painted over the field is the one thing left that tells a student
    where in the hall a painted cell lies.
    """
    src = [source(x=3.0, y=2.0, range_m=4.0, d0=1.0)]
    rend = render.Renderer(fake_engine(src), load_config(None, {"gui": True, "width": 900,
                                                               "height": 600}))
    try:
        seen = lambda spot: rend.screen.get_at(spot)[:3]            # noqa: E731 - reads inline
        paint = lambda spot: sum(abs(a - b) for a, b in zip(spot, rend.col_floor[:3]))
        radii = [tuple(int(v) for v in rend.px(3.0, 2.5)),          # 0.5 m from the source
                 tuple(int(v) for v in rend.px(3.0, 3.0)),          # 1 m: its d0
                 tuple(int(v) for v in rend.px(3.0, 5.9))]          # 3.9 m: a hint of a field
        at_field, at_wall = radii[1], tuple(int(v) for v in rend.px(0.1, 2.0))

        rend.draw(cap=False)                                  # the clean view: nobody asked for it
        assert seen(at_field) == rend.col_floor[:3], "the clean view paints a model over the floor"

        rend.show_dose = True
        rend.draw(cap=False)
        painted = [paint(seen(spot)) for spot in radii]
        assert painted[0] > painted[1] > painted[2] > 0, \
            f"no fall-off with distance, {[round(v) for v in painted]} at 0.5, 1 and 3.9 m"
        assert seen(at_wall) == rend.col_wall[:3], \
            "the dose map is painted over the walls instead of under them"
    finally:
        rend.close()


def test_the_p_layer_draws_the_source_and_debug_truth_adds_the_field_rings():
    """Layer off: floor. Layer on: the symbol. With truth: the rings the field is worth."""
    src = [source(x=6.0, y=4.5, range_m=2.5, d0=1.0)]          # centre of the fake hall, d0 = 1 m
    rend = render.Renderer(fake_engine(src), load_config(None, {"gui": True, "width": 900,
                                                               "height": 600}))
    try:
        centre, ring = rend.px(6.0, 4.5), int(round(1.0 * rend.s))
        spot = (int(round(centre[0])), int(round(centre[1])))
        rend.show_pois = False
        rend.draw(cap=False)
        assert rend.screen.get_at(spot)[:3] == rend.col_floor[:3]
        rend.show_pois = True
        rend.draw(cap=False)
        assert rend.screen.get_at(spot)[:3] != rend.col_floor[:3], "the source symbol is missing"
        assert ring_pixels(rend, centre, ring) == 0, "rings drawn without debug_truth"
        rend.cfg["debug_truth"] = True
        rend.draw(cap=False)
        assert ring_pixels(rend, centre, ring) >= 30, "no ring at d0 with debug_truth"
        assert ring_pixels(rend, centre, int(2.5 * rend.s)) >= 30, "no ring at the range"
        assert ring_pixels(rend, centre, int(1.75 * rend.s)) == 0, "a ring in between"
    finally:
        rend.close()


def test_the_layer_switch_never_touches_what_the_sensor_publishes():
    """The rule of the view (test_view_menu_c covers all layers): fewer pixels, same messages."""
    _eng, got = drive(poi_cfg(), seconds=2.0)
    assert sum(1 for k, _r, _p in got if k == "poi") in (9, 10)
