"""The radio link: one link budget, one delivery rule, one onboard rule (`mecanum_lab/wifi.py`).

Two promises are checked here and they pull in opposite directions, which is why both live in one
file. The first is that `wifi.enabled: false` — the shipped default — changes *nothing*: the graded
runs of both experiments are calibrated on a link that delivers every frame at once, so the command
stream, the recorded CSV and the message series must be exactly what they were before a radio existed
(`types.DEFAULT_CONFIG` claims that, `test_the_command_stream_is_identical...` proves it). The second
is that with the option on, the model does the arithmetic of its own docstring: distance and wall
crossings in; quality, drop probability, delivery delay and — below the failsafe level — autonomy out.

Nothing in here waits for anything. Every drive is stepped in fixed physics steps and every timing
assertion counts **simulation** seconds, so one seed gives the same numbers on a loaded machine
(`node.run_loop(..., fixed_step=True)` and `MECANUM_FAST`, the same switch `./lab run --fixed-step`
sets). The two places where a rate would blur a measurement say so and are given a rate to remove it.

Every number asserted is reproducible from `wifi.py` and the spot named next to it; the spots are
open floor of `worlds/production.txt`, measurable with a tape measure in the window.
"""
import csv
import importlib.util
import json
import math
import os

import pytest

from mecanum_lab import node, overlays, render, ros_bridge, wifi
from mecanum_lab.engine import SimEngine
from mecanum_lab.logbook import Logbook
from mecanum_lab.stub import StubBus
from mecanum_lab.types import Link, Pose, Rect, Twist, World, cfg_get, load_config, topic, wrap_angle
from mecanum_lab.worlds import load_world

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEMO = os.path.join(REPO, "config", "demo_wifi.json")
EXAMPLE = os.path.join(REPO, "student", "link_autonomy_example.py")
STEP = 0.02                                  # the 50 Hz physics step every drive below runs at
AP = (1.0, 2.0)                              # production's own access point, above the loading area
SPAWN = (2.25, 2.75)                         # where the first robot of a run stands
FAR = (18.7, 2.75)                           # east end of the south aisle: 17.7 m free, q 0.43


def config(**wifi_over) -> dict:
    """The lab configuration with the radio switched on; every keyword is one `wifi` key.

    `config(enabled=False)` is the one reversal the tests for the off-path need, so the default is
    set by updating a dict and not by a keyword that `wifi_over` could collide with.
    """
    wifi_block = {"enabled": True}
    wifi_block.update(wifi_over)
    return load_config(None, {"gui": False, "wifi": wifi_block})


def radio(world: str = "production", seed: int = 1, **wifi_over) -> wifi.Wifi:
    """A bare radio over a named hall, for the questions that do not need a robot."""
    from mecanum_lab.sensors import Noise
    cfg = config(**wifi_over)
    hall = load_world(world, cfg=cfg)
    return wifi.Wifi(hall, wifi.access_point(cfg_get(cfg, "wifi"), hall), Noise(seed),
                     cfg_get(cfg, "wifi"))


def sim(cfg: dict, seed: int = 1, world: str = "production", name: str = "muster") -> SimEngine:
    """One simulation with one robot in it — the shape nearly every test here starts from."""
    eng = SimEngine(load_world(world, cfg=cfg), cfg, seed=seed)
    eng.spawn(name)
    return eng


def place(eng: SimEngine, spot: tuple, name: str = "muster") -> None:
    """Stand a robot at one spot: an antenna has to be measured somewhere, and driving there each
    time would put the drive's own noise between the test and the thing it tests."""
    robot = eng.robots[name]
    robot.chassis.pose = Pose(spot[0], spot[1], robot.pose.theta)
    robot.pose.x, robot.pose.y = spot


def step_once(eng: SimEngine, name: str = "muster") -> list:
    eng.step(eng.sub_step)
    return eng.drain()


# --------------------------------------------------------------- the one option, and its absence


def test_the_option_is_off_in_the_defaults_and_on_in_the_demo():
    """One option, one demo file that turns it on — and nothing else different in that file."""
    default = cfg_get(load_config(), "wifi")
    assert default["enabled"] is False, "a graded run must not start with a radio by accident"
    demo = load_config(DEMO)
    assert cfg_get(demo, "wifi.enabled") is True
    assert cfg_get(demo, "world") == "production"
    for key, value in default.items():
        if key != "enabled":
            assert cfg_get(demo, f"wifi.{key}") == value, f"the demo changed {key}"


def test_no_radio_is_built_while_the_option_is_off():
    eng = sim(load_config(None, {"gui": False}))
    assert eng._wifi is None
    assert eng.link_health("muster") is None
    assert overlays.link_readout(StubRenderer(), eng.robots["muster"]) == []


def test_the_command_stream_is_identical_while_the_option_is_off(tmp_path):
    """The CSV of one fixed drive, run with the `wifi` block written out at its defaults, == plain.

    This is the guard for every line the feature touched in the command path: `set_cmd_vel` and
    `set_wheel_speeds` go through `_deliver()` now, and an option that is off may not delay, drop,
    count or reorder one frame — nor so much as ask the random generator, which is what would move the
    odom/gps/imu series that both experiments' grades are calibrated on. The last line checks that
    this test has teeth: the same drive *with* the radio must not give the same file.
    """
    plain = record(tmp_path / "plain.csv", load_config(None, {"gui": False}))
    off = record(tmp_path / "off.csv", load_config(None, {"gui": False, "wifi": {
        "enabled": False, "ap": None, "ap_by_world": {}, "rate": 5.0, "tx_dbm": -40.0, "d0": 1.0,
        "n": 2.4, "wall_db": 12.0, "floor_dbm": -85.0, "good_dbm": -50.0, "shadow_db": 3.0,
        "shadow_period": 2.0, "latency_ms": 20.0, "link_up_q": 0.15, "link_timeout": 1.5,
        "autonomy": "stop"}}))
    assert plain == off, "a default changed: the graded runs would move with it"
    assert len(plain) > 100, "the drive has to be long enough for this to mean something"
    assert record(tmp_path / "on.csv", config()) != plain, "the radio changed nothing at all"


def test_no_link_message_exists_while_the_option_is_off():
    eng = sim(config(enabled=False))
    assert [g for g in drive(eng, 3.0, Twist(vx=0.4)) if g[1] == "link"] == []
    assert eng.link_health("muster") is None


def test_link_messages_exist_with_the_option_on():
    got = [g for g in drive(sim(config()), 3.0, Twist(vx=0.4)) if g[1] == "link"]
    assert got and all(isinstance(payload, Link) for _t, _k, _r, payload in got)


def drive(eng: SimEngine, seconds: float, twist: Twist, name: str = "muster") -> list:
    """Step a robot with one command and collect `(sim_t, kind, robot, payload)` of what was sent."""
    seen = []
    for _ in range(int(seconds * eng.cfg["rate"])):
        eng.set_cmd_vel(name, twist)
        for kind, robot, payload in step_once(eng, name):
            seen.append((eng.t, kind, robot, payload))
    return seen


def record(path: str, cfg: dict, seconds: float = 12.0) -> list:
    """One fixed drive through the Logbook, returned as the rows the file ended up holding."""
    eng = sim(cfg)
    book = Logbook(eng, path, interval=0.1)
    for i in range(int(seconds * eng.cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.45, omega=0.05 * (1 if i % 100 < 50 else -1)))
        for kind, robot, payload in step_once(eng):
            book.tap(kind, robot, payload)
        book.tick()
    book.close()
    with open(path, encoding="utf-8") as fh:
        return list(csv.reader(fh, delimiter=";"))


# ------------------------------------------------------------------------ the link budget itself


def test_quality_falls_monotonically_with_distance_on_free_floor():
    """`open` holds nothing but its border, so the only thing between the two is metres."""
    rad = radio("open", ap=[1.0, 3.0])
    spots = [(3.0, 3.0), (5.0, 3.0), (9.0, 3.0), (17.0, 3.0), (25.0, 3.0)]        # 2 m … 24 m
    levels = [rad.budget(x, y)[0] for x, y in spots]
    quals = [rad.budget(x, y)[1] for x, y in spots]
    assert all(b < a for a, b in zip(levels, levels[1:])), levels
    assert quals[0] == 1.0, "2 m from an access point the link is simply good"
    assert all(b <= a for a, b in zip(quals, quals[1:])), quals
    assert quals[3] < quals[2] < quals[1], quals
    # the two numbers README quotes, from the config and nothing else
    assert levels[2] == pytest.approx(-40.0 - 24.0 * math.log10(8.0), abs=0.02)
    assert quals[2] == pytest.approx(0.666, abs=0.002)


def test_the_path_law_is_10_n_log10_and_nothing_else():
    """Doubling the distance costs the same dB at 2 m as at 16 m, because dB are logarithmic."""
    rad = radio("open", ap=[1.0, 3.0])
    base = 10.0 * rad.n * math.log10(2.0)
    for d in (2.0, 4.0, 8.0):
        assert rad.budget(1.0 + d, 3.0)[0] - rad.budget(1.0 + 2 * d, 3.0)[0] == pytest.approx(base)
    assert base == pytest.approx(7.22, abs=0.01), "the number in the module's docstring"


def test_two_walls_cost_at_least_two_times_wall_db():
    """Same hall, same spot, one number changed: the difference *is* the walls."""
    real = radio("production")
    honest = radio("production", wall_db=0.0)                      # same hall, walls cost 0 dB
    one, two = (6.05, 8.20), (15.55, 5.65)   # one rack in the line at 8 m; two racks at 15 m
    assert real.walls_between(*one) == 1 and real.walls_between(*two) == 2
    for spot, walls in ((one, 1), (two, 2)):
        with_walls = real.budget(*spot)[0]
        free = honest.budget(*spot)[0]
        assert with_walls < free, "the walls cost nothing at all"
        assert free - with_walls == pytest.approx(walls * real.wall_db, abs=1e-9)
    # walls cost dB and never metres: the distance term of the two radios is the same number
    assert real.budget(*two)[2] == pytest.approx(honest.budget(*two)[2])
    # and what they cost is paid in frames: same spot, one rack between, nothing drivable left
    assert real.drop_chance(real.budget(*two)[1]) > honest.drop_chance(honest.budget(*two)[1])
    assert real.budget(*two)[1] < honest.budget(*two)[1] < 1.0


def test_the_budget_is_the_documented_formula_at_every_spot_of_the_readme_table():
    """The nine cells of README section "Radio link", recomputed from the config alone."""
    rad = radio("production")
    table = {(2.0, 0): -47.22, (2.0, 1): -59.22, (2.0, 2): -71.22, (8.0, 0): -61.67,
             (8.0, 1): -73.67, (8.0, 2): -85.67, (15.0, 0): -68.23, (15.0, 1): -80.23,
             (15.0, 2): -92.23}
    for (d, walls), rssi in table.items():
        assert rssi == pytest.approx(rad.tx_dbm - 10 * rad.n * math.log10(d / rad.d0)
                                     - walls * rad.wall_db, abs=0.01)
    assert wifi.quality_of(table[(15.0, 1)]) == pytest.approx(0.136, abs=0.002)
    assert wifi.quality_of(table[(15.0, 2)]) == 0.0, "below the floor there is no link at all"
    assert wifi.quality_of(-40.0) == 1.0, "and above the good level it does not get better"
    assert (rad.floor_dbm, rad.good_dbm) == (-85.0, -50.0)


def test_the_fade_walks_and_never_leaves_its_window():
    """Shadow: continuous, slow, bounded — and with `shadow_db: 0` not even a random draw."""
    rad = radio(shadow_db=4.0, shadow_period=2.0)
    steps = [rad.step("muster", Pose(9.0, 3.0, 0.0), STEP).shadow_db for _ in range(1500)]
    assert all(abs(value) <= 4.0 + 1e-9 for value in steps)
    jumps = [abs(b - a) for a, b in zip(steps, steps[1:])]
    # The most one step can carry: a new target may sit on the far side of the window, so the slope
    # is bounded by 2 · shadow_db per shadow_period — 0.08 dB here, not the 0.04 dB of a mid-sweep.
    assert max(jumps) <= 2 * 4.0 * STEP / 2.0 + 1e-9, f"a step of {max(jumps):.2f} dB is a switch"
    assert len({round(value, 6) for value in steps}) > 1000, "the fade never moved"
    assert len({round(value, 6) for value in steps[:100]}) > 90, "the fade is not sampled per step"
    still = radio(shadow_db=0.0)
    assert [still.step("muster", Pose(9.0, 3.0, 0.0), STEP).shadow_db
            for _ in range(500)] == [0.0] * 500


def test_an_access_point_has_to_be_in_the_hall_and_not_in_a_wall():
    """`wifi.ap` over `wifi.ap_by_world` over no radio, and the same three checks a POI gets."""
    hall = load_world("production", cfg=load_config())
    assert wifi.access_point({"ap": [10.0, 6.0]}, hall) == (10.0, 6.0)
    assert wifi.access_point({"ap_by_world": {"production": [3.0, 4.0]}}, hall) == (3.0, 4.0)
    assert wifi.access_point({"ap": [3.0, 4.0], "ap_by_world": {"production": [9.0, 9.0]}},
                             hall) == (3.0, 4.0), "wifi.ap is the override"
    assert wifi.access_point({}, hall) is None, "a hall nobody named has no radio, that is all"
    with pytest.raises(ValueError):
        wifi.access_point({"ap": [10.0]}, hall)                            # not two numbers
    with pytest.raises(ValueError):
        wifi.access_point({"ap": [100.0, 6.0]}, hall)                      # outside the walls
    with pytest.raises(ValueError):
        wifi.access_point({"ap": [10.0, 6.0]}, _walled())                  # inside a wall


def _walled() -> World:
    return World(name="box", walls=[Rect(9.0, 5.0, 12.0, 7.0)], size=(20.0, 12.0),
                 spawns=[Pose(2.0, 2.0, 0.0)])


def test_the_running_simulation_says_which_access_point_it_used():
    """`/sim/config` carries the effective AP, so nobody has to guess which config layer won."""
    shipped = json.loads(sim(config()).config_json())["wifi"]
    assert shipped["ap"] is None and shipped["effective_ap"] == list(AP), \
        "the hall's own spot has to be readable somewhere"
    moved = json.loads(sim(config(ap=[10.0, 6.0])).config_json())["wifi"]
    assert moved["ap"] == [10.0, 6.0], "the configured key is published as it was configured"
    assert moved["effective_ap"] == [10.0, 6.0], "and the effective one follows the override"
    assert json.loads(sim(load_config(None, {"gui": False})).config_json())["wifi"][
        "effective_ap"] is None, "without a radio there is no position to report"


# -------------------------------------------------------------------- delivery: drops and delay


def delivered_pattern(seed: int, spot: tuple = FAR, seconds: float = 6.0,
                      **wifi_over) -> str:
    """One command per step at one spot, one character per frame: did the radio refuse it?

    Read from the radio's own counter and not from what reached the chassis: a frame that is still on
    the wire has arrived nowhere yet, and a latency of exactly one physics step would otherwise read
    as a loss here that is not one.
    """
    eng = sim(config(**wifi_over), seed=seed)
    if spot is not None:
        place(eng, spot)
    pattern = ""
    for _ in range(int(seconds * eng.cfg["rate"])):
        before = eng.link_health("muster")       # None before the first frame: no state yet
        refused = before[5] if before else 0
        eng.set_cmd_vel("muster", Twist())                      # no motion: the spot stays the spot
        step_once(eng)
        pattern += "x" if eng.link_health("muster")[5] > refused else "."
    return pattern


def test_the_drop_pattern_is_the_seed_and_nothing_else():
    assert delivered_pattern(7) == delivered_pattern(7), "same seed, other loss: not reproducible"
    assert delivered_pattern(7) != delivered_pattern(8)
    assert delivered_pattern(7).count("x") != delivered_pattern(3).count("x"), \
        "two seeds that lose the same number of frames is a coin that is not flipped"
    lost = delivered_pattern(7).count("x")
    assert 70 < lost < 130, f"q 0.43 means (1-q)² = 0.32 of 300 frames, measured {lost}"


def test_a_link_at_full_quality_loses_nothing():
    """On the spawn's own line q is 1, so (1 - q)² is 0: perfect, not merely good."""
    assert delivered_pattern(3, spot=None, seconds=6.0).count("x") == 0, \
        "frames were lost 1.5 m from the access point"
    assert delivered_pattern(7, spot=None, seconds=6.0).count("x") == 0


def test_a_command_is_late_in_sim_time_by_exactly_the_latency_of_its_quality():
    """400 ms of wire at q = 1: every frame arrives 0.4 s later, and that is the only change."""
    eng = sim(config(latency_ms=400.0, shadow_db=0.0))
    robot = eng.robots["muster"]
    sent, lags, last = {}, [], None
    for i in range(80):
        sent[0.10 + 0.001 * i] = eng.t
        eng.set_cmd_vel("muster", Twist(vx=0.10 + 0.001 * i))
        step_once(eng)
        now = robot.vel_cmd.vx if robot.vel_cmd else None
        if now is not None and now != last:
            lags.append((eng.t, eng.t - sent[now], now))
            last = now
    assert len(lags) > 30, "hardly a frame arrived"
    assert all(0.4 - 1e-9 <= lag <= 0.4 + 2 * STEP for _t, lag, _v in lags), lags[:6]
    assert max(lag for _t, lag, _v in lags) - min(lag for _t, lag, _v in lags) < 2 * STEP
    assert eng.link_health("muster")[0] == 1.0, "q drifted off 1 and the delay is not 400 ms alone"


def test_a_slow_link_delays_everything_but_keeps_the_order_it_was_given():
    """Quality-dependent latency, and still the frames reach the chassis in the order they were sent.

    A real link reorders; this one does not — `wifi.due()` says why (a student debugging a drive with
    `ros2 topic echo /alice/cmd_vel` cannot use a shuffled sequence). The marker values are strictly
    increasing, so an inversion would show up as a step down in `seen`.
    """
    eng = sim(config(latency_ms=600.0, tx_dbm=-70.0, shadow_db=0.0))     # weak everywhere: q ~ 0.31
    robot = eng.robots["muster"]
    seen, first = [], None
    for i in range(300):
        eng.set_cmd_vel("muster", Twist(vx=0.001 * i))
        step_once(eng)
        now = robot.vel_cmd.vx if robot.vel_cmd else None
        if now is not None and now != (seen[-1][1] if seen else None):
            if first is None:
                first = eng.t
            seen.append((eng.t, now))
    assert len(seen) > 50, f"only {len(seen)} frames arrived in 6 s"
    assert [v for _t, v in seen] == sorted(v for _t, v in seen), "the commands arrived shuffled"
    assert 1.4 < first < 2.2, f"q 0.31 on a 600 ms wire is 1.43 s, the first frame arrived at {first}"


def test_the_wheel_command_path_goes_through_the_same_radio():
    """`/wheels` is a command like `/cmd_vel`: the radio decides when the chassis hears it."""
    eng = sim(config(latency_ms=200.0, shadow_db=0.0))
    robot = eng.robots["muster"]
    eng.set_wheel_speeds("muster", [3.0] * 4)
    step_once(eng)
    assert robot.wheel_cmd is None, "the frame reached the chassis before its wire time was over"
    for _ in range(int(0.25 / STEP)):
        step_once(eng)
    assert robot.wheel_cmd == [3.0] * 4 and robot.mode == "wheels"
    assert max(abs(w) for w in robot.wheels) > 1.0


def test_latency_and_loss_grow_as_the_link_gets_worse():
    rad = radio()
    assert rad.latency(1.0) == pytest.approx(0.020), "even a good link is not free"
    assert rad.latency(0.5) == pytest.approx(0.040)
    assert rad.latency(0.0) == pytest.approx(0.060), "three times the floor at q = 0"
    assert rad.drop_chance(0.9) == pytest.approx(0.01)
    assert rad.drop_chance(0.5) == pytest.approx(0.25)
    assert rad.drop_chance(0.0) == pytest.approx(1.0)


# ------------------------------------------------------------------------- the autonomy rule


def walk_into_shadow(eng: SimEngine, name: str = "muster", limit: float = 90.0) -> dict:
    """Down the south aisle and up the east one until the radio gives out. Sim seconds only.

    The route of `student/link_autonomy_example.py`, shortened to the two legs that matter, with the
    same heading P controller. The return holds the sim time at which the level first fell under
    `wifi.link_up_q` and the sim time at which `mode` became `autonomy` — the two numbers README
    quotes, and both are 1/50 s-accurate because the loop steps the physics itself.
    """
    robot = eng.robots[name]
    waypoints = [(18.7, SPAWN[1]), (18.7, 6.0)]
    leg, low_at, flip_at = 0, None, None
    for _ in range(int(limit * eng.cfg["rate"])):
        if leg < len(waypoints) - 1 and math.dist(waypoints[leg], (robot.pose.x, robot.pose.y)) < 0.4:
            leg += 1
        tx, ty = waypoints[leg]
        err = wrap_angle(math.atan2(ty - robot.pose.y, tx - robot.pose.x) - robot.pose.theta)
        eng.set_cmd_vel(name, Twist(vx=0.6 if abs(err) < 0.6 else 0.0, omega=2.2 * err))
        step_once(eng)
        view = eng.link_health(name)
        if view[0] < cfg_get(eng.cfg, "wifi.link_up_q") and low_at is None:
            low_at = eng.t
        if robot.mode == "autonomy" and flip_at is None:
            flip_at = eng.t
        if flip_at is not None and eng.t > flip_at + 2.0:
            break
    return {"low": low_at, "flip": flip_at, "t": eng.t}


def test_the_link_goes_down_link_timeout_after_the_level_drops_and_not_before():
    """The failsafe is a timer in simulation seconds: 1.5 s, not a wall clock and not one bad step."""
    cfg = config(shadow_db=0.0)
    eng = sim(cfg)
    robot = eng.robots["muster"]
    low_at = flip_at = None
    for _ in range(int(90.0 * eng.cfg["rate"])):
        tx, ty = (18.7, SPAWN[1]) if eng.t < 33.0 else (18.7, 6.0)
        err = wrap_angle(math.atan2(ty - robot.pose.y, tx - robot.pose.x) - robot.pose.theta)
        eng.set_cmd_vel("muster", Twist(vx=0.6 if abs(err) < 0.6 else 0.0, omega=2.2 * err))
        step_once(eng)
        if eng.link_health("muster")[0] < cfg_get(cfg, "wifi.link_up_q") and low_at is None:
            low_at = eng.t
        if robot.mode == "autonomy" and flip_at is None:
            flip_at = eng.t
        if flip_at is not None and eng.t > flip_at + 0.5:
            break
    assert low_at is not None and flip_at is not None, f"never left coverage (low={low_at})"
    waited = flip_at - low_at
    # One physics step of resolution: the timer counts with the step that first measured the low
    # level, and the sample above is taken at the end of that step. 1/50 s is the finest statement a
    # simulation like this can make about a timeout — and so about `link_timeout` itself.
    assert abs(waited - cfg_get(cfg, "wifi.link_timeout")) < STEP, \
        f"the failsafe took {waited:.3f} s of sim time instead of wifi.link_timeout"


def test_autonomy_holds_position_and_nothing_external_arrives_anymore():
    eng = sim(config(shadow_db=0.0))
    walk_into_shadow(eng)
    robot = eng.robots["muster"]
    assert robot.mode == "autonomy"
    assert eng.link_health("muster")[4] is False, "the /link message still claims the link is up"
    place_after = (robot.pose.x, robot.pose.y)
    driving = robot.vel_cmd.vx
    for _ in range(int(6.0 * eng.cfg["rate"])):            # the student drives hard the other way
        eng.set_cmd_vel("muster", Twist(vx=-0.6, omega=1.0))
        step_once(eng)
    assert robot.vel_cmd.vx == pytest.approx(driving), "a rescue command reached the robot after all"
    assert math.dist(place_after, (robot.pose.x, robot.pose.y)) < 0.05, "it did not hold its place"
    assert max(abs(w) for w in robot.wheels) < 0.02
    assert robot.mode == "autonomy"
    assert eng.link_health("muster")[5] > 0, "the frames it refused have to be counted"


def test_the_link_comes_back_when_the_level_comes_back():
    """The other half of a failsafe: a robot carried into coverage is drivable again."""
    eng = sim(config(shadow_db=0.0))
    walk_into_shadow(eng)
    robot = eng.robots["muster"]
    assert robot.mode == "autonomy"
    place(eng, SPAWN)
    for _ in range(int(2.0 * eng.cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.3))
        step_once(eng)
    assert robot.mode == "pass-through", f"back in coverage and still {robot.mode}"
    assert eng.link_health("muster")[0] == pytest.approx(1.0)
    assert eng.link_health("muster")[4] is True


def test_dead_reckoning_keeps_executing_the_last_command_that_arrived():
    """The other onboard rule, and the reason a course has to choose one: it drives on regardless."""
    eng = sim(config(shadow_db=0.0, autonomy="dead_reckoning"))
    walk_into_shadow(eng)
    robot = eng.robots["muster"]
    assert robot.mode == "autonomy" and robot.vel_cmd.vx == pytest.approx(0.6)
    before = robot.distance
    for _ in range(int(2.0 * eng.cfg["rate"])):
        step_once(eng)                                    # nothing is sent: the robot is on its own
    assert robot.distance > before + 0.4, f"only {robot.distance - before:.2f} m: that is 'stop'"


def test_an_unreachable_robot_is_still_a_measured_link():
    """`up: false` is a reading: the topic keeps running, or "far away" and "no topic" look alike."""
    eng = sim(config(shadow_db=0.0))
    place(eng, (15.55, 5.65))                             # 15 m behind two racks: q 0.00
    links = [payload for _t, kind, _r, payload in drive(eng, 4.0, Twist()) if kind == "link"]
    assert links and links[0].up is True and links[-1].up is False, \
        "the level is fatal at once, the failsafe only after wifi.link_timeout"
    assert sum(1 for p in links if not p.up) > 5, "the topic stopped instead of reporting"
    assert links[-1].quality == 0.0 and links[-1].rssi_dbm < -85.0
    assert eng.link_health("muster")[4] is False


# ------------------------------------------------------------------------ the /link topic


def link_stream(cfg: dict, seconds: float = 6.0, seed: int = 1) -> list:
    """Every `/link` of one drive, with the position the robot was at when it was stamped."""
    eng = sim(cfg, seed=seed)
    out = []
    for _ in range(int(seconds * eng.cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.4))
        for kind, _robot, payload in step_once(eng):
            if kind == "link":
                out.append((eng.t, payload, (eng.robots["muster"].pose.x,
                                             eng.robots["muster"].pose.y)))
    return out


def test_link_is_published_at_wifi_rate_with_the_model_in_its_fields():
    cfg = config(rate=10.0, shadow_db=0.0)
    got = link_stream(cfg, 6.0)
    assert 55 < len(got) < 65, f"10 Hz for 6 s is 60 messages, measured {len(got)}"
    _t, msg, spot = got[-1]
    rad = radio()
    rssi, q, _d, _walls = rad.budget(*spot)
    assert msg.ap == pytest.approx(AP), "the message has to name the access point it is about"
    assert msg.rssi_dbm == pytest.approx(rssi, abs=1e-9), "not the model's number at that spot"
    assert msg.quality == pytest.approx(q, abs=1e-9)
    assert msg.quality == pytest.approx(wifi.quality_of(msg.rssi_dbm), abs=1e-9)
    assert msg.up is True and msg.dropped >= 0
    # The wire time in the message is the one of the last frame that went on the wire, so it belongs
    # to a spot one step behind this one: 0.1 ms is a tenth of a millisecond of walking at 0.4 m/s.
    assert msg.latency_ms == pytest.approx(1000.0 * rad.latency(msg.quality), abs=0.1)
    assert 20.0 <= msg.latency_ms <= 60.0, "latency_ms lives between the floor and three times that"
    assert topic("link", "alice") == "/alice/link"
    assert ros_bridge.KIND_MSG["link"] == "String", "no custom interface, no build step"


def test_a_link_message_survives_the_ros_form_it_travels_in():
    """/link is JSON on a String (no custom interface, no build step): all seven fields come back."""
    class Str:
        def __init__(self, data=""):
            self.data = data

    msg = Link(t=1.5, quality=0.42, rssi_dbm=-72.0, ap=AP, up=False, dropped=7, latency_ms=43.0)
    text = ros_bridge.to_ros({"String": Str}, "link", msg).data
    back = ros_bridge.from_ros("link", Str(data=text))
    assert (back.t, back.quality, back.rssi_dbm, back.ap) == (1.5, 0.42, -72.0, AP)
    assert (back.up, back.dropped, back.latency_ms) == (False, 7, 43.0)
    assert "ap" in json.loads(text) and "rssi_dbm" in json.loads(text)


def test_the_student_side_reads_the_link_without_a_second_terminal():
    from mecanum_lab.robot_io import RobotIO
    rob = RobotIO("muster", bus=StubBus())
    assert rob.link() is None, "no message yet has to read as nothing, not as a good link"
    rob.bus.publish(topic("link", "muster"), Link(t=1.0, quality=0.3, rssi_dbm=-74.0, ap=AP))
    assert rob.link().quality == 0.3 and rob.link().ap == AP
    assert rob.link().up is True and rob.link().dropped == 0


# --------------------------------------------------------------------- what the window shows


class StubRenderer:
    """The bit of `Renderer` the network layer looks at: an engine and a config."""

    def __init__(self, engine=None):
        self.engine, self.cfg = engine, getattr(engine, "cfg", None)


def test_the_network_layer_is_empty_without_a_radio():
    eng = sim(load_config(None, {"gui": False}))
    rend = StubRenderer(eng)
    assert overlays.link_readout(rend, eng.robots["muster"]) == []
    assert overlays.access_point(rend) is None
    assert overlays.autonomy_mark(rend, eng.robots["muster"]) is None   # draws nothing, needs no screen


def test_the_readout_line_carries_the_five_numbers_of_the_radio():
    eng = sim(config(shadow_db=0.0))
    robot = eng.robots["muster"]
    for _ in range(int(6.0 * eng.cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.5))
        step_once(eng)
    health = eng.link_health("muster")
    line = overlays.link_readout(StubRenderer(eng), robot)[0][0]
    assert line == overlays.link_text(health)
    assert line.startswith(f"wifi q {health[0]:.2f} {health[1]:.1f} dBm"), line
    assert "free" in line and "lost " in line and line.endswith("ms"), line
    place(eng, (15.55, 5.65))                                # two racks away: the same line, darker
    for _ in range(int(2.0 * eng.cfg["rate"])):
        step_once(eng)
    dark = overlays.link_readout(StubRenderer(eng), robot)[0][0]
    assert dark.startswith("wifi DOWN") and "2 walls" in dark, dark


def test_the_network_layer_is_a_menu_row_and_a_key():
    assert {attribute: key for attribute, _text, key in render.menue.LAYERS}["show_network"] == "n"
    assert render.KEYS["n"] == ("show_network", "")
    assert "n" in render.keys.layer_hint().split(), "the key is promised nowhere but the panel"


def test_the_readout_says_who_last_commanded_the_robot():
    """A node and the keyboard share `/cmd_vel`, so the answer is a timestamp — and it is on screen.

    `cmd topic` while the freshest frame came from the bus, `cmd keys` while the window's own keys
    were the last publisher (`node.run_loop` notes that, and only then), `cmd none` after
    `cmd_timeout` without any frame. The precedence itself is not a dict order: a node publishes every
    tick and the keys publish only while one is held, so the keys interrupt and never replace.
    """
    eng = sim(load_config(None, {"gui": False}))
    robot = eng.robots["muster"]
    rend = StubRenderer(eng)
    assert overlays.command_readout(rend, robot)[0][0] == "cmd none"
    eng.set_cmd_vel("muster", Twist(vx=0.2))
    step_once(eng)
    assert overlays.command_readout(rend, robot)[0][0].startswith("cmd topic ")
    eng.note_keys("muster")
    step_once(eng)
    assert overlays.command_readout(rend, robot)[0][0].startswith("cmd keys ")
    for _ in range(int(0.5 * eng.cfg["rate"])):            # command watchdog outlives every source
        eng.drain()
        eng.step(eng.sub_step)
    assert eng.robots["muster"].mode == "pass-through"


# ---------------------------------------------------------------- `./lab run` and its one seam


MINI_NODE = '''
"""Five lines of student node: one throttle, nothing else. Written by tests/test_wifi_w6.py."""


def drive(rob):
    while rob.running():
        rob.publish_cmd_vel(0.25)
        rob.spin(0.02)
'''


def lab_run(tmp_path, monkeypatch, cli: list):
    """Run the handout's own command in-process; hand back its measurement log and its bus.

    Deliberately not a harness that calls `node.subscribe()` first: the line whose absence this tests
    is that very call, so the test has to enter through `cmd_run()` like a student does. Paced at
    `--speed`, because the node is a thread that needs wall time to publish in; every assertion the
    caller makes is about the `t` and the positions in the CSV, which are simulation values.
    """
    from mecanum_lab import stub as stub_module
    monkeypatch.delenv("MECANUM_FAST", raising=False)
    script = tmp_path / "mini_node.py"
    script.write_text(MINI_NODE, encoding="utf-8")
    log = tmp_path / "lab_run.csv"
    bus = StubBus()
    stub_module.set_bus(bus)
    try:
        args = node.parser().parse_args([*cli, "--controller", str(script), "--log", str(log)])
        assert node.cmd_run(args) == 0                # ./lab run, dispatched by main() on argv[0]
    finally:
        bus.shutdown()                       # ends the node's `while rob.running()` loop
        stub_module.set_bus(None)
    with open(log, encoding="utf-8") as fh:
        return list(csv.DictReader(fh, delimiter=";")), bus


def test_a_node_under_lab_run_reaches_the_robot(tmp_path, monkeypatch):
    """`./lab run --controller` drives the robot — once it did not, and nothing said so.

    `cmd_run()` started the node and ran the loop but never subscribed the topics, so every frame the
    node published stayed on the bus: the robot stood at its spawn pose for the whole run while the GPS
    fixed away merrily beside it. Two assertions, the first on the seam itself, the second on the log
    a student would have read in vain.
    """
    rows, bus = lab_run(tmp_path, monkeypatch, ["--world", "production", "--robot", "muster",
                                                "--headless", "--speed", "4", "--seconds", "4"])
    assert "/muster/cmd_vel" in bus.topics(), "nothing ever subscribed the node's commands"
    moved = float(rows[-1]["x_odom"]) - float(rows[0]["x_odom"])
    assert 0.5 < moved < 2.0, f"0.25 m/s for 4 s of simulation time drove {moved:.2f} m"
    assert float(rows[-1]["t"]) > 3.5, "the run ended before the drive"


def test_the_radio_demo_command_goes_through_the_same_path(tmp_path, monkeypatch):
    """The README line for the radio, run for real: node, engine, one access point between them."""
    rows, _bus = lab_run(tmp_path, monkeypatch, ["--world", "production",
                                                 "--config", DEMO, "--robot", "muster",
                                                 "--headless", "--speed", "6", "--seconds", "20"])
    moved = math.dist((float(rows[0]["x_odom"]), float(rows[0]["y_odom"])),
                      (float(rows[-1]["x_odom"]), float(rows[-1]["y_odom"])))
    assert moved > 4.0, f"the example got {moved:.2f} m from the spawn"
    assert float(rows[-1]["t"]) > 19.0


# ------------------------------------------------------------------ the example controller


def example_module():
    spec = importlib.util.spec_from_file_location("link_autonomy_example", EXAMPLE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def run_example(monkeypatch, seconds: float = 45.0, seed: int = 7, **wifi_over):
    """The example node against one engine, stepped as fast as the CPU allows and no faster.

    `MECANUM_FAST` plus `fixed_step` is what `./lab run --fixed-step` sets: the in-process bus stops
    sleeping and the physics keeps its 1/50 s step, so this is the same measurement series as the
    real-time run of the README command — only sooner, and with every assertion below free to count
    simulation seconds.
    """
    monkeypatch.setenv("MECANUM_FAST", "1")
    cfg = config(**wifi_over)
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=seed)
    bus = StubBus()
    eng.spawn("muster")
    node.subscribe(bus, eng, "muster")
    node.wire_task(bus, eng, robot="muster")
    track = []
    bus.sub("link", "muster", lambda m: track.append((eng.t, m.quality, m.up, m.dropped)))
    node.add_node(EXAMPLE, "muster", bus)
    node.run_loop(eng, bus, None, (), seconds, hz=60.0, fixed_step=True)
    return eng, eng.robots["muster"], track


def test_the_example_is_a_drive_and_not_a_task():
    """`serve()` runs drive(rob); and it must brake before the simulator has to."""
    module = example_module()
    assert callable(getattr(module, "drive", None))
    assert not hasattr(module, "inverse_kinematics"), "a pass-through drive has no kinematics"
    assert module.Q_BRAKE > cfg_get(load_config(), "wifi.link_up_q"), \
        "a controller that waits for the failsafe has already lost every command it sends"


def test_the_example_drives_itself_out_of_coverage_and_says_where(monkeypatch):
    eng, robot, track = run_example(monkeypatch, seconds=60.0)
    assert robot.distance > 8.0, "it never went anywhere"
    assert robot.contacts == 0, "it found a rack instead of a shadow"
    assert track, "the example never saw a /link message"
    worst = min(quality for _t, quality, _u, _d in track)
    assert worst < 0.45, f"it never walked into the shadow, lowest q was {worst:.2f}"
    assert abs(robot.twist.vx) < 0.02 and abs(robot.twist.omega) < 0.02, "it is still rolling"
    assert eng.t == pytest.approx(60.0, abs=0.1)


def test_the_example_stops_when_the_link_dies_and_is_not_started_by_a_command(monkeypatch):
    """A failsafe level above the controller's own brake: here the radio stops the robot.

    `link_up_q` 0.5 is above the 0.35 the example brakes at, and `rate` 50 samples the link every
    physics step instead of every 200 ms — otherwise the publish rate alone would blur the 1.5 s the
    first assertion is about. The node runs in its own thread, so *which* frame lands in *which* step
    is scheduling and not the seed: that is why the last three lines compare the robot's last command
    with itself after the rescue attempt instead of naming a number, and why the drive gets the same
    60 s budget the README command gives it.
    """
    eng, robot, track = run_example(monkeypatch, seconds=60.0, link_up_q=0.5, rate=50.0,
                                    shadow_period=600.0)
    assert track, "the example never saw a /link message"
    low = [t for t, quality, _u, _d in track if quality < 0.5]
    down = [t for t, _q, up, _d in track[1:] if not up]
    worst = min(quality for _t, quality, _u, _d in track)
    assert low and down, f"60 s of driving never took the link down, lowest q {worst:.2f}"
    assert robot.mode == "autonomy", robot.mode
    assert abs(down[0] - low[0] - 1.5) < 3 * STEP, \
        f"the failsafe took {down[0] - low[0]:.2f} s of sim time"
    place, refused = (robot.pose.x, robot.pose.y), (robot.vel_cmd.vx, robot.vel_cmd.omega)
    for _ in range(int(4.0 * eng.cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=-0.5))            # the rescue attempt
        step_once(eng)
    assert math.dist(place, (robot.pose.x, robot.pose.y)) < 0.05, "it moved after the link died"
    assert (robot.vel_cmd.vx, robot.vel_cmd.omega) == refused, "the rescue command got through"
    assert robot.mode == "autonomy"


# ------------------------------------------------------------------ the coverage map of layer `c`


def test_the_coverage_map_is_the_budget_of_every_cell_of_the_hall():
    """One spot per cell of the world, each the number `budget()` gives at that spot — nothing added.

    The layer is allowed to exist only if it is the model and not a picture of a distance: a student
    who reads a square and echoes `/link` must get the same quality, and the two are the same function.
    """
    hall = radio()
    step, (wide, high) = hall.world.cell, hall.world.size
    grid = hall.coverage()
    assert len(grid) == int(wide / step) * int(high / step) == 960, \
        f"{wide}×{high} m at {step} m is one spot per cell"
    assert all(0.0 < x < wide and 0.0 < y < high for x, y, _q, _w in grid), "none on the border line"
    for x, y, q, walls in grid[::37]:
        _rssi, drawn, _d, drawn_walls = hall.budget(x, y)
        assert (q, walls) == (drawn, drawn_walls), f"({x}, {y}): the map and the budget disagree"


def test_the_coverage_map_is_the_room_and_not_the_weather():
    """Sampled without the slow shadow fade, because the window caches it for the whole run.

    The fade is a few dB walking in and out of a corridor (§6.14). A map that carried it would be a
    minute out of date the moment it was drawn, and a cache keyed on the room would be wrong; so the
    map answers "what does this room do to a signal" and the bar of each robot carries the moment.
    """
    grid = radio().coverage()
    assert max(q for _x, _y, q, _w in grid) == 1.0, "at the access point the room has no say"
    assert min(q for _x, _y, q, _w in grid) == 0.0, "and behind a rack nothing usable arrives"
    hall = radio()
    open_at_eight, faded = hall.budget(9.0, 2.0), hall.budget(9.0, 2.0, shadow_db=-6.0)
    assert faded[1] < open_at_eight[1] < 1.0, \
        f"the fade is real ({open_at_eight[1]:.2f} → {faded[1]:.2f}) — the map just does not carry it"


def test_the_map_shows_the_rack_and_not_only_the_distance():
    """Same distance, one rack in the way: 0.67 becomes 0.32. Measured on the shipped hall.

    Measured, `config/demo_wifi.json` over `production` (AP at 1, 2): every cell 8 ± 0.3 m from the
    access point, split by whether the straight line to it crosses a rack. Without this difference the
    layer would be a distance colormap and would teach the inverse-square law nobody taught.
    """
    hall = radio()
    near = [(x, y, q, w) for x, y, q, w in hall.coverage()
            if abs(math.hypot(x - hall.ap[0], y - hall.ap[1]) - 8.0) < 0.3]
    free = [q for _x, _y, q, w in near if w == 0]
    behind = [q for _x, _y, q, w in near if w == 1]
    assert free and behind, f"8 m ring: {len(free)} open, {len(behind)} behind a rack"
    assert max(free) == pytest.approx(0.68, abs=0.03), f"open floor at 8 m: {max(free):.2f}"
    assert min(behind) == pytest.approx(0.32, abs=0.03), f"one rack at 8 m: {min(behind):.2f}"
