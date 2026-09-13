"""Sensor realism: quality, satellites, temperature, dropped messages, uneven stamps.

Everything here is a knob that is **off** in `DEFAULT_CONFIG`, so the test that matters most is the
first one: with the default config the sensors must produce the numbers they produced before these
knobs existed — not approximately, message for message. The rest covers what each knob does when a
demo file asks for it, and that a seeded run drops the same messages every time.

Measured values in the assertions come from the pre-W3 modules (the same drive run against a copy
of `sensors.py`/`engine.py`/`types.py` from before the knobs existed); the numbers in
`docs/CONTRACT.md` §6.4 and the README come from the same scripts.
"""
import csv
import hashlib
import math
import os
import statistics

import pytest

from mecanum_lab import overlays, sensors
from mecanum_lab.engine import SimEngine
from mecanum_lab.logbook import COLUMNS, Logbook
from mecanum_lab.types import (Gps, Imu, Pose, Rect, Scan, Twist, World, cfg_get, load_config)
from mecanum_lab.worlds import load_world

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STEP = 0.02                      # the 50 Hz physics step these drives are run at
DEMO = "config/demo_sensor_reality.json"
MEASURED = ("odom", "scan", "gps", "imu", "truth")
# The four fields W3 added to the messages. Comparing streams is about the numbers that already
# existed; the new fields have their own tests below.
NEW_FIELDS = {"quality", "sats", "missing", "temp"}
# Every odom/scan/gps/imu/truth message of the drive below, full precision, the four fields of W3
# left out; one line per message, each ending in a newline. Measured against the modules from
# before the knobs existed: 2337 messages, sha256 of that text.
OLD_STREAM_SHA = "5e3ff35e3bdd5c8367da9d460971702e3850fabe8986c485f5d9636c2789e4fc"
OLD_FIELDS = {                                   # three messages of that same drive, by name
    "odom 600": (12.596531842298187, 4.844210558459618, -0.0055394781769289025),
    "gps 25": (9.44757954770287, 4.878530702766589),
    "imu 999": (0.10543543806085989, -0.051272239591280716, 9.803257552161265),
}


def config(name: str) -> str:
    """Path to a file in config/ — the demo file is part of what these tests protect."""
    return os.path.join(REPO, name)


def canon(payload) -> str:
    """A measurement as text, full precision, the fields of W3 left out."""
    return payload.__class__.__name__ + "(" + ", ".join(
        f"{k}={v!r}" for k, v in sorted(vars(payload).items()) if k not in NEW_FIELDS) + ")"


def drive(cfg: dict, seconds: float = 12.0, seed: int = 3, world: str = "arena") -> list:
    """One fixed drive, every measurement as `(delivery_t, kind, robot, payload)` — always the same.

    With a turn rate that flips sign every second, so the drive is not a straight line and a change
    in the noise sequence cannot hide in a coordinate that happens to stay still.
    """
    eng = SimEngine(load_world(world, cfg=cfg), cfg, seed=seed)
    eng.spawn("muster")
    eng.set_task("kf_gps")
    got = []
    for i in range(int(seconds * cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.45, omega=0.05 * (1 if i % 100 < 50 else -1)))
        eng.step(eng.sub_step)
        for kind, robot, payload in eng.drain():
            if kind in MEASURED:
                got.append((eng.t, kind, robot, payload))
    return got


def text_of(got: list) -> str:
    """The whole measurement series as text, one line per message — what the digest is taken of."""
    return "\n".join(f"{kind} {robot} {canon(payload)}" for _t, kind, robot, payload in got) + "\n"


# ------------------------------------------------------------- default config = old behaviour


def test_the_default_config_still_produces_the_old_streams():
    """Byte for byte: no new random number, no changed value, no shifted stamp.

    This is the guard for every knob in this package. A knob that is off may not so much as ask the
    generator a question — `Noise.chance(0.0)` and `Noise.late(0.0)` return without drawing for
    exactly that reason. The digest below was measured against the modules from before the knobs
    existed; if it changes, a default did, and every graded threshold belongs in the review.
    """
    got = drive(load_config(None, {"gui": False, "debug_truth": True}))
    assert hashlib.sha256(text_of(got).encode()).hexdigest() == OLD_STREAM_SHA
    by_kind: dict = {}
    for _t, kind, _robot, payload in got:
        by_kind.setdefault(kind, []).append(payload)
    assert len(by_kind["odom"]) == 600 and len(by_kind["gps"]) == 59
    assert (by_kind["odom"][599].x, by_kind["odom"][599].y,
            by_kind["odom"][599].theta) == OLD_FIELDS["odom 600"]
    assert (by_kind["gps"][24].x, by_kind["gps"][24].y) == OLD_FIELDS["gps 25"]
    assert (by_kind["imu"][998].ax, by_kind["imu"][998].gz,
            by_kind["imu"][998].az) == OLD_FIELDS["imu 999"]


def test_the_new_keys_change_nothing_while_they_are_off():
    """The same drive with every new key written out at its default value: the same series.

    Catches a key that is read even when it is switched off (a multiplier that is applied anyway, a
    satellite count that ends up in the fix) — each demo file's `_comment` promises that off means
    off, and only running both ways proves it.
    """
    off = {"odom": {"jitter": 0.0}, "lidar": {"reflectivity_min": 0.0},
           "gps": {"dropout": 0.0, "latency": 0.0, "sats": 8, "sats_min": 4},
           "imu": {"temp_start": 24.0, "temp_motor": 0.0, "temp_tau": 30.0,
                   "temp_walk": 0.0, "temp_walk_gyro": 0.0}}
    assert text_of(drive(load_config(None, dict(off, gui=False, debug_truth=True)))) == \
        text_of(drive(load_config(None, {"gui": False, "debug_truth": True})))


def test_the_poi_keys_change_nothing_while_no_source_is_planted():
    """The same test as above for the keys of W5: `pois` empty and `poi` at its defaults.

    A radiation detector is a sensor of its own and draws from this one stream, so the guard here is
    that the *keys* are inert while no source is planted — not that the topic is cheap. With a source
    in the world the streams do shift, which is why `config/tasks.json` names no source and
    `tests/test_world_poi_w5.py` asserts the shift instead of pretending it away.
    """
    off = {"pois": [], "poi": {"rate": 5.0, "d0": 1.0, "counts": 400.0,
                               "publish_distance": False}}
    assert text_of(drive(load_config(None, dict(off, gui=False, debug_truth=True)))) == \
        text_of(drive(load_config(None, {"gui": False, "debug_truth": True})))


# ------------------------------------------------------------------- quality and satellite count


def gps_at(**cfg) -> sensors.GpsSensor:
    return sensors.GpsSensor(sensors.Noise(5), cfg)


def test_open_floor_is_a_good_fix_with_every_anchor_in_view():
    sensor = gps_at(sigma_xy=0.0, sigma_theta=0.0, sats=8, sats_min=4)
    fix = sensor.fix(Pose(3.0, 2.0, 0.0), t=1.0)
    assert (fix.quality, fix.sats) == (2, 8)
    assert (fix.x, fix.y) == (3.0, 2.0)


def test_quality_goes_to_zero_in_the_blocking_zone_and_back():
    """`sky()` is the receiver's opinion about a place, so it answers while no message does."""
    sensor = gps_at(sigma_xy=0.0, sigma_theta=0.0, sats=8, zones=[
        {"name": "dock", "rect": [2.0, 1.0, 4.0, 3.0], "block": True}])
    assert sensor.sky(Pose(1.0, 2.0)) == (2, 8)
    assert sensor.sky(Pose(3.0, 2.0)) == (0, 0)                 # inside the dock: no fix
    assert sensor.fix(Pose(3.0, 2.0), 1.0) is None              # and it says nothing at all
    assert sensor.sky(Pose(5.0, 2.0)) == (2, 8)                 # one metre out: good again
    assert sensor.fix(Pose(5.0, 2.0), 1.0).quality == 2


def test_a_zone_that_hides_anchors_degrades_the_fix_rather_than_losing_it():
    sensor = gps_at(sigma_xy=0.0, sigma_theta=0.0, sats=8, sats_min=4, zones=[
        {"name": "rack", "rect": [2.0, 1.0, 4.0, 3.0], "sigma_scale": 3, "sats": 3}])
    fix = sensor.fix(Pose(3.0, 2.0), 1.0)
    assert (fix.quality, fix.sats) == (1, 3)
    without = sensors.GpsSensor(sensors.Noise(5), {"sigma_xy": 0.0, "sigma_theta": 0.0,
                                                  "zones": [dict(
                                                      name="rack", rect=[2.0, 1.0, 4.0, 3.0],
                                                      sigma_scale=3)]})
    assert without.fix(Pose(3.0, 2.0), 1.0).sats == 6           # sigma x3 costs 2 of the 8 anchors
    assert without.fix(Pose(3.0, 2.0), 1.0).quality == 1


def test_a_multi_path_bias_step_reports_a_fix_that_is_only_degraded():
    sensor = sensors.GpsSensor(sensors.Noise(2), {"sigma_xy": 0.0, "sigma_theta": 0.0,
                                                 "bias_step": [1.0, 2.0, 0.8, -0.5]})
    assert sensor.fix(Pose(3.0, 2.0), 0.5).quality == 2
    inside = sensor.fix(Pose(3.0, 2.0), 1.5)
    assert (inside.quality, round(inside.x - 3.0, 9), round(inside.y - 2.0, 9)) == (1, 0.8, -0.5)
    assert sensor.fix(Pose(3.0, 2.0), 3.5).quality == 2


# ----------------------------------------------------------- dropout: same seed, same holes


def drop_pattern(seed: int, n: int = 120, dropout: float = 0.2, **extra) -> str:
    sensor = sensors.GpsSensor(sensors.Noise(seed),
                               {"sigma_xy": 0.0, "sigma_theta": 0.0, "dropout": dropout, **extra})
    return "".join("." if sensor.fix(Pose(1.0, 1.0), i * 0.2, "muster") is not None else "x"
                   for i in range(n))


def test_the_dropout_pattern_is_the_seed_and_nothing_else():
    """Same seed -> the same holes. Different seed -> other holes. That is what makes it a demo."""
    assert drop_pattern(7) == drop_pattern(7)
    assert drop_pattern(7) != drop_pattern(8)
    holes = drop_pattern(7).count("x")
    assert 12 < holes < 40, f"20 % of 120 messages, measured {holes}"      # measured: 27


def test_the_dropout_count_is_kept_per_robot_and_starts_at_zero_again():
    cfg = {"sigma_xy": 0.0, "sigma_theta": 0.0, "dropout": 0.5}
    sensor = sensors.GpsSensor(sensors.Noise(3), cfg)
    for i in range(20):
        sensor.fix(Pose(1.0, 1.0), i * 0.2, "alice")
    assert sensor.drops.get("alice", 0) > 0 and "bob" not in sensor.drops
    sensor.reset()
    assert sensor.drops == {}


def test_a_dropped_message_takes_the_noise_of_a_message_nobody_received():
    """The coin is tossed **before** the fix is measured, so the holes of a run are the pattern of
    the seed and not a consequence of where the robot happened to be standing.

    Two very different drives with one seed lose a different *number* of messages only because of
    how many emissions the drive lasted to; the pattern itself never looks at the pose.
    """
    def holes(seed, seconds):
        sensor = sensors.GpsSensor(sensors.Noise(seed), {"sigma_xy": 0.05, "dropout": 0.4})
        return "".join("." if sensor.fix(Pose(1.0 + 0.1 * i, 1.0), i * 0.2, "muster") is not None
                       else "x" for i in range(int(seconds / 0.2)))

    assert holes(11, 12.0)[:60] == holes(11, 24.0)[:60]          # the same start, always
    assert holes(11, 12.0).count("x") > 10                       # 40 %, measured: 26 of 60


# ------------------------------------------------------------------ latency: stamp, not value


def gps_series(cfg: dict, n: int = 40, pose_step: float = 0.25) -> list:
    """The fixes one robot receives over `n` emissions, as (stamp, x) — x moves 0.25 m per fix."""
    sensor = sensors.GpsSensor(sensors.Noise(4), dict(cfg))
    out = []
    for i in range(n):
        fix = sensor.fix(Pose(i * pose_step, 0.0), i * 0.2)
        if fix is not None:
            out.append((fix.t, fix.x))
    return out


def test_latency_shifts_the_stamp_and_never_the_position():
    """The whole delayed series is the head of the undelayed one — a delay, not a wrong fix."""
    honest = {"sigma_xy": 0.0, "sigma_theta": 0.0}
    plain = gps_series(honest)
    late = gps_series({**honest, "latency": 0.35})
    assert late == plain[:len(late)]
    assert len(late) < len(plain), "with 0.35 s of wire something must still be on it"
    assert [round(x, 9) for _t, x in late] == [round(x, 9) for _t, x in plain[:len(late)]]


def test_latency_jitters_the_arrival_and_keeps_them_in_order():
    """latency is the middle of an interval, not a promise — and one fix at a time comes out."""
    sensor = sensors.GpsSensor(sensors.Noise(9), {"sigma_xy": 0.0, "latency": 0.4})
    arrived = []
    for i in range(40):
        fix = sensor.fix(Pose(float(i), 0.0), i * 0.2)
        if fix is not None:
            arrived.append((i * 0.2, fix))
    ages = [t_arrive - fix.t for t_arrive, fix in arrived[1:]]
    assert min(ages) > 0.2 - 1e-9, "a fix arrived before it was measured"
    assert max(ages) <= 0.4 * 1.5 + 0.2 + 1e-9, f"latency left its own window: {max(ages):.2f} s"
    assert len({round(a, 6) for a in ages}) > 1, "that is one constant lag, not a jitter"
    stamps = [fix.t for _a, fix in arrived]
    assert stamps == sorted(stamps), "a filter cannot predict backwards"


def test_a_reset_empties_the_wire_as_well_as_the_ring_buffer():
    """After `reset()` no position of the drive before it may turn up again."""
    sensor = sensors.GpsSensor(sensors.Noise(6), {"sigma_xy": 0.0, "latency": 0.5})
    for i in range(10):
        sensor.fix(Pose(float(i), 0.0), i * 0.2)
    sensor.reset()
    later = [sensor.fix(Pose(100.0 + i, 0.0), 10.0 + i * 0.2) for i in range(12)]
    got = [fix.x for fix in later if fix is not None]
    assert got, "after a reset nothing arrives at all"
    assert all(x >= 100.0 for x in got), f"fixes of the old drive came back: {got}"


# --------------------------------------------------------------------- IMU temperature, the bias


def imu_samples(cfg: dict, seconds: float = 6.0, speed: float = 0.6, seed: int = 3) -> list:
    sensor = sensors.ImuSensor(sensors.Noise(seed), cfg)
    out = []
    for _ in range(int(seconds / STEP)):
        out += sensor.sample(Pose(0.0, 0.0), Twist(vx=speed, omega=0.0), STEP)
    return out


def test_standing_still_the_imu_is_cold_and_reads_gravity():
    """Default config: no warm-up at all, and `az` stays at +9.81 — the statement of §6.4."""
    samples = imu_samples(cfg_get(load_config(), "imu"), speed=0.0)
    assert {s.temp for s in samples} == {24.0}
    assert abs(statistics.fmean(s.az for s in samples) - 9.81) < 0.1
    assert samples[-1].az > 9.5, "the IMU stopped measuring the floor it is standing on"

def test_the_chip_warms_up_while_the_robot_drives_and_cools_when_it_stands():
    cfg = dict(cfg_get(load_config(), "imu"), temp_motor=14.0, temp_tau=20.0)
    sensor = sensors.ImuSensor(sensors.Noise(3), cfg)
    driving = []
    for _ in range(int(40.0 / STEP)):
        driving += sensor.sample(Pose(0.0, 0.0), Twist(vx=0.6), STEP)
    warm = driving[-1].temp
    assert warm > 28.0, f"40 s of driving and the chip stayed at {warm:.1f} °C"
    assert all(b.temp >= a.temp for a, b in zip(driving, driving[1:]))     # monotone rise
    standing = []
    for _ in range(int(60.0 / STEP)):
        standing += sensor.sample(Pose(0.0, 0.0), Twist(), STEP)
    assert standing[-1].temp < warm - 5.0, "a robot that stands still does not keep heating"
    assert standing[-1].temp < 26.0


def test_the_bias_walk_follows_the_temperature_and_nothing_else():
    """Same seed, same noise, same bias random walk — only `temp_walk` differs, so the difference
    between the two runs *is* the thermal term."""
    base = dict(cfg_get(load_config(), "imu"), temp_motor=14.0, temp_tau=20.0)
    cold = imu_samples(dict(base, temp_walk=0.0), seconds=20.0)
    warm = imu_samples(dict(base, temp_walk=0.004), seconds=20.0)
    rise = warm[-1].temp - 24.0
    shift = statistics.fmean(s.az for s in warm[-200:]) - statistics.fmean(s.az for s in cold[-200:])
    assert 0.02 < shift < 0.20, f"the thermal term measured {shift:.4f} m/s^2"
    assert abs(shift - 0.004 * rise) < 0.02, f"{shift:.4f} against 0.004 * {rise:.2f}"


def test_the_temperature_is_reported_with_the_sample():
    """The chip's own reading belongs in the message: a student has to be able to see the drift."""
    cfg = dict(cfg_get(load_config(), "imu"), temp_motor=14.0)
    samples = imu_samples(cfg, seconds=1.0)
    assert samples[0].temp < 24.05                         # it starts out cold
    assert samples[-1].temp > 24.2                         # and 1 s of driving warms it visibly


# ---------------------------------------------------------------------------- LIDAR reflectivity


def long_wall() -> World:
    """One 30 m wall — the wall of the hall, and of every arena, drawn on its own."""
    return World(name="long", walls=[Rect(0.0, 0.0, 30.0, 0.2)], size=(30.0, 6.0))


def wall_scan(reflectivity: float, pose=Pose(5.0, 0.7, 0.0)) -> Scan:
    lidar = sensors.Lidar(long_wall(), sensors.Noise(1),
                          {"beams": 360, "range_max": 8.0, "sigma": 0.0,
                           "reflectivity_min": reflectivity})
    return lidar.scan(pose)


def test_a_wall_seen_straight_on_is_the_same_wall_with_and_without_reflectivity():
    plain, real = wall_scan(0.0), wall_scan(0.25)
    straight = 270                                       # the beam pointing at the wall (−90°)
    assert plain.ranges[straight] == pytest.approx(0.5)
    assert real.ranges[straight] == plain.ranges[straight]
    assert real.ranges[0] == plain.ranges[0]             # and the beam ahead of the robot


def test_a_grazing_ray_on_a_long_wall_disappears_when_reflectivity_is_on():
    """The wall is 0.5 m to the left and 30 m long. Looking along it, nothing comes back.

    The three angles below hit that wall 7.1, 3.6 and 2.4 m away — inside range, so the wall really
    is there — while the incidence is so shallow that a real receiver does not see the echo.
    """
    plain, real = wall_scan(0.0), wall_scan(0.25)
    for body_angle in (-4, -8, -12):                       # 4…12° down from parallel to the wall
        beam = body_angle % 360
        assert plain.ranges[beam] < 8.0, f"{body_angle}°: the wall is there and is reported"
        assert math.isinf(real.ranges[beam]), f"{body_angle}° still came back"
    assert real.missing > plain.missing


def test_reflectivity_shortens_the_reachable_end_of_the_wall_not_the_wall_itself():
    plain, real = wall_scan(0.0), wall_scan(0.25)
    far_plain = max(r for r in plain.ranges if r < 8.0)
    far_real = max(r for r in real.ranges if r < 8.0)
    assert far_plain == pytest.approx(7.17, abs=0.05)     # measured with the default
    assert far_real < 2.5                                 # measured: 1.93 m
    assert min(real.ranges) == min(plain.ranges)


def test_missing_counts_the_beams_that_came_back_with_nothing():
    """`inf` in `ranges` *and* a number: a clipped reading must not be mistaken for a wall."""
    scan = wall_scan(0.0)
    assert scan.missing == sum(1 for r in scan.ranges if math.isinf(r))
    assert scan.missing == 189                            # 360 - the beams that hit the wall
    assert scan.range_max == 8.0 and all(
        r <= scan.range_max for r in scan.ranges if not math.isinf(r))


def test_a_scan_of_a_real_arena_reports_its_beams_without_reflectivity():
    cfg = load_config()
    lidar = sensors.Lidar(load_world("production", cfg=cfg), sensors.Noise(2),
                          cfg_get(cfg, "lidar"))
    scan = lidar.scan(Pose(4.0, 4.0, 0.3))
    assert lidar.reflectivity_min == 0.0
    assert scan.missing == sum(1 for r in scan.ranges if math.isinf(r)) > 0


# --------------------------------------------------------------------------- odom stamp jitter


def odom_stamps(jitter: float, seconds: float = 10.0) -> list:
    """The odom messages of one straight drive, as (delivery_t, message)."""
    cfg = load_config(None, {"gui": False, "odom": {"jitter": jitter}})
    eng = SimEngine(load_world("arena", cfg=cfg), cfg, seed=2)
    eng.spawn("muster")
    got = []
    for _ in range(int(seconds * cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.4))
        eng.step(eng.sub_step)
        for kind, _robot, payload in eng.drain():
            if kind == "odom":
                got.append((eng.t, payload))
    return got


def test_jitter_makes_the_odom_stamps_uneven_and_their_order_untouched():
    got = [msg for _delivery, msg in odom_stamps(0.4)]
    gaps = [b.t - a.t for a, b in zip(got, got[1:])]
    assert max(gaps) - min(gaps) > 0.008, "the stamps stayed metronomic"
    assert min(gaps) > 0.0, "a message stamped before the one before it — a filter cannot cope"
    assert all(b.t > a.t for a, b in zip(got, got[1:]))


def test_jitter_only_moves_the_stamp_the_measurement_stays_on_the_grid():
    """A late report is still the same report: never stamped after the step that published it."""
    period = 1.0 / 50.0
    for delivery, payload in odom_stamps(0.4)[-200:]:
        latest = delivery - STEP                                  # the sim instant it was taken at
        assert payload.t <= latest + 1e-9
        assert payload.t >= latest - 0.4 * period - 1e-9


def test_without_jitter_the_stamps_are_exactly_what_they_were():
    got = [msg for _delivery, msg in odom_stamps(0.0)]
    assert {round(b.t - a.t, 9) for a, b in zip(got, got[1:])} == {STEP}


# --------------------------------------------------------------------- the demo file and its view


def test_the_demo_file_turns_on_exactly_the_four_knobs():
    cfg = load_config(config(DEMO))
    assert cfg_get(cfg, "gps.dropout") == 0.15 and cfg_get(cfg, "gps.latency") == 0.25
    assert cfg_get(cfg, "imu.temp_motor") > 0.0 and cfg_get(cfg, "imu.temp_walk") > 0.0
    assert cfg_get(cfg, "lidar.reflectivity_min") == 0.25
    assert cfg_get(cfg, "odom.jitter") == 0.35
    assert len(cfg_get(cfg, "gps.zones")) == 2                # for quality 1 and quality 0
    assert cfg_get(load_config(), "gps.dropout") == 0.0        # and the lab default stays clean
    for key in ("dropout", "latency", "sats", "sats_min"):
        assert key in cfg_get(load_config(), "gps")


def test_the_quality_scale_shows_up_across_one_straight_drive():
    """Open floor good, between the racks degraded, in the dock no fix — one drive, seed 1."""
    cfg = load_config(config(DEMO), {"gui": False})
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=1)
    eng.spawn("muster")
    eng.set_task("kinematik")
    seen: set = set()
    for _ in range(int(45.0 * cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.5))
        eng.step(eng.sub_step)
        seen.add(eng.gps_health("muster")[:2])
    assert (2, 8) in seen and (1, 3) in seen and (0, 0) in seen, seen
    assert eng.gps_health("muster")[2] > 0, "no message was ever dropped"


class SimpleRenderer:
    """The bit of `Renderer` that `overlays.sensor_readout()` looks at: an engine and a config."""

    def __init__(self, engine):
        self.engine, self.cfg = engine, engine.cfg


def test_the_readout_line_shows_quality_satellites_temperature_and_losses():
    """Without `ros2 topic echo`: the four numbers are in the line the student already reads."""
    cfg = load_config(config(DEMO), {"gui": False})
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=1)
    robot = eng.spawn("muster")
    for _ in range(int(6.0 * cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.5))
        eng.step(eng.sub_step)
    segments = dict(overlays.sensor_readout(SimpleRenderer(eng), robot))
    gps_line = next(t for t in segments if t.startswith("gps "))
    imu_line = next(t for t in segments if t.startswith("imu "))
    assert "q2 8 sats" in gps_line, gps_line
    assert "°C" in imu_line and "no imu" not in imu_line, imu_line
    assert "lost " in gps_line, f"dropout without a count: {gps_line}"


def test_the_logbook_records_the_new_fields(tmp_path):
    """`tools/kfplot.py` lists whatever is in the header, so the header is the contract."""
    cfg = load_config(config(DEMO), {"gui": False})
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=1)
    eng.spawn("muster")
    book = Logbook(eng, str(tmp_path / "messung.csv"), interval=0.5)
    path = book.path
    for _ in range(int(30.0 * cfg["rate"])):
        eng.set_cmd_vel("muster", Twist(vx=0.5))
        eng.step(eng.sub_step)
        for kind, robot, payload in eng.drain():
            book.tap(kind, robot, payload)
        book.tick()
    book.close()
    rows = list(csv.DictReader(open(path, encoding="utf-8"), delimiter=";"))
    assert {"q_gps", "sats_gps", "lost_gps", "temp_imu", "noecho_scan"} <= set(COLUMNS)
    assert {"q_gps", "temp_imu", "noecho_scan", "lost_gps"} <= set(rows[0])
    warm = [float(r["temp_imu"]) for r in rows if r["temp_imu"]]
    assert warm[-1] > warm[0] + 1.0, f"the chip never warmed: {warm[0]} -> {warm[-1]}"
    assert any(float(r["lost_gps"] or 0) > 0 for r in rows), "dropout without a trace in the log"
    assert any(float(r["noecho_scan"] or 0) > 0 for r in rows)
    assert float(rows[-1]["q_gps"]) in (0.0, 1.0, 2.0)


def test_a_message_that_does_not_carry_the_new_fields_still_reads_as_a_good_fix():
    """Old producers (the grader's synthetic GPS, a student's own node) must not look broken."""
    assert (Gps(t=1.0, x=1.0, y=2.0, theta=0.0).quality, Gps().sats) == (2, 8)
    assert Scan(t=0.0, ranges=[1.0, float("inf")]).missing == 0
    assert Imu().temp == 24.0
