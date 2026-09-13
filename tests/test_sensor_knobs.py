"""Two config keys that existed but nothing read: `gps.delay_ticks`/`kf.gps_delay` and
`lidar.max_walls`. Both are now what the robot actually experiences, so both are tested there.

The delay is the interesting one: CONTRACT-KF tells the students their filter has to live with
`delay_ticks >= 1`, and until the fix came out of a ring buffer there was nothing to simulate that
with. The rule that makes it a *delay* rather than a *wrong position* is that the message keeps the
stamp it was generated with — a filter can see `now - fix.t` and predict over it.
"""
import math

from mecanum_lab import sensors
from mecanum_lab.engine import SimEngine
from mecanum_lab.types import Pose, Rect, Twist, World, cfg_get, load_config
from mecanum_lab.worlds import load_world, list_worlds

RATE = 5.0                     # GPS emissions per sim second in these tests
STEP = 0.02                    # and the physics step of the engine tests


def gps_sensor(delay: int = 0, seed: int = 7, **extra) -> sensors.GpsSensor:
    cfg = {"rate": RATE, "sigma_xy": 0.0, "sigma_theta": 0.0, "delay_ticks": delay}
    cfg.update(extra)
    return sensors.GpsSensor(sensors.Noise(seed), cfg)


def engine_with(overrides: dict, world: str = "arena", seed: int = 2) -> SimEngine:
    cfg = load_config(None, overrides)
    eng = SimEngine(load_world(world, cfg=cfg), cfg, seed=seed)
    eng.spawn("muster")
    return eng


def gps_stamps(eng: SimEngine, seconds: float, drive: bool = True) -> list:
    """Every GPS message the engine emits over the next `seconds`, as (delivery_t, stamp)."""
    got = []
    delivery = eng.t
    for _ in range(int(seconds / eng.sub_step)):
        if drive:
            eng.set_cmd_vel("muster", Twist(vx=0.3))
        eng.step(eng.sub_step)
        delivery = eng.t
        for kind, _robot, payload in eng.drain():
            if kind == "gps":
                got.append((delivery, payload.t, payload))
    return got


# ------------------------------------------------------------------ GPS delay (delay_ticks)


def test_without_a_delay_the_fix_is_stamped_with_the_time_it_was_measured():
    sensor = gps_sensor()
    fix = sensor.fix(Pose(1.0, 2.0, 0.0), t=0.6)
    assert fix.t == 0.6 and fix.x == 1.0


def test_a_delayed_fix_arrives_n_emissions_late_and_stays_the_old_position():
    """The point of the ring buffer: what comes out at t=0.4 is what was measured at t=0.0."""
    sensor = gps_sensor(delay=2)
    emits = [(0.0, 1.0), (0.2, 2.0), (0.4, 3.0), (0.6, 4.0)]
    out = [sensor.fix(Pose(x, 0.0), t=t) for t, x in emits]
    assert out[0] is None and out[1] is None           # the buffer is still filling
    assert out[2].t == 0.0 and out[2].x == 1.0         # first fix: measured at the start
    assert out[3].t == 0.2 and out[3].x == 2.0


def gps_series(delay: int, seconds: float = 2.0) -> list:
    """The GPS series as the robot sees it, one entry per message: (stamp, x)."""
    eng = engine_with({"gui": False, "gps": {"delay_ticks": delay, "rate": RATE}})
    return [(stamp, round(fix.x, 9)) for _delivery, stamp, fix in gps_stamps(eng, seconds)]


def test_the_engine_delivers_the_fix_n_emissions_after_it_measured_it():
    """A delay shifts the whole series; it does not shorten or bend it."""
    plain, delayed = gps_series(0), gps_series(3)
    assert delayed == plain[:-3], "the delayed run must deliver exactly the older fixes"
    assert len(delayed) == len(plain) - 3             # the buffer fills before anything arrives


def test_the_message_keeps_the_stamp_it_was_generated_with():
    """If the engine re-stamped it, the receiver could not tell a late fix from a live one."""
    eng = engine_with({"gui": False, "gps": {"delay_ticks": 3, "rate": RATE}})
    got = gps_stamps(eng, 2.0)
    assert got
    _delivery, stamp, _fix = got[-1]
    # One physics step of that lag is bookkeeping, not delay: the engine stamps a measurement with
    # the sim instant it was taken at, which is one step before the `eng.t` the loop reports.
    assert abs((got[-1][0] - stamp) - (3 / RATE + eng.sub_step)) < 1e-9


def test_a_late_fix_is_still_measured_where_the_robot_was():
    """One second later the robot has moved; the fix still reports the metres it passed."""
    eng = engine_with({"gui": False, "gps": {"delay_ticks": 5, "rate": RATE}})
    got = gps_stamps(eng, 4.0)
    _delivery, _stamp, fix = got[-1]
    r = eng.robots["muster"]
    lag_metres = abs(fix.x - r.pose.x)
    assert 0.2 < lag_metres < 0.4, f"1 s of delay at 0.3 m/s, measured lag {lag_metres:.3f} m"


def test_kf_gps_delay_is_the_same_delay_in_seconds():
    """`kf.gps_delay` is the student-facing name (seconds); the engine turns it into ticks."""
    eng = engine_with({"gui": False, "kf": {"gps_delay": 0.4}, "gps": {"rate": RATE}})
    assert eng._gps.delay_ticks == round(0.4 * RATE) == 2      # 0.4 s at 5 Hz = two emissions
    assert cfg_get(eng.cfg, "gps.delay_ticks") == 2     # and /sim/config can tell the students
    plain, delayed = gps_series(0, 1.0), gps_series(2, 1.0)
    assert delayed == plain[:len(delayed)]


def test_the_gap_is_still_read_at_the_time_the_fix_was_made():
    """An outage must not be smeared by the delay: no fix whose *emission* lay inside the gap."""
    eng = engine_with({"gui": False, "gps": {"delay_ticks": 2, "rate": RATE, "gap": [1.0, 1.0]},
                       "debug_truth": False})
    eng.set_task("kf_fusion")                     # t0 = now, so the gap is 1.0 … 2.0 s from here
    got = gps_stamps(eng, 3.0)
    assert got, "the GPS went away completely"
    for _delivery, stamp, _fix in got:
        assert not 1.0 <= stamp - eng.t_task < 2.0


def test_a_reset_empties_the_buffer():
    """After reset() no fix from the previous run may arrive."""
    eng = engine_with({"gui": False, "gps": {"delay_ticks": 5, "rate": RATE}})
    gps_stamps(eng, 4.0)
    eng.reset()
    got = gps_stamps(eng, 0.6)
    assert all(stamp >= eng.t - 1e-9 for _delivery, stamp, _fix in got)


# ---------------------------------------------------------------------- LIDAR max_walls


def ring_world(n: int, radius: float = 2.0) -> World:
    """`n` one-metre walls in a ring around the origin, in world order — one per beam of an
    8-beam scan, so `beam i points at wall i` is checkable without a diagram."""
    walls = []
    for i in range(n):
        a = 2 * math.pi * i / n
        cx, cy = radius * math.cos(a), radius * math.sin(a)
        walls.append(Rect(cx - 0.25, cy - 0.25, cx + 0.25, cy + 0.25))
    return World(name="ring", walls=walls, size=(2 * radius + 2, 2 * radius + 2))


def scan_of(max_walls: int, n: int = 8):
    lidar = sensors.Lidar(ring_world(n), sensors.Noise(1),
                          {"beams": 8, "range_max": 8.0, "sigma": 0.0, "max_walls": max_walls})
    return lidar, lidar.scan(Pose(0.0, 0.0, 0.0))


def test_an_uncapped_scan_sees_every_wall_nearby():
    lidar, scan = scan_of(400)
    assert lidar.max_walls == 400
    assert all(r < 8.0 for r in scan.ranges), scan.ranges


def test_the_cap_is_what_the_robot_sees():
    """With max_walls=2 the robot has two segments and drives through the other six as if air."""
    lidar, scan = scan_of(2)
    hits = [i for i, r in enumerate(scan.ranges) if r < 8.0]
    assert hits == [0, 1], f"the cap was not what the robot saw: {hits}"
    assert all(math.isinf(scan.ranges[i]) for i in range(2, 8))


def test_a_smaller_cap_hides_more_of_the_room():
    _lidar, wide = scan_of(400)
    _lidar, narrow = scan_of(4)
    assert math.isinf(narrow.ranges[5]) and wide.ranges[5] < 8.0   # wall 5 is past the cap
    assert narrow.ranges[0] == wide.ranges[0]         # what is seen is seen unchanged


def test_no_world_in_the_repository_is_capped_by_the_default():
    """The default must stay above every wall count, or the graded LIDAR task would change."""
    biggest = max(len(load_world(w).walls) for w in list_worlds())
    assert cfg_get(load_config(), "lidar.max_walls") > biggest


# --------------------------------------------------------------- the geometry helper itself


def test_odom_geometry_returns_the_true_object_when_nothing_is_mis_measured():
    from mecanum_lab import physics
    g = physics.Geometry(lx=0.14, ly=0.13, r=0.05)
    assert sensors.odom_geometry(g, None) is g
    assert sensors.odom_geometry(g, {"lever_scale": 1.0}) is g
    scaled = sensors.odom_geometry(g, {"lever_scale": 0.9})
    assert scaled is not g and abs(scaled.arm - 0.9 * g.arm) < 1e-12
