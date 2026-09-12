"""GPS shadow zones (`gps.zones`) and the three overlays that draw them.

The zones are a demo feature: the default is `[]`, so the first test below is the important one —
with the default config the GPS must produce exactly the numbers it produced before zones existed.
The rest covers what a zone does to the fix, and that a broken entry is dropped instead of
ending a lab run.
"""
import math
import os
import statistics

import pytest

from mecanum_lab import overlays
from mecanum_lab.engine import SimEngine
from mecanum_lab.sensors import GpsSensor, Noise
from mecanum_lab.types import Odom, Pose, cfg_get, load_config
from mecanum_lab.worlds import load_world

CFG = load_config()
SIGMA = cfg_get(CFG, "gps.sigma_xy")


def konfig(name: str) -> str:
    """Path to a file in config/ — the demo file is part of what this test protects."""
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        "config", name)


def messwert(x=3.0, y=2.0, theta=0.4, zonen=None, sigma=SIGMA, seed=4):
    cfg = {"sigma_xy": sigma, "sigma_theta": 0.0}
    if zonen is not None:
        cfg["zones"] = zonen
    sensor = GpsSensor(Noise(seed), cfg)
    return sensor, sensor.fix(Pose(x, y, theta), t=1.0)


def streuung(zonen, pose=(3.0, 2.0), sigma=0.05, n=400):
    sensor = GpsSensor(Noise(11), {"sigma_xy": sigma, "sigma_theta": 0.0, "zones": zonen})
    punkte = [sensor.fix(Pose(*pose, 0.0), t=1.0) for _ in range(n)]
    return statistics.pstdev([p.x for p in punkte]), statistics.pstdev([p.y for p in punkte])


# --------------------------------------------------------------------------- default = nothing


def test_ohne_zonen_messt_der_gps_sensor_genau_wie_vorher():
    """No key, an empty list and a broken entry must all give the identical stream."""
    ohne_key = GpsSensor(Noise(7), {"sigma_xy": 0.5, "sigma_theta": 0.1})
    leer = GpsSensor(Noise(7), {"sigma_xy": 0.5, "sigma_theta": 0.1, "zones": []})
    kaputt = GpsSensor(Noise(7), {"sigma_xy": 0.5, "sigma_theta": 0.1,
                                  "zones": [{"name": "typo"}, {"rect": [1, 2, 3]}]})
    messwerte = {name: sensor.fix(Pose(1.0, 2.0, 0.5), 3.0)          # each stream exactly once
                 for name, sensor in (("ohne", ohne_key), ("leer", leer), ("kaputt", kaputt))}
    assert messwerte["leer"] == messwerte["ohne"]
    assert messwerte["kaputt"] == messwerte["ohne"]
    assert cfg_get(load_config(), "gps.zones") == []          # the lab default stays off


def test_demo_konfiguration_liefert_drei_zonen():
    cfg = load_config(konfig("demo_gps_shadow.json"))
    assert len(cfg["gps"]["zones"]) == 3
    assert GpsSensor(Noise(1), cfg["gps"]).zones[2]["block"] is True


# -------------------------------------------------------------------------------------- effects


def test_zone_erhoeht_nur_den_fehler_nicht_den_mittelwert():
    aussen = streuung([])
    zone = [{"name": "regal", "rect": [2.0, 1.0, 4.0, 3.0], "sigma_scale": 6}]
    innen = streuung(zone)
    assert aussen[0] == pytest.approx(0.05, abs=0.01)           # baseline is the plain sigma
    for a, b in zip(aussen, innen):
        assert b > 3 * a, f"{a} -> {b}"                        # 6x sigma must be visible
    sensor = GpsSensor(Noise(21), {"sigma_xy": 0.05, "sigma_theta": 0.0, "zones": zone})
    xs = [sensor.fix(Pose(3.0, 2.0, 0.0), 1.0).x for _ in range(300)]
    assert statistics.fmean(xs) == pytest.approx(3.0, abs=0.03)  # noise stays unbiased, no shift


def test_bias_zone_verraet_die_richtung_des_reflektors():
    sensor = GpsSensor(Noise(3), {"sigma_xy": 0.02, "sigma_theta": 0.0, "zones": [
        {"name": "wand", "rect": [2.0, 1.0, 4.0, 3.0], "bias_xy": [0.8, -0.5]}]})
    xs = [sensor.fix(Pose(3.0, 2.0, 0.0), 1.0).x for _ in range(300)]
    ys = [sensor.fix(Pose(3.0, 2.0, 0.0), 1.0).y for _ in range(300)]
    assert statistics.fmean(xs) == pytest.approx(3.8, abs=0.02)
    assert statistics.fmean(ys) == pytest.approx(1.5, abs=0.02)


def test_blockzone_liefert_keine_messung():
    sensor, fix = messwert(zonen=[{"name": "ladedock", "rect": [2.0, 1.0, 4.0, 3.0],
                                   "block": True}])
    assert fix is None
    assert sensor.fix(Pose(5.0, 2.0, 0.0), 1.0) is not None    # one metre further: fix again
    assert sensor.fix(Pose(4.0, 1.0, 0.0), 1.0) is None        # the edge still counts as inside


def test_erste_zone_gewinnt_und_rechtecke_werden_sortiert():
    sensor = GpsSensor(Noise(1), {"sigma_xy": 0.1, "zones": [
        {"name": "dense", "rect": [3.5, 2.5, 1.5, 0.5], "block": True},      # x0>x1, y0>y1
        {"name": "grosse", "rect": [0.0, 0.0, 10.0, 10.0], "sigma_scale": 2}]})
    assert sensor.zones[0]["rect"] == (1.5, 0.5, 3.5, 2.5)     # normalised, not swapped away
    assert sensor.fix(Pose(3.0, 2.0, 0.0), 1.0) is None        # the first match is the blackout
    assert sensor.fix(Pose(6.0, 6.0, 0.0), 1.0) is not None    # then the second one applies


def test_broken_zone_entries_are_dropped_not_fatal():
    sensor = GpsSensor(Noise(1), {"sigma_xy": 0.1, "zones": [
        "not a dict", {"rect": [1, 2, 3]}, {"rect": ["a", 1, 2, 3]},
        {"name": "ok", "rect": [1.0, 1.0, 2.0, 2.0], "sigma_scale": -5}]})
    assert [z["name"] for z in sensor.zones] == ["ok"]
    assert sensor.zones[0]["sigma_scale"] == 1.0               # negative nonsense cannot shrink σ
    assert sensor.fix(Pose(1.5, 1.5, 0.0), 1.0) is not None


# ------------------------------------------------------------------------------- the overlays


@pytest.fixture
def renderer():
    from mecanum_lab.render import Renderer
    cfg = load_config(overrides={"gps": {"zones": [
        {"name": "regal", "rect": [2.0, 1.0, 6.0, 3.0], "sigma_scale": 4},
        {"name": "dock", "rect": [8.0, 1.0, 9.0, 2.0], "block": True}]}})
    welt = load_world("production", cfg=cfg)
    eng = SimEngine(welt, cfg, seed=3)
    eng.spawn("alice")
    rend = Renderer(eng, cfg)
    try:
        yield rend
    finally:
        rend.close()


def test_overlays_zeichnen_ohne_etwas_zu_veraendern(renderer):
    rob = renderer.engine.robots["alice"]
    vorher = (rob.pose.x, rob.pose.y, rob.spec.rgb[0])
    rob.odom = Odom(t=1.0, x=rob.pose.x + 0.9, y=rob.pose.y - 0.4, theta=rob.pose.theta)
    renderer.draw()
    overlays.zones(renderer)                                   # both layers on by default
    rob.chassis.touching = True
    rob.wheels = [8.0] * 4
    overlays.skid_marks(renderer, rob, 0.05)
    assert renderer.skid_spur, "slipping wheels must leave rubber"
    renderer.draw()
    overlays.odom_ghost(renderer, rob)
    rob.chassis.touching = False
    for _ in range(80):
        overlays.skid_marks(renderer, rob, 0.05)               # 4 s later the floor is clean
    assert renderer.skid_spur == []
    assert (rob.pose.x, rob.pose.y, rob.spec.rgb[0]) == vorher  # drawing draws, nothing else
    assert overlays.zonen_im_lauf(renderer)[1]["block"] is True  # read from the live profile


def test_odometrie_start_in_der_spawnpose_und_nicht_bei_null():
    """The odometry ghost is only honest when the odometer starts where the robot was put.

    `set_task` and `reset_robot` always reset it; `spawn` did not, so a robot spawned at
    (7.5, 3.5) reported its own position as (0, 0) plus travel — and the drift overlay showed a
    8 m error while the robot drove a straight line.
    """
    cfg = load_config()
    eng = SimEngine(load_world("arena", cfg=cfg), cfg, seed=2)
    rob = eng.spawn("penelope", pose=Pose(7.5, 3.5, 1.0))
    assert (rob.odometer.pose.x, rob.odometer.pose.y, rob.odometer.pose.theta) == (7.5, 3.5, 1.0)
    for _ in range(60):
        eng.step(1 / 60.0)
    assert rob.odom is not None                            # stands still, stays where it is
    assert math.dist((rob.odom.x, rob.odom.y), (7.5, 3.5)) < 0.05
