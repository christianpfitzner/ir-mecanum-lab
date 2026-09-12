"""Regression tests for three findings of the read-the-code audit.

1. `tasks.welt_fuer` picked the arena with `max(set(...))`. Set order follows the string hash,
   which CPython randomises per process, so `--task beide` (four tasks in production, four in
   arena) graded a different arena in another run — same seed, different hall.
2. `types.Scan` promised "clockwise from front-left" while the lidar, the contract and the
   grading helper all use beam 0 forward and counter-clockwise — the central sign convention.
3. The GPS cross was drawn inside `_schaetzung`, which the frame loop only calls when the
   estimate layer is on: the "gps fix" layer silently did nothing whenever `k` was off.
"""
import os
import subprocess
import sys

import pygame
import pytest

from mecanum_lab import tasks
from mecanum_lab.engine import SimEngine
from mecanum_lab.render import GPS_FARBE, Renderer
from mecanum_lab.types import Gps, Pose, cfg_get, load_config
from mecanum_lab.worlds import load_world

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = tasks.load_tasks()


def sim_welt(seed: str, abfrage="beide") -> str:
    umgebung = dict(os.environ, PYTHONHASHSEED=seed)
    ausgang = subprocess.run(
        [sys.executable, "-c", "from mecanum_lab import tasks; "
         "print(tasks.welt_fuer(tasks.load_tasks(), %r))" % abfrage],
        capture_output=True, text=True, cwd=ROOT, env=umgebung, timeout=60)
    assert ausgang.returncode == 0, ausgang.stderr[-300:]
    return ausgang.stdout.strip()


@pytest.mark.parametrize("seed", ["0", "1", "2", "3"])
def test_wahl_der_welt_haengt_nicht_vom_hash_seed_ab(seed):
    assert sim_welt(seed) == sim_welt("0")


def test_beide_nimmt_die_Welt_der_ersten_Aufgabe():
    """Four tasks name production, four name arena: the tie goes to the first task, not to fate."""
    welt = {t["id"]: t.get("welt") for t in CFG["tasks"]}
    assert welt_fuer_id("beide") == welt["kinematik"]
    assert [welt[i] for i in ("kinematik", "quadrat", "korridor", "gps_anfahrt")].count(
        welt["kinematik"]) == 4


def welt_fuer_id(ids: str) -> str:
    return tasks.welt_fuer(CFG, ids)


def test_mehrheit_gewinnt_weiterhin():
    assert tasks.welt_fuer(CFG, "alle") == "production"                 # experiment 1 only
    assert tasks.welt_fuer(CFG, "kf_alle") == "arena"                   # experiment 2 only


def test_scan_docstring_und_sensor_drehen_gleich():
    """One convention, written down once: beam 0 forward, then counter-clockwise (CONTRACT §5)."""
    from mecanum_lab.types import Scan
    tekst = " ".join((Scan.__doc__ or "").split()).lower()
    assert "counter-clockwise" in tekst or "ccw" in tekst
    assert "clockwise from front-left" not in tekst


@pytest.fixture
def renderer():
    cfg = load_config()
    eng = SimEngine(load_world("production", cfg=cfg), cfg, seed=3)
    rob = eng.spawn("alice", pose=Pose(7.0, 6.0, 0.0))
    rob.gps = Gps(t=1.0, x=7.4, y=6.3, theta=0.0)
    rend = Renderer(eng, cfg)
    for _ in range(6):
        eng.step(1 / 60.0)
    try:
        yield rend, rob
    finally:
        rend.close()


def gps_pixel(rend, show_gps: bool, show_kf: bool) -> int:
    rend.show_gps, rend.show_kf = show_gps, show_kf
    rend.show_hud = rend.show_scan = rend.show_trails = rend.show_ghost = False
    rend.skid_spur = []
    rend.draw()
    punkte = pygame.surfarray.array3d(rend.screen).reshape(-1, 3)
    target = sum(1 for p in punkte if all(abs(int(p[i]) - GPS_FARBE[i]) <= 2 for i in range(3)))
    return target


def test_gps_layer_unabhaengig_vom_kf_layer(renderer):
    rend, _ = renderer
    mit_kf = gps_pixel(rend, show_gps=True, show_kf=True)
    ohne_kf = gps_pixel(rend, show_gps=True, show_kf=False)
    assert mit_kf > 0 and ohne_kf > 0, "the GPS cross must be drawn with k on *and* off"
    assert abs(mit_kf - ohne_kf) < max(4, 0.2 * mit_kf)                  # same cross, same size
    assert gps_pixel(rend, show_gps=False, show_kf=False) == 0           # `g` off really hides it
