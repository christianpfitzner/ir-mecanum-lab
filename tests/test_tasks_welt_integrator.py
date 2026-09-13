"""Regression tests for three findings of the read-the-code audit.

1. `tasks.world_for` picked the arena with `max(set(...))`. Set order follows the string hash,
   which CPython randomises per process, so `--task beide` (four tasks in production, four in
   arena) graded a different arena in another run — same seed, different hall.
2. `types.Scan` promised "clockwise from front-left" while the lidar, the contract and the
   grading helper all use beam 0 forward and counter-clockwise — the central sign convention.
3. The GPS cross was drawn inside `_estimate`, which the frame loop only calls when the
   estimate layer is on: the "gps fix" layer silently did nothing whenever `k` was off.
"""
import json

from support_logging import logged, messages
import os
import subprocess
import sys

import pygame
import pytest

from mecanum_lab import tasks
from mecanum_lab.engine import SimEngine
from mecanum_lab.render import GPS_COLOR, Renderer
from mecanum_lab.types import Gps, Pose, load_config
from mecanum_lab.worlds import load_world

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = tasks.load_tasks()


def sim_world(seed: str, query="beide") -> str:
    env = dict(os.environ, PYTHONHASHSEED=seed)
    done = subprocess.run(
        [sys.executable, "-c", "from mecanum_lab import tasks; "
         "print(tasks.world_for(tasks.load_tasks(), %r))" % query],
        capture_output=True, text=True, cwd=ROOT, env=env, timeout=60)
    assert done.returncode == 0, done.stderr[-300:]
    return done.stdout.strip()


@pytest.mark.parametrize("seed", ["0", "1", "2", "3"])
def test_world_choice_does_not_depend_on_the_hash_seed(seed):
    assert sim_world(seed) == sim_world("0")


def test_both_takes_the_world_of_the_first_task():
    """Four tasks name production, four name arena: the tie goes to the first task, not to fate."""
    world = {t["id"]: t.get("world") for t in CFG["tasks"]}
    assert world_for_id("beide") == world["kinematik"]
    assert [world[i] for i in ("kinematik", "quadrat", "korridor", "gps_anfahrt")].count(
        world["kinematik"]) == 4


def world_for_id(ids: str) -> str:
    return tasks.world_for(CFG, ids)


def test_majority_still_wins():
    assert tasks.world_for(CFG, "alle") == "production"                 # experiment 1 only
    assert tasks.world_for(CFG, "kf_alle") == "arena"                   # experiment 2 only


def test_scan_docstring_and_sensor_rotation_agree():
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
    rend.skid_trail = []
    rend.draw()
    points = pygame.surfarray.array3d(rend.screen).reshape(-1, 3)
    target = sum(1 for p in points if all(abs(int(p[i]) - GPS_COLOR[i]) <= 2 for i in range(3)))
    return target


def test_gps_layer_is_independent_of_the_kf_layer(renderer):
    rend, _ = renderer
    with_kf = gps_pixel(rend, show_gps=True, show_kf=True)
    without_kf = gps_pixel(rend, show_gps=True, show_kf=False)
    assert with_kf > 0 and without_kf > 0, "the GPS cross must be drawn with k on *and* off"
    assert abs(with_kf - without_kf) < max(4, 0.2 * with_kf)                  # same cross, same size
    assert gps_pixel(rend, show_gps=False, show_kf=False) == 0           # `g` off really hides it


def test_a_task_file_with_the_old_german_keys_still_loads(tmp_path, caplog):
    """A tasks.json from before the English migration gives the same thresholds — see _LEGACY_KEYS.

    germanids: legacy keys — this test *is* that interface, so it spells the old keys on purpose.
    """
    legacy = {"tasks": [{"id": "old_task", "titel": "Quadrat", "punkte": 30, "welt": "maze",
                         "phasen": [{"ziel": [1.0, 2.0], "dauer": 3.0,
                                     "erwarte": {"winkel_max_deg": 5.0, "abschluss_max": 9.0}}]}],
              "reihenfolge": ["old_task"]}
    file_name = tmp_path / "alt.json"
    file_name.write_text(json.dumps(legacy), encoding="utf-8")
    with logged("mecanum.tasks") as records:
        cfg = tasks.load_tasks(str(file_name))
    text = " | ".join(messages(records))
    task = cfg["tasks"][0]
    assert (task["title"], task["points"], task["world"]) == ("Quadrat", 30, "maze")
    phase = task["phases"][0]
    assert phase["target"] == [1.0, 2.0] and phase["duration"] == 3.0
    assert phase["expect"]["yaw_max_deg"] == 5.0                  # nested keys too
    assert "dauer" not in phase and "ziel" not in phase           # only today's spelling survives
    assert "deprecated key 'titel'" in text
    assert "deprecated key 'erwarte'" in text


def test_the_front_page_keeps_an_overview_of_the_halls():
    """Someone who has the repository but not the documentation still needs to know what to `--world`.

    The five halls used to be listed on the front page, then the list moved to `docs/worlds.md` and left
    nothing behind but a link — so a reader had to open a second file to learn that there is a hall that
    is deliberately empty and a hall that is deliberately narrow. The overview is back on the front page,
    without the numbers (those are measured by `tools/worldcheck.py` and checked against
    `docs/worlds.md`, and a size copied into a second page is a size that drifts): one line per hall in
    `list_worlds()`, what it is for, how to start it, and the link to the page that has the figures.
    """
    import pathlib
    from mecanum_lab.worlds import list_worlds

    readme = (pathlib.Path(__file__).resolve().parent.parent / "README.md").read_text(encoding="utf-8")
    halls = list_worlds()
    assert halls, "the simulator lists no halls at all — the scan has nothing to check"
    for hall in halls:
        assert f"`{hall}`" in readme, f"the front page lost the hall `{hall}`"
        assert f"docs/img/world_{hall}.png" in readme, \
            f"the front page lost the picture of `{hall}` — a hall named in a table and not shown is a " \
            "reader guessing what narrow means"
    assert "docs/worlds.md" in readme, "the overview no longer leads to the page with the measurements"
    assert "docs/img/worlds.png" in readme, "the overview lost the panel that shows the five at one scale"
