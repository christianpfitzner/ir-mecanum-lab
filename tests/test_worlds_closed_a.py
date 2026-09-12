"""Tests for the worlds: closed borders, per-world cell size, and the maze that fits a robot.

All of this is checked with `tools/worldcheck.py` too — but that tool is a CLI for the
supervisor, and these are the invariants a student's new world file must not break.
"""
import math
import os
import subprocess
import sys

import pytest

from mecanum_lab.types import load_config
from mecanum_lab.worlds import list_worlds, load_world, parse_grid, zell

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = load_config()


def abstand(welt, x, y):
    """Distance from a point to the nearest wall rectangle."""
    d = 9e9
    for wand in welt.walls:
        d = min(d, math.hypot(max(wand.x0 - x, 0, x - wand.x1),
                              max(wand.y0 - y, 0, y - wand.y1)))
    return d


def freie_zellen(welt):
    z = welt.cell
    return {(i, j) for j in range(int(welt.size[1] / z)) for i in range(int(welt.size[0] / z))
            if abstand(welt, (i + .5) * z, (j + .5) * z) > 1e-9}


def erreichbar(von, zellen):
    seen, offen = {von}, [von]
    while offen:
        i, j = offen.pop()
        for nachbar in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if nachbar in zellen and nachbar not in seen:
                seen.add(nachbar)
                offen.append(nachbar)
    return seen


# ------------------------------------------------------------------------ closed everywhere


@pytest.mark.parametrize("name", list_worlds())
def test_every_world_is_closed(name):
    """No free cell may touch the outer edge — otherwise the robot drives out of the map."""
    welt = load_world(name, cfg=CFG)
    weit, hoch = int(welt.size[0] / welt.cell) - 1, int(welt.size[1] / welt.cell) - 1
    am_rand = [(i, j) for i, j in freie_zellen(welt) if i in (0, weit) or j in (0, hoch)]
    assert not am_rand, f"{name} is open at these cells: {am_rand[:4]}"


@pytest.mark.parametrize("name", list_worlds())
def test_every_spawn_and_the_goal_reach_each_other(name):
    welt = load_world(name, cfg=CFG)
    zellen = freie_zellen(welt)
    assert zellen, f"{name} has no free cell"
    start = min(zellen, key=lambda ij: (ij[0] ** 2 + ij[1] ** 2))
    erreichbar_von = erreichbar(start, zellen)
    for pose in welt.spawns + ([welt.goal] if welt.goal else []):
        i, j = int(pose.x / welt.cell), int(pose.y / welt.cell)
        assert (i, j) in erreichbar_von, f"{name}: {pose} is cut off"
        assert abstand(welt, pose.x, pose.y) > welt.cell * 0.28, f"{name}: {pose} is too tight"


def test_the_maze_is_wide_enough_for_the_robot():
    """Lab 1 starts in the maze; a 0.42 m robot must fit without scraping every corner."""
    welt = load_world("maze", cfg=CFG)
    frei, fuss = freie_zellen(welt), CFG["robot"]["footprint_r"]
    engste = min(abstand(welt, (i + .5) * welt.cell, (j + .5) * welt.cell) for i, j in frei)
    assert welt.cell == pytest.approx(1.0)
    assert engste > fuss, f"narrowest place is {engste:.2f} m, the robot needs {fuss:.2f} m"


def test_the_maze_is_no_arena():
    welt = load_world("maze", cfg=CFG)
    frei = freie_zellen(welt)
    anteile = [all((i + di, j + dj) in frei for di in (-1, 0, 1) for dj in (-1, 0, 1))
               for i, j in frei]
    assert sum(anteile) / len(anteile) < 0.6, "the maze may be coarser, but it must stay a maze"


# ------------------------------------------------------------------- cell size per world


def test_cell_size_can_be_set_per_world():
    cfg = {"worlds": {"cell": 0.5, "cell_by_world": {"labyrinth": 1.25}}}
    assert zell(cfg, "labyrinth") == 1.25
    assert zell(cfg, "arena") == 0.5
    assert zell({}, "arena") == 0.5


def test_the_same_grid_is_a_different_world_for_another_cell_size():
    text = "#####\n#S.G#\n#####\n"
    klein, gross = parse_grid(text, 0.5, "x"), parse_grid(text, 1.0, "x")
    assert klein.size == (2.5, 1.5) and gross.size == (5.0, 3.0)
    assert gross.spawns[0].x == 1.5 and klein.spawns[0].x == 0.75


def test_load_world_caches_per_cell_size():
    welt_halb = load_world("maze", cfg={"worlds": {"cell": 0.5}})
    welt_ganz = load_world("maze", cfg={"worlds": {"cell_by_world": {"maze": 1.0}}})
    assert welt_halb.cell == 0.5 and welt_ganz.cell == 1.0
    assert welt_ganz.size[0] > welt_halb.size[0]
    assert load_world("maze", cfg={"worlds": {"cell": 0.5}}) is welt_halb


# ------------------------------------------------------------------------ the supervisor tool


def test_worldcheck_finds_nothing_to_complain_about():
    """The worlds must pass the tool the tutor runs before the lab session."""
    lauf = subprocess.run([sys.executable, os.path.join(REPO, "tools", "worldcheck.py")],
                          capture_output=True, text=True, cwd=REPO)
    assert "FAIL" not in lauf.stdout, lauf.stdout
    assert lauf.stdout.count("ok  ") >= 4, lauf.stdout        # every shipped world listed
    assert lauf.returncode == 0, f"worldcheck exited {lauf.returncode}: {lauf.stdout}"
