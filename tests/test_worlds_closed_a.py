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
from mecanum_lab.worlds import list_worlds, load_world, parse_grid, cell_size

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CFG = load_config()


def gap(world, x, y):
    """Distance from a point to the nearest wall rectangle."""
    d = 9e9
    for wand in world.walls:
        d = min(d, math.hypot(max(wand.x0 - x, 0, x - wand.x1),
                              max(wand.y0 - y, 0, y - wand.y1)))
    return d


def free_cells(world):
    z = world.cell
    return {(i, j) for j in range(int(world.size[1] / z)) for i in range(int(world.size[0] / z))
            if gap(world, (i + .5) * z, (j + .5) * z) > 1e-9}


def reachable(von, zellen):
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
    world = load_world(name, cfg=CFG)
    weit, hoch = int(world.size[0] / world.cell) - 1, int(world.size[1] / world.cell) - 1
    at_border = [(i, j) for i, j in free_cells(world) if i in (0, weit) or j in (0, hoch)]
    assert not at_border, f"{name} is open at these cells: {at_border[:4]}"


@pytest.mark.parametrize("name", list_worlds())
def test_every_spawn_and_the_goal_reach_each_other(name):
    world = load_world(name, cfg=CFG)
    zellen = free_cells(world)
    assert zellen, f"{name} has no free cell"
    start = min(zellen, key=lambda ij: (ij[0] ** 2 + ij[1] ** 2))
    erreichbar_von = reachable(start, zellen)
    for pose in world.spawns + ([world.goal] if world.goal else []):
        i, j = int(pose.x / world.cell), int(pose.y / world.cell)
        assert (i, j) in erreichbar_von, f"{name}: {pose} is cut off"
        assert gap(world, pose.x, pose.y) > world.cell * 0.28, f"{name}: {pose} is too tight"


def test_the_maze_is_wide_enough_for_the_robot():
    """Lab 1 starts in the maze; a 0.42 m robot must fit without scraping every corner."""
    world = load_world("maze", cfg=CFG)
    frei, fuss = free_cells(world), CFG["robot"]["footprint_r"]
    engste = min(gap(world, (i + .5) * world.cell, (j + .5) * world.cell) for i, j in frei)
    assert world.cell == pytest.approx(1.0)
    assert engste > fuss, f"narrowest place is {engste:.2f} m, the robot needs {fuss:.2f} m"


def test_the_maze_is_no_arena():
    world = load_world("maze", cfg=CFG)
    frei = free_cells(world)
    shares = [all((i + di, j + dj) in frei for di in (-1, 0, 1) for dj in (-1, 0, 1))
               for i, j in frei]
    assert sum(shares) / len(shares) < 0.6, "the maze may be coarser, but it must stay a maze"


# ------------------------------------------------------------------- cell size per world


def test_cell_size_can_be_set_per_world():
    cfg = {"worlds": {"cell": 0.5, "cell_by_world": {"labyrinth": 1.25}}}
    assert cell_size(cfg, "labyrinth") == 1.25
    assert cell_size(cfg, "arena") == 0.5
    assert cell_size({}, "arena") == 0.5


def test_the_same_grid_is_a_different_world_for_another_cell_size():
    text = "#####\n#S.G#\n#####\n"
    small, big = parse_grid(text, 0.5, "x"), parse_grid(text, 1.0, "x")
    assert small.size == (2.5, 1.5) and big.size == (5.0, 3.0)
    assert big.spawns[0].x == 1.5 and small.spawns[0].x == 0.75


def test_load_world_caches_per_cell_size():
    world_half = load_world("maze", cfg={"worlds": {"cell": 0.5}})
    world_full = load_world("maze", cfg={"worlds": {"cell_by_world": {"maze": 1.0}}})
    assert world_half.cell == 0.5 and world_full.cell == 1.0
    assert world_full.size[0] > world_half.size[0]
    assert load_world("maze", cfg={"worlds": {"cell": 0.5}}) is world_half


# ------------------------------------------------------------------------ the supervisor tool


def test_worldcheck_finds_nothing_to_complain_about():
    """The worlds must pass the tool the tutor runs before the lab session."""
    run = subprocess.run([sys.executable, os.path.join(REPO, "tools", "worldcheck.py")],
                          capture_output=True, text=True, cwd=REPO)
    assert "FAIL" not in run.stdout, run.stdout
    assert run.stdout.count("ok  ") >= len(list_worlds()), run.stdout   # every shipped world listed
    assert run.returncode == 0, f"worldcheck exited {run.returncode}: {run.stdout}"
