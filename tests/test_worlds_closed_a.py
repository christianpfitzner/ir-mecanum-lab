"""Tests for the worlds: closed borders, per-world cell size, and the maze that fits a robot.

All of this is checked with `tools/worldcheck.py` too — but that tool is a CLI for the
supervisor, and these are the invariants a student's new world file must not break.
"""
import heapq
import math
import os
import re
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


def reachable(start, cells):
    seen, frontier = {start}, [start]
    while frontier:
        i, j = frontier.pop()
        for neighbor in ((i + 1, j), (i - 1, j), (i, j + 1), (i, j - 1)):
            if neighbor in cells and neighbor not in seen:
                seen.add(neighbor)
                frontier.append(neighbor)
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
    cells = free_cells(world)
    assert cells, f"{name} has no free cell"
    start = min(cells, key=lambda ij: (ij[0] ** 2 + ij[1] ** 2))
    erreichbar_von = reachable(start, cells)
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


# ------------------------------------------------------------------------------- rooms
#   The one world whose walls divide the hall instead of standing in it. `production` has racks — you
#   drive around them and the hall stays one hall — so every measure of how open a world is rates it
#   like an indoor plan. What makes `rooms` indoor is measured here in the two things that do differ.


def _open_share(world) -> float:
    """Share of free cells whose whole 3x3 neighbourhood is free — the maze test's own yardstick."""
    free = free_cells(world)
    return sum(all((i + di, j + dj) in free for di in (-1, 0, 1) for dj in (-1, 0, 1))
               for i, j in free) / len(free)


def _doors(line: str, wall: str = "#") -> list:
    """Widths of the gaps in a line that is mostly wall — the doors of that wall.

    Only lines that are at least 60 % wall count: a row of racks is not a wall that divides the hall,
    and `production`, whose obstacles are all blocks, has no such line at all. A gap wider than six
    cells is not a door either, it is the hall opening up.
    """
    if line.count(wall) < 0.6 * len(line):
        return []
    return [len(run) for run in re.findall(rf"[^{wall}]+", line) if 0 < len(run) <= 6]


def test_rooms_is_no_arena():
    """0.58 by the maze's yardstick, where `arena` reaches 0.89 and `open` 0.91."""
    assert _open_share(load_world("rooms", cfg=CFG)) < 0.75, \
        "rooms has been emptied — an indoor plan with nothing in it is the arena again"


def test_the_widest_way_through_rooms_is_a_corridor():
    """The widest route from a spawn to the goal is 0.75 m — the spine's own width, not an accident.

    Measured as the widest path (the route whose narrowest point is as wide as any route's can be),
    because that is what a driver faces: 0.75 m in `rooms` and in `track`, against 1.25 m in
    `production` and 4.75 m in `arena`. The tight cells of that route are the cells of the 1.5 m corridor
    — measured, they are the painted axis row of the plan — which is worth saying because it is the
    *corridor* that makes this world indoor, not the doors: widening the doorway into the goal room to
    4 m leaves the number at 0.75 m, while opening the corridor to 2.5 m turns the plan into a hall
    (1.25 m) and this assertion says so. The lower bound is what the strict world check asks of the
    robot (0.46 m): constrained route, still drivable.
    """
    world = load_world("rooms", cfg=CFG)
    free = free_cells(world)
    capacity = {p: gap(world, (p[0] + .5) * world.cell, (p[1] + .5) * world.cell) for p in free}
    goal = (int(world.goal.x / world.cell), int(world.goal.y / world.cell))
    widest = 0.0
    for pose in world.spawns:
        start = (int(pose.x / world.cell), int(pose.y / world.cell))
        best, queue = {start: capacity[start]}, [(-capacity[start], start)]
        while queue:
            narrow, p = heapq.heappop(queue); narrow = -narrow
            if narrow < best.get(p, 0) - 1e-9:
                continue
            for neighbour in ((p[0] + 1, p[1]), (p[0] - 1, p[1]), (p[0], p[1] + 1), (p[0], p[1] - 1)):
                if neighbour in free:
                    narrow_here = min(narrow, capacity[neighbour])
                    if narrow_here > best.get(neighbour, -1):
                        best[neighbour] = narrow_here
                        heapq.heappush(queue, (-narrow_here, neighbour))
        widest = max(widest, best.get(goal, 0.0))
    needed = CFG["robot"]["footprint_r"] + 0.25
    assert widest >= needed, f"the widest route is {widest:.2f} m, the robot needs {needed:.2f} m"
    assert widest <= 1.0, f"a route {widest:.2f} m wide everywhere is `arena` with walls drawn in"


def test_the_hard_door_in_rooms_is_still_the_only_one():
    """Ten doorways divide the hall, nine of them 1.5 m and one 1.0 m — and that one is the exercise.

    The wide doors are what makes the graded route drivable: the strict check wants 0.46 m at the
    narrowest point of a route, and a 1 m doorway leaves only 0.25 m at its centre cell. The single
    narrow doorway joins two rooms directly, off every start->goal route — the place where a sideways
    odometry error costs a student a minute instead of a collision. Both halves are asserted because
    both are easy to break by "fixing" one of them: widen the hard door and the world has lost its
    point, narrow the rest and it has become unfair.
    """
    lines = open(os.path.join(REPO, "worlds", "rooms.txt"), encoding="utf-8").read().splitlines()
    openings = [w for line in lines for w in _doors(line)] \
        + [w for column in zip(*lines) for w in _doors("".join(column))]
    assert len(openings) >= 6, f"only {len(openings)} doorways in walls that divide the hall"
    assert sorted(openings)[:2] == [2, 3], f"door widths {sorted(openings)} — expected one 1 m door, rest 1.5 m"
    assert 1 not in openings, "a 0.5 m doorway is not a door for a 0.42 m robot, it is a trap"


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
