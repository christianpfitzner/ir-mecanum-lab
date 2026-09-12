#!/usr/bin/env python3
"""World checker: does a worlds/*.txt actually work for the lab?

Students may build their own environments (worlds/<name>.txt, characters per
worlds.py). This checker reports the four errors that cost the most nerves in
the lab: a start or goal inside a wall, gaps too narrow for the robot, no path
from start to goal, and start poses that blocking each other.

    python3 tools/worldcheck.py                 # all worlds
    python3 tools/worldcheck.py --world maze --free 0.35
Exit code 1 when a world is unusable (for tools/check.sh).
"""
import argparse
import collections
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mecanum_lab.types import Rect, cfg_get, load_config     # noqa: E402
from mecanum_lab.worlds import list_worlds, parse_grid               # noqa: E402
from mecanum_lab.worlds import cell_size as cell_size_for                     # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RICHTUNGEN = ((1, 0), (-1, 0), (0, 1), (0, -1))


def rows(path):
    with open(path, encoding="utf-8") as fh:
        text = fh.read()
    return [z for z in text.splitlines()]


def check(name, cfg, free_lift=0.0, start_page=1.0, open_max=None):
    """Check one world for usability; returns (meldungen, in_ordnung)."""
    path = os.path.join(ROOT, "worlds", f"{name}.txt")
    if not os.path.exists(path):
        return [f"{name}: file missing"], False
    cells = [list(z) for z in rows(path)]
    width = max(len(z) for z in cells)
    cells = [z + [" "] * (width - len(z)) for z in cells]
    height, cell_size = len(cells), cell_size_for(cfg, name)                  # metres per grid cell
    radius = cfg_get(cfg, "robot.footprint_r", 0.21) + free_lift

    def wall(r, k):
        return 0 <= r < height and 0 <= k < len(cells[r]) and cells[r][k] == "#"

    def center(r, k):
        return (k + 0.5) * cell_size, (height - 1 - r + 0.5) * cell_size

    world = parse_grid("\n".join("".join(z) for z in cells), cell_size, name)

    def gap(x, y):
        """Straight-line distance to the nearest wall (rects from worlds.py, point clamped in)."""
        if not world.walls:
            return math.inf
        return min(math.dist((x, y), (min(max(x, w.x0), w.x1), min(max(y, w.y0), w.y1)))
                   for w in world.walls)

    # Compute the clearance of every cell ONCE (the neighbourhood search reads it thousands of
    # times; otherwise the check goes quadratic and a maze never runs to the end).
    freiheit = {(r, k): gap(*center(r, k))
                for r in range(height) for k in range(len(cells[r])) if not wall(r, k)}

    def wall_gap(r, k):
        return freiheit.get((r, k), 0.0)

    clearance = start_page + cfg_get(cfg, "robot.footprint_r", 0.21) + 0.05

    def square_free(r, k):
        """Does the start pose have `start_page` + buffer free on all sides? -> cell or None.

        Task T2 drives its square along the starting heading, and that heading changes
        per participant (0/90/180/270 degrees, see worlds.py). So the check takes the
        envelope over all four directions: a box around the pose.
        """
        x, y = center(r, k)
        for rr in range(height):
            for kk in range(len(cells[rr])):
                if not wall(rr, kk):
                    continue
                cx0 = kk * cell_size                       # cell: x0, y_bottom
                cy0 = (height - 1 - rr) * cell_size
                if (cx0 < x + clearance and cx0 + cell_size > x - clearance
                        and cy0 < y + clearance and cy0 + cell_size > y - clearance):
                    return (rr, kk)
        return None

    starts, ziel = [], None
    for r in range(height):
        for k in range(len(cells[r])):
            if cells[r][k] in "S23456":
                starts.append((r, k))
            elif cells[r][k] == "G":
                ziel = (r, k)
    problems, ok = [], True
    if not starts:
        problems.append("no start pose (S) in the file")
        ok = False
    if ziel is None:
        problems.append("no goal (G) in the file — T3 and T4 have no target")
    for r, k in list(starts) + ([ziel] if ziel is not None else []):
        if wall(r, k):
            problems.append(f"{'Start' if (r, k) in starts else 'Goal'} in a wall (row {r}, col {k})")
            ok = False
        elif wall_gap(r, k) < radius:
            problems.append(f"{'Start' if (r, k) in starts else 'Goal'} (row {r}, col {k}) too narrow: "
                           f"{wall_gap(r, k):.2f} m free, robot needs {radius:.2f} m")
            ok = False
        elif (r, k) in starts:
            blocking = square_free(r, k)
            if blocking:
                problems.append(f"Start (row {r}, col {k}) needs {clearance:.2f} m of clearance on "
                               f"all sides: wall in row {blocking[0]}, col {blocking[1]} "
                               f"(T2 square drive, the starting heading varies per participant)")
                ok = False

    # Shortest cell path start -> goal (4-neighbours, plain cell connectivity)
    reachable, prev = False, {}
    if starts and ziel is not None:
        start = starts[0]
        queue, prev[start] = collections.deque([start]), None
        while queue:
            r, k = queue.popleft()
            if (r, k) == ziel:
                reachable = True
                break
            for dr, dk in RICHTUNGEN:
                nr, nk = r + dr, k + dk
                if wall(nr, nk) or (nr, nk) in prev:
                    continue
                prev[(nr, nk)] = (r, k)
                queue.append((nr, nk))
        if not reachable:
            problems.append("goal not reachable from the start (4-connectivity)")
            ok = False
        else:                                       # widest path: largest minimum clearance
            # What counts is not the shortest cell path (it hugs the walls) but the widest
            # one: the largest clearance b at which start and goal are connected at all
            # through cells with at least b clearance. "connected at b" only worsens as b
            # grows -> binary search over the clearance values that occur, each step a
            # plain flood fill. (A maximin Dijkstra with updates takes minutes here,
            # because a maze has a great many different widths.)
            values = sorted(set(freiheit.values()))

            def verbunden_ab(start, b):
                seen, Menge = {start}, [start]
                while Menge:
                    r, k = Menge.pop()
                    for dr, dk in RICHTUNGEN:
                        nachbar = (r + dr, k + dk)
                        if nachbar not in seen and freiheit.get(nachbar, 0.0) >= b:
                            seen.add(nachbar)
                            Menge.append(nachbar)
                return ziel in seen

            widest = 0.0
            for start in starts:                    # every participant starts elsewhere
                lo, hi = 0, len(values)              # Invariant: everything below lo is connected
                while lo < hi:
                    mitte = (lo + hi) // 2
                    if verbunden_ab(start, values[mitte]):
                        lo = mitte + 1
                    else:
                        hi = mitte
                widest = max(widest, values[lo - 1] if lo else 0.0)
            problems.append(f"widest start->goal path: narrowest point {widest:.2f} m free "
                           f"(needs {radius:.2f} m)" + ("" if widest >= radius else "  << too narrow"))
            if widest < radius:
                ok = False

    # Maze or arena? Free cells with a completely free 3x3 neighbourhood count as "open" —
    # a real maze has almost none of them, an arena with tables has many.
    free_cells = [(r, k) for r in range(height) for k in range(len(cells[r])) if not wall(r, k)]
    at_border = [(r, k) for r, k in free_cells if r in (0, height - 1) or k in (0, len(cells[r]) - 1)]
    problems.append(f"free cells on the outer edge (the world has to be closed): {len(at_border)}"
                   + ("" if not at_border else f"  << open at row/col {at_border[:3]}"))
    if at_border:
        ok = False
    if free_cells and open_max is not None:
        open_cells = [cell for cell in free_cells
                 if all(not wall(cell[0] + dr, cell[1] + dk)
                        for dr in (-1, 0, 1) for dk in (-1, 0, 1))]
        share = len(open_cells) / len(free_cells)
        problems.append(f"share of open cells (3x3 neighbourhood free): {share:.2f} "
                       f"({len(open_cells)}/{len(free_cells)}) — limit {open_max:.2f}"
                       + ("" if share <= open_max else "  << too open, that is an arena"))
        if share > open_max:
            ok = False
    size = (width * cell_size, height * cell_size)
    problems.insert(0, f"{name}: {size[0]:.1f} x {size[1]:.1f} m, {len(world.walls)} rectangles, "
                      f"{len(starts)} starts, goal {'yes' if ziel else 'no'}")
    return problems, ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Check worlds for usability")
    ap.add_argument("--world", default=None, help="check a single world only")
    ap.add_argument("--free", type=float, default=0.25,
                    help="allowance on top of the robot radius, for slack (m)")
    ap.add_argument("--start_page", type=float, default=None,
                    help="side length of the square that must fit at every start pose (m); "
                         "task T2 drives exactly that square")
    ap.add_argument("--open-max", "--offen-max", dest="open_max", type=float, default=None,
                    help="maximum share of free cells with a fully free 3x3 neighbourhood; "
                         "0.25 for a maze, larger for an arena")
    ap.add_argument("--cell", type=float, default=None)
    args = ap.parse_args(argv)
    if any(a.startswith("--offen-max") for a in (argv or sys.argv[1:])):
        print("deprecated option '--offen-max', use '--open-max'")
    cfg = load_config()
    if args.cell:
        cfg.setdefault("worlds", {})["cell"] = args.cell
    # How much free space around a start pose does a world need? config/tasks.json says it:
    # only tasks with a side length ("side") — currently T2 — drive a square there.
    try:
        from mecanum_lab import tasks as T
        square_side = {}
        for auftrag in T.load_tasks().get("tasks", []):
            if auftrag.get("side"):
                world = auftrag.get("world", "production")
                square_side[world] = max(square_side.get(world, 0.0), float(auftrag["side"]))
    except Exception as exc:
        print(f"  note: config/tasks.json not readable ({exc}), square check disabled")
        square_side = {}
    bad_worlds = 0
    for name in ([args.world] if args.world else list_worlds()):
        page = args.start_page or square_side.get(name, 0.0)      # square drives only

        problems, ok = check(name, cfg, args.free, page)
        print(("  ok   " if ok else "  FAIL ") + problems[0])
        for line in problems[1:]:
            print(f"         {line}")
        bad_worlds += 0 if ok else 1
    print(f"\n{bad_worlds} worlds with problems")
    return 1 if bad_worlds else 0


if __name__ == "__main__":
    sys.exit(main())
