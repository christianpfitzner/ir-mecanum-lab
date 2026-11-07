"""Environments from ASCII grids — a map is a text file, not a binary format.

Characters (CONTRACT §6.2):  `#` wall · `.` or space free · `S` first spawn ·
`2`..`9` further spawns · `G` goal · `-` and `|` painted floor: free to drive over,
and nothing is drawn for it. The parser has no comments: a line that starts with `#`
is a wall line.

The paint stays in the files because `production` is a hall with lanes, and the grid is the readable
form of that hall — a person looking at the file should see the lanes the robot is supposed to follow.
The window stopped drawing them: a painted line is neither an obstacle nor a measurement, and every
layer that costs a student the view of something real has to answer for it.

Line 0 of the text is the top edge. The world origin sits at the bottom left, every pose
in the center of its cell, the heading follows the spawn number (0, 90, 180, 270 degrees).
"""
import logging
import math
import os
from dataclasses import replace

from .types import ROOT, Circle, Pose, Rect, World, cfg_get

CELL = 0.5                                       # edge length of a grid cell in meters
THETA = (0.0, math.pi / 2, math.pi, -math.pi / 2)
_cache: dict = {}
log = logging.getLogger("mecanum.worlds")


def parse_grid(text: str, cell: float = CELL, name: str = "?") -> World:
    """Grid text -> World with merged walls, spawn poses and goal."""
    rows = text.split("\n")
    while rows and not rows[-1].strip():
        rows.pop()
    if not rows:
        raise ValueError(f"World '{name}' is empty.")
    rows = [r.ljust(max(len(x) for x in rows)) for r in rows]
    top, wide = len(rows), max(len(r) for r in rows)
    center = lambda r, c: ((c + 0.5) * cell, (top - 1 - r + 0.5) * cell)
    wall_cells, spawns, goal = set(), {}, None
    for r, row in enumerate(rows):
        for c, ch in enumerate(row):
            if ch == "#":
                wall_cells.add((r, c))
            elif ch in ". ":
                pass
            elif ch == "G":
                goal = Pose(*center(r, c), 0.0) if goal is None else goal
            elif ch == "S":
                spawns.setdefault(0, Pose(*center(r, c), THETA[0]))
            elif ch.isdigit():
                n = int(ch) - 1
                spawns.setdefault(n, Pose(*center(r, c), THETA[n % 4]))
            elif ch in "-|":
                pass                                    # paint: free floor, nothing drawn
            else:
                raise ValueError(f"World '{name}': unknown character '{ch}' in "
                                 f"line {r + 1}, column {c + 1} (only #. SG2-9 -| allowed)")
    world = World(name=name, cell=cell, walls=_rects(wall_cells, top, wide, cell),
                  spawns=[spawns[k] for k in sorted(spawns)], goal=goal,
                  size=(wide * cell, top * cell))
    if not world.spawns:
        log.warning("World '%s' has no spawn pose (S missing).", name)
    return world


def _rects(cells: set, nrows: int, ncols: int, cell: float) -> list:
    """Merge wall cells in two passes into as few large rectangles as possible."""
    spans = {}                                    # (c0, c1) -> [rows ascending]
    for r in range(nrows):
        c = 0
        while c < ncols:
            if (r, c) not in cells:
                c += 1
                continue
            c0 = c
            while c < ncols and (r, c) in cells:
                c += 1
            spans.setdefault((c0, c - 1), []).append(r)
    rects = []
    for (c0, c1), rws in spans.items():
        start = prev = rws[0]
        for r in rws[1:] + [None]:
            if r is not None and r == prev + 1:
                prev = r
                continue
            # World Y: row r sits at the top, so y0 is below the lowest row of the run
            rects.append(Rect(c0 * cell, (nrows - 1 - prev) * cell,
                              (c1 + 1) * cell, (nrows - start) * cell))
            if r is not None:
                start = prev = r
    return rects


def cell_size(cfg: dict | None, name: str, default: float = CELL) -> float:
    """Edge length of one grid cell for this world.

    `worlds.cell_by_world.<name>` wins over `worlds.cell` over the built-in CELL. That is how
    the maze can be built on a 0.6 m grid while the graded halls stay on 0.5 m — the robot is
    the same size everywhere, the world decides how much room it has.
    """
    cfg = cfg or {}
    per_world = cfg_get(cfg, "worlds.cell_by_world") or {}
    value = per_world.get(name, cfg_get(cfg, "worlds.cell", default))
    return float(value)


def load_world(name: str, path: str | None = None, cfg: dict | None = None) -> World:
    """Read worlds/<name>.txt (path relative to the project, not to the cwd) and cache it.

    Cached per cell size: the same file can be read as a 0.5 m and as a 0.6 m world.
    A `name` that ends in `.txt` is a file of its own, e.g. the hall a lab repository brings along
    (`--world /path/to/box.txt`); the world is then called after the file, `box`.
    """
    if not path and str(name).endswith(".txt"):
        path, name = name, os.path.basename(name)[:-4]
    zellen = cell_size(cfg, name)
    key = (name, zellen)
    if key not in _cache or path:
        p = path or os.path.join(ROOT, "worlds", f"{name}.txt")
        with open(p, encoding="utf-8") as fh:
            _cache[key] = parse_grid(fh.read(), zellen, name)
    return _cache[key]


def apply_config(world: World, cfg: dict | None) -> World:
    """The world with what the config adds to the grid: round obstacles and start poses with any heading.

    `obstacles.circles` is `[[x, y, r], ...]` and `spawns` is `[[x, y, theta_deg], ...]` (types.DEFAULT_CONFIG).
    Both are empty by default, and then the very same world object comes back — the cached one, so no
    hall of a graded task changes. Otherwise a copy, because the cache is shared by every engine.
    A circle outside the hall or a pose inside a wall or circle raises: nobody can solve that scenario.
    """
    circles = [Circle(*map(float, c)) for c in (cfg_get(cfg or {}, "obstacles.circles") or [])]
    poses = [Pose(float(x), float(y), math.radians(float(deg)))
             for x, y, deg in (cfg_get(cfg or {}, "spawns") or [])]
    if not circles and not poses:
        return world
    out = replace(world, circles=circles, spawns=poses or world.spawns)
    for c in circles:
        if c.r <= 0.0:
            raise ValueError(f"circle at ({c.x:g}, {c.y:g}) in world '{world.name}': radius must be > 0")
        world.free("circle", c.x, c.y)              # against the walls; `out` would find the circle itself
    for pose in poses:
        out.free("spawn pose", pose.x, pose.y, " (config key `spawns`)")
    return out


def list_worlds() -> list:
    """Names of all available environments, sorted."""
    folder = os.path.join(ROOT, "worlds")
    return sorted(f[:-4] for f in os.listdir(folder) if f.endswith(".txt"))
