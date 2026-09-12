"""Umgebungen aus ASCII-Grids — eine Karte ist ein Textfile, kein Binärformat.

Zeichen (CONTRACT §6.2):  `#` Wand · `.` oder Leerschlag frei · `S` erster Start ·
`2`..`9` weitere Starts · `G` Ziel · `-` und `|` reine Bodenmarkierung (nur Ansicht,
keine Kollision). Der Parser kennt keine Kommentare: eine Zeile, die mit `#` anfängt,
ist eine Wandzeile.

Zeile 0 des Textes ist die Oberkante. Der Weltursprung liegt unten links, jede Pose
in der Mitte ihrer Zelle, die Blickrichtung folgt der Startnummer (0, 90, 180, 270 Grad).
"""
import logging
import math
import os

from .types import ROOT, Pose, Rect, World

CELL = 0.5                                       # Kantenlänge einer Gridzelle in Metern
THETA = (0.0, math.pi / 2, math.pi, -math.pi / 2)
_cache: dict = {}
log = logging.getLogger("mecanum.worlds")


def parse_grid(text: str, cell: float = CELL, name: str = "?") -> World:
    """Gridtext -> World mit verschmolzenen Wänden, Startposes, Ziel und Markierungen."""
    rows = text.split("\n")
    while rows and not rows[-1].strip():
        rows.pop()
    if not rows:
        raise ValueError(f"Welt '{name}' ist leer.")
    rows = [r.ljust(max(len(x) for x in rows)) for r in rows]
    top, wide = len(rows), max(len(r) for r in rows)
    center = lambda r, c: ((c + 0.5) * cell, (top - 1 - r + 0.5) * cell)
    wall_cells, marks, spawns, goal = set(), {"-": set(), "|": set()}, {}, None
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
                marks[ch].add((r, c))
            else:
                raise ValueError(f"Welt '{name}': unbekanntes Zeichen '{ch}' in "
                                 f"Zeile {r + 1}, Spalte {c + 1} (nur #. SG2-9 -| erlaubt)")
    world = World(name=name, cell=cell, walls=_rects(wall_cells, top, wide, cell),
                  spawns=[spawns[k] for k in sorted(spawns)], goal=goal,
                  size=(wide * cell, top * cell))
    for glyph, (dr, dc) in (("-", (0, 1)), ("|", (1, 0))):   # Zuge statt Einzelstriche
        cells = marks[glyph]
        for r, c in sorted(cells):
            if (r - dr, c - dc) in cells:          # nur der Anfang eines Zugs zaehlt
                continue
            n = 1
            while (r + n * dr, c + n * dc) in cells:
                n += 1
            world.markings.append((*center(r, c), *center(r + (n - 1) * dr, c + (n - 1) * dc)))
    if not world.spawns:
        log.warning("Welt '%s' hat keine Startpose (S fehlt).", name)
    return world


def _rects(cells: set, nrows: int, ncols: int, cell: float) -> list:
    """Wandzellen in zwei Daemen zu möglichst wenigen großen Rechtecken verschmelzen."""
    spans = {}                                    # (c0, c1) -> [Zeilen aufsteigend]
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
            # Welt-Y: Zeile r liegt oben, also y0 unter der untersten Zeile der Folge
            rects.append(Rect(c0 * cell, (nrows - 1 - prev) * cell,
                              (c1 + 1) * cell, (nrows - start) * cell))
            if r is not None:
                start = prev = r
    return rects


def load_world(name: str, path: str | None = None) -> World:
    """worlds/<name>.txt einlesen (Pfad relativ zum Projekt, nicht zur cwd) und cachen."""
    if name not in _cache or path:
        p = path or os.path.join(ROOT, "worlds", f"{name}.txt")
        with open(p, encoding="utf-8") as fh:
            _cache[name] = parse_grid(fh.read(), CELL, name)
    return _cache[name]


def list_worlds() -> list:
    """Namen aller verfügbaren Umgebungen, sortiert."""
    folder = os.path.join(ROOT, "worlds")
    return sorted(f[:-4] for f in os.listdir(folder) if f.endswith(".txt"))
