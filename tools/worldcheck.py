#!/usr/bin/env python3
"""World checker: does a worlds/*.txt actually work for the lab?

Students may build their own environments (worlds/<name>.txt, characters per
worlds.py). This checker reports the four errors that cost the most nerves in
the lab: a start or goal inside a wall, gaps too narrow for the robot, no path
from start to goal, and start poses that block each other.

    python3 tools/worldcheck.py                 # all worlds
    python3 tools/worldcheck.py --welt maze --frei 0.35
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
from mecanum_lab.worlds import zell as zell_fuer                     # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RICHTUNGEN = ((1, 0), (-1, 0), (0, 1), (0, -1))


def zeilen(pfad):
    with open(pfad, encoding="utf-8") as fh:
        text = fh.read()
    return [z for z in text.splitlines()]


def pruefe(name, cfg, frei_hebe=0.0, startseite=1.0, offen_max=None):
    """Check one world for usability; returns (meldungen, in_ordnung)."""
    pfad = os.path.join(ROOT, "worlds", f"{name}.txt")
    if not os.path.exists(pfad):
        return [f"{name}: file missing"], False
    zellen = [list(z) for z in zeilen(pfad)]
    breite = max(len(z) for z in zellen)
    zellen = [z + [" "] * (breite - len(z)) for z in zellen]
    hoehe, zell = len(zellen), zell_fuer(cfg, name)                  # metres per grid cell
    radius = cfg_get(cfg, "robot.footprint_r", 0.21) + frei_hebe

    def wand(r, k):
        return 0 <= r < hoehe and 0 <= k < len(zellen[r]) and zellen[r][k] == "#"

    def mittelpunkt(r, k):
        return (k + 0.5) * zell, (hoehe - 1 - r + 0.5) * zell

    welt = parse_grid("\n".join("".join(z) for z in zellen), zell, name)

    def abstand(x, y):
        """Straight-line distance to the nearest wall (rects from worlds.py, point clamped in)."""
        if not welt.walls:
            return math.inf
        return min(math.dist((x, y), (min(max(x, w.x0), w.x1), min(max(y, w.y0), w.y1)))
                   for w in welt.walls)

    # Compute the clearance of every cell ONCE (the neighbourhood search reads it thousands of
    # times; otherwise the check goes quadratic and a maze never runs to the end).
    freiheit = {(r, k): abstand(*mittelpunkt(r, k))
                for r in range(hoehe) for k in range(len(zellen[r])) if not wand(r, k)}

    def wandabstand(r, k):
        return freiheit.get((r, k), 0.0)

    reich = startseite + cfg_get(cfg, "robot.footprint_r", 0.21) + 0.05

    def quadratfrei(r, k):
        """Does the start pose have `startseite` + buffer free on all sides? -> cell or None.

        Task T2 drives its square along the starting heading, and that heading changes
        per participant (0/90/180/270 degrees, see worlds.py). So the check takes the
        envelope over all four directions: a box around the pose.
        """
        x, y = mittelpunkt(r, k)
        for rr in range(hoehe):
            for kk in range(len(zellen[rr])):
                if not wand(rr, kk):
                    continue
                ku, ko = kk * zell, (hoehe - 1 - rr) * zell      # cell: x0, y_bottom
                if ku < x + reich and ku + zell > x - reich and ko < y + reich and ko + zell > y - reich:
                    return (rr, kk)
        return None

    starts, ziel = [], None
    for r in range(hoehe):
        for k in range(len(zellen[r])):
            if zellen[r][k] in "S23456":
                starts.append((r, k))
            elif zellen[r][k] == "G":
                ziel = (r, k)
    meldung, ok = [], True
    if not starts:
        meldung.append("no start pose (S) in the file")
        ok = False
    if ziel is None:
        meldung.append("no goal (G) in the file — T3 and T4 have no target")
    for r, k in list(starts) + ([ziel] if ziel is not None else []):
        if wand(r, k):
            meldung.append(f"{'Start' if (r, k) in starts else 'Goal'} in a wall (row {r}, col {k})")
            ok = False
        elif wandabstand(r, k) < radius:
            meldung.append(f"{'Start' if (r, k) in starts else 'Goal'} (row {r}, col {k}) too narrow: "
                           f"{wandabstand(r, k):.2f} m free, robot needs {radius:.2f} m")
            ok = False
        elif (r, k) in starts:
            block = quadratfrei(r, k)
            if block:
                meldung.append(f"Start (row {r}, col {k}) needs {reich:.2f} m of clearance on "
                               f"all sides: wall in row {block[0]}, col {block[1]} "
                               f"(T2 square drive, the starting heading varies per participant)")
                ok = False

    # Shortest cell path start -> goal (4-neighbours, plain cell connectivity)
    erreichbar, vorgaenger = False, {}
    if starts and ziel is not None:
        start = starts[0]
        offen, vorgaenger[start] = collections.deque([start]), None
        while offen:
            r, k = offen.popleft()
            if (r, k) == ziel:
                erreichbar = True
                break
            for dr, dk in RICHTUNGEN:
                nr, nk = r + dr, k + dk
                if wand(nr, nk) or (nr, nk) in vorgaenger:
                    continue
                vorgaenger[(nr, nk)] = (r, k)
                offen.append((nr, nk))
        if not erreichbar:
            meldung.append("goal not reachable from the start (4-connectivity)")
            ok = False
        else:                                       # widest path: largest minimum clearance
            # What counts is not the shortest cell path (it hugs the walls) but the widest
            # one: the largest clearance b at which start and goal are connected at all
            # through cells with at least b clearance. "connected at b" only worsens as b
            # grows -> binary search over the clearance values that occur, each step a
            # plain flood fill. (A maximin Dijkstra with updates takes minutes here,
            # because a maze has a great many different widths.)
            werte = sorted(set(freiheit.values()))

            def verbunden_ab(start, b):
                gesehen, Menge = {start}, [start]
                while Menge:
                    r, k = Menge.pop()
                    for dr, dk in RICHTUNGEN:
                        nachbar = (r + dr, k + dk)
                        if nachbar not in gesehen and freiheit.get(nachbar, 0.0) >= b:
                            gesehen.add(nachbar)
                            Menge.append(nachbar)
                return ziel in gesehen

            eng = 0.0
            for start in starts:                    # every participant starts elsewhere
                lo, hi = 0, len(werte)              # Invariant: everything below lo is connected
                while lo < hi:
                    mitte = (lo + hi) // 2
                    if verbunden_ab(start, werte[mitte]):
                        lo = mitte + 1
                    else:
                        hi = mitte
                eng = max(eng, werte[lo - 1] if lo else 0.0)
            meldung.append(f"widest start->goal path: narrowest point {eng:.2f} m free "
                           f"(needs {radius:.2f} m)" + ("" if eng >= radius else "  << too narrow"))
            if eng < radius:
                ok = False

    # Maze or arena? Free cells with a completely free 3x3 neighbourhood count as "open" —
    # a real maze has almost none of them, an arena with tables has many.
    freie = [(r, k) for r in range(hoehe) for k in range(len(zellen[r])) if not wand(r, k)]
    am_rand = [(r, k) for r, k in freie if r in (0, hoehe - 1) or k in (0, len(zellen[r]) - 1)]
    meldung.append(f"free cells on the outer edge (the world has to be closed): {len(am_rand)}"
                   + ("" if not am_rand else f"  << open at row/col {am_rand[:3]}"))
    if am_rand:
        ok = False
    if freie and offen_max is not None:
        offen = [cell for cell in freie
                 if all(not wand(cell[0] + dr, cell[1] + dk)
                        for dr in (-1, 0, 1) for dk in (-1, 0, 1))]
        anteil = len(offen) / len(freie)
        meldung.append(f"share of open cells (3x3 neighbourhood free): {anteil:.2f} "
                       f"({len(offen)}/{len(freie)}) — limit {offen_max:.2f}"
                       + ("" if anteil <= offen_max else "  << too open, that is an arena"))
        if anteil > offen_max:
            ok = False
    gruesse = (breite * zell, hoehe * zell)
    meldung.insert(0, f"{name}: {gruesse[0]:.1f} x {gruesse[1]:.1f} m, {len(welt.walls)} rectangles, "
                      f"{len(starts)} starts, goal {'yes' if ziel else 'no'}")
    return meldung, ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Check worlds for usability")
    ap.add_argument("--welt", default=None, help="check a single world only")
    ap.add_argument("--frei", type=float, default=0.25,
                    help="allowance on top of the robot radius, for slack (m)")
    ap.add_argument("--startseite", type=float, default=None,
                    help="side length of the square that must fit at every start pose (m); "
                         "task T2 drives exactly that square")
    ap.add_argument("--offen-max", type=float, default=None,
                    help="maximum share of free cells with a fully free 3x3 neighbourhood; "
                         "0.25 for a maze, larger for an arena")
    ap.add_argument("--cell", type=float, default=None)
    args = ap.parse_args(argv)
    cfg = load_config()
    if args.cell:
        cfg.setdefault("worlds", {})["cell"] = args.cell
    # How much free space around a start pose does a world need? config/tasks.json says it:
    # only tasks with a side length ("seite") — currently T2 — drive a square there.
    try:
        from mecanum_lab import tasks as T
        quadrat_seiten = {}
        for auftrag in T.load_tasks().get("tasks", []):
            if auftrag.get("seite"):
                welt = auftrag.get("welt", "production")
                quadrat_seiten[welt] = max(quadrat_seiten.get(welt, 0.0), float(auftrag["seite"]))
    except Exception as exc:
        print(f"  note: config/tasks.json not readable ({exc}), square check disabled")
        quadrat_seiten = {}
    fehler = 0
    for name in ([args.welt] if args.welt else list_worlds()):
        seite = args.startseite or quadrat_seiten.get(name, 0.0)   # square drives only

        meldung, ok = pruefe(name, cfg, args.frei, seite)
        print(("  ok   " if ok else "  FAIL ") + meldung[0])
        for zeile in meldung[1:]:
            print(f"         {zeile}")
        fehler += 0 if ok else 1
    print(f"\n{fehler} worlds with problems")
    return 1 if fehler else 0


if __name__ == "__main__":
    sys.exit(main())
