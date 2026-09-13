#!/usr/bin/env python3
"""One picture of every arena — for the README and for the tutor's screen.

One metre has the same thickness everywhere in this figure: the panels are the worlds at a
common scale, so `maze` (13 × 11 m, 1 m cells) and `arena` (24 × 16 m, 0.5 m cells) are
comparable and a wall is as thick as it really is. Walls are solid blocks with a lit edge;
floor markings (`-` and `|` in the grid) are drawn as dashed paint — they are decoration
without collision and must not read as walls.

Under each panel: which tasks name this world (`config/tasks.json`) and how wide the tightest
passage on the widest start->goal path is (`tools/worldcheck.py`) — the number that says
whether a robot fits through.

Headless (SDL dummy), deterministic, stdlib + pygame only (CONTRACT section 1).

    python3 tools/worldpic.py                       # docs/img/worlds.png
    python3 tools/worldpic.py --out /tmp/w.png --scale 28
"""
import argparse
import json
import math
import os
import re
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")       # no window needed to draw a picture
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame                                           # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools"))
from mecanum_lab.types import PALETTE, cfg_get, load_config      # noqa: E402
from mecanum_lab.worlds import load_world, list_worlds           # noqa: E402
from worldcheck import check                                    # noqa: E402

SCALE = 20.0                                    # pixels per metre, the same in every panel
PADDING, TITLE_H, CAPTION_H, LEGEND = 16, 30, 54, 46  # panel padding, title and caption strips
MARGIN, COLUMNS = 24, 3                             # gaps between panels, columns of the grid
# 3 columns: five arenas, three rows of two at two columns, two rows of three at three. With `open`
# (30 x 20 m) in the set the second shape is the one that stays near square — see docs/img/worlds.png.
BG, TITLE, CAPTION = (248, 248, 251), (36, 40, 52), (98, 104, 118)
FLOOR, WALL = (.13, .14, .17), (.42, .45, .52)     # floor and wall, close to the GUI's gui_style
TARGET, HELL = (255, 226, 110), (238, 241, 247)
FREI = 0.25                                        # same clearance the world checker defaults to
ENG = re.compile(r"narrowest point ([\d.]+) m free \(needs ([\d.]+) m\)")
# A world without a goal has no start->goal path; the checker then reports the widest spot it found
# (tools/worldcheck.py), which is the number this figure quotes for `open` instead.
ENG_FREI = re.compile(r"widest free spot ([\d.]+) m free \(needs ([\d.]+) m\)")


def colors(cfg: dict) -> tuple:
    """Floor, wall, edge and paint colours — the same gui_style values the window reads."""
    stil = cfg_get(cfg or {}, "gui_style") or {}
    boden = tuple(int(255 * v) for v in stil.get("floor", FLOOR))
    wand = tuple(int(255 * v) for v in stil.get("wall", WALL))
    edge = tuple((2 * c + 255) // 3 for c in wand)             # lit edge of a solid body
    malung = tuple((3 * c + 255) // 4 for c in boden)          # painted line, clearly not wall
    return boden, edge, wand, malung


def tasks_per_world() -> dict:
    """{world: "T1 T2 T3 T4"} — which tasks name which world, straight out of tasks.json."""
    try:
        with open(os.path.join(ROOT, "config", "tasks.json"), encoding="utf-8") as fh:
            tasks = json.load(fh)["tasks"]
    except (OSError, ValueError, KeyError):
        return {}
    worlds = {}
    for task in tasks:
        kurz = (task.get("title") or task["id"]).split()[0]
        worlds.setdefault(task.get("world", "production"), []).append(kurz)
    return {name: " ".join(kurz) for name, kurz in worlds.items()}


def clearance(name: str, cfg: dict) -> str:
    """'tightest passage 1.25 m, robot needs 0.46 m' — straight from the world checker."""
    try:
        meldungen, _ = check(name, cfg, FREI)
    except Exception as exc:                                   # the picture is still drawn
        return f"passage width unavailable ({type(exc).__name__})"
    for row_rect in meldungen:
        hits = ENG.search(row_rect)
        if hits:
            return f"tightest passage {hits.group(1)} m, robot needs {hits.group(2)} m"
        hits = ENG_FREI.search(row_rect)
        if hits:
            return (f"widest free spot {hits.group(1)} m, robot needs {hits.group(2)} m — "
                    "nothing to hit")
    return "no start->goal pair (free driving)"


def caption_lines(font, text: str, width: int):
    """Wrap a caption to the panel width — a clipped sentence helps nobody."""
    lines, aktuelle = [], ""
    for word in text.split():
        kandidat = f"{aktuelle} {word}".strip()
        if aktuelle and font.size(kandidat)[0] > width:
            lines.append(aktuelle)
            aktuelle = word
        else:
            aktuelle = kandidat
    if aktuelle:
        lines.append(aktuelle)
    return lines


def _px(origin, s, x, y):
    """World metres -> pixels: origin bottom left, y grows upwards."""
    return (origin[0] + x * s, origin[1] - y * s)


def _dashed(sc, a, b, color, stich=5, outage=4):
    """Dashed line — the only honest way to draw paint on the floor."""
    (x0, y0), (x1, y1) = a, b
    length = math.hypot(x1 - x0, y1 - y0) or 1.0
    dx, dy = (x1 - x0) / length, (y1 - y0) / length
    zurueck, an = 0.0, True
    while zurueck < length:
        step = min(stich if an else outage, length - zurueck)
        if an:
            pygame.draw.line(sc, color, (x0 + dx * zurueck, y0 + dy * zurueck),
                             (x0 + dx * (zurueck + step), y0 + dy * (zurueck + step)), 1)
        zurueck, an = zurueck + step, not an


def cell_size(sc, world, corner, s, pairs):
    """Draw one world with its bottom left corner at `corner` (pixels)."""
    boden, edge, wand, malung = pairs
    width, height = int(world.size[0] * s), int(world.size[1] * s)
    frame = pygame.Rect(corner[0], int(corner[1] - height), width, height)
    sc.fill(boden, frame)
    for wall in world.walls or []:
        left = _px(corner, s, wall.x0, wall.y1)
        fill = pygame.Rect(left, (max(2, int((wall.x1 - wall.x0) * s)),
                                   max(2, int((wall.y1 - wall.y0) * s))))
        pygame.draw.rect(sc, wand, fill)
        pygame.draw.rect(sc, edge, fill, 1)              # reads as a body, not as a stroke
    for x0, y0, x1, y1 in world.markings or []:
        _dashed(sc, _px(corner, s, x0, y0), _px(corner, s, x1, y1), malung)
    pygame.draw.rect(sc, edge, frame, 2)                # where the world ends
    if world.goal:
        mitte = _px(corner, s, world.goal.x, world.goal.y)
        for ring, rad in enumerate((13, 8, 3)):
            pygame.draw.circle(sc, HELL if ring == 1 else TARGET, mitte, rad, 2 if ring else 0)
    for index, start in enumerate(world.spawns or []):
        mitte, color = _px(corner, s, start.x, start.y), _palette(index)
        pygame.draw.circle(sc, color, mitte, 7)
        pygame.draw.line(sc, color, mitte, (mitte[0] + 15 * math.cos(start.theta),
                                           mitte[1] - 15 * math.sin(start.theta)), 2)
        pygame.draw.circle(sc, (18, 20, 26), mitte, 7, 1)


def _palette(index: int) -> tuple:
    """Robot colour n from types.PALETTE (0..1 floats there, 0..255 on the surface)."""
    return tuple(min(255, int(255 * w)) for w in PALETTE[index % len(PALETTE)][1])


def draw_legend(sc, y, font, pairs, s):
    """Drawn, not typeset: the default pygame font has no reliable symbol glyphs."""
    boden, edge, wand, malung = pairs
    x, row_rect = MARGIN, y
    for text, male in (("wall (collision)",
                        lambda cx, cy: pygame.draw.rect(sc, wand, (cx - 9, cy - 6, 18, 12))),
                       ("floor marking (paint, no collision)",
                        lambda cx, cy: _dashed(sc, (cx - 16, cy), (cx + 16, cy), malung)),
                       ("start pose — colour = robot n",
                        lambda cx, cy: pygame.draw.circle(sc, _palette(0), (cx, cy), 6)),
                       ("goal",
                        lambda cx, cy: [pygame.draw.circle(sc, TARGET, (cx, cy), 7),
                                        pygame.draw.circle(sc, HELL, (cx, cy), 4, 2)])):
        bild = font.render(text, True, CAPTION)
        if x + 64 + bild.get_width() > sc.get_width() - MARGIN:      # flow into the next row
            x, row_rect = MARGIN, row_rect + 20
        male(x + 18, row_rect)
        sc.blit(bild, (x + 40, row_rect - 8))
        x += 64 + bild.get_width()
    length = int(5 * s)
    if x + 20 + length + 40 > sc.get_width() - MARGIN:
        x, row_rect = MARGIN, row_rect + 20
    pygame.draw.line(sc, CAPTION, (x, row_rect), (x + length, row_rect), 2)   # one scale, all panels
    for rand_px in (x, x + length):
        pygame.draw.line(sc, CAPTION, (rand_px, row_rect - 5), (rand_px, row_rect + 5), 2)
    sc.blit(font.render("5 m", True, CAPTION), (x + length + 8, row_rect - 8))


def make(names, path: str, zoom: float = 1.0) -> str:
    """Render the given worlds at one common scale into one PNG and return the path."""
    cfg = load_config()
    worlds = [(name, load_world(name, cfg=cfg)) for name in names]
    pygame.init()
    big, small = pygame.font.Font(None, 32), pygame.font.Font(None, 23)
    pairs, s = colors(cfg), SCALE
    messen = [(int(w.size[0] * s) + 2 * PADDING, int(w.size[1] * s) + 2 * PADDING)
              for _, w in worlds]
    columns = max(1, min(COLUMNS, len(worlds)))
    reihen = -(-len(worlds) // columns)
    spalt_b = [max([messen[i][0] for i in range(len(worlds)) if i % columns == c] or [0])
               for c in range(columns)]
    reihen_h = [max([messen[i][1] for i in range(len(worlds)) if i // columns == r] or [0])
                for r in range(reihen)]
    streifen = TITLE_H + CAPTION_H                       # title + caption around each panel
    sc = pygame.Surface((sum(spalt_b) + (columns + 1) * MARGIN,
                         sum(reihen_h) + reihen * streifen + 2 * MARGIN + LEGEND))
    sc.fill(BG)
    tasks = tasks_per_world()
    for index, (name, world) in enumerate(worlds):
        column, reihe = index % columns, index // columns
        einschub = (spalt_b[column] - messen[index][0]) // 2          # centre in the wider column
        x = MARGIN + sum(spalt_b[:column]) + (column + 1) * MARGIN + einschub
        y = MARGIN + sum(reihen_h[:reihe]) + reihe * (streifen + MARGIN)
        sc.blit(big.render(f"{name}  —  {world.size[0]:g} × {world.size[1]:g} m  ·  "
                             f"cell {world.cell:g} m", True, TITLE), (x, y))
        cell_size(sc, world, (x + PADDING, y + TITLE_H + messen[index][1] - PADDING), s, pairs)
        rand_text = (f"{tasks.get(name, 'no task names it — free driving')}  ·  "
                     f"{clearance(name, cfg)}")
        for nr_z, row_rect in enumerate(caption_lines(small, rand_text, messen[index][0])):
            sc.blit(small.render(row_rect, True, CAPTION),
                    (x, y + TITLE_H + messen[index][1] + 10 + 17 * nr_z))
    draw_legend(sc, sc.get_height() - LEGEND + 20, small, pairs, s)
    if zoom != 1.0:
        sc = pygame.transform.smoothscale(sc, (int(sc.get_width() * zoom),
                                              int(sc.get_height() * zoom)))
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    pygame.image.save(sc, path)
    print(f"{path}: {sc.get_width()}×{sc.get_height()} px · {', '.join(names)}")
    pygame.quit()
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description="Draw every worlds/*.txt into one PNG.")
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "img", "worlds.png"))
    ap.add_argument("--worlds", default=",".join(list_worlds()), help="comma-separated names")
    ap.add_argument("--zoom", type=float, default=1.0, help="enlarge the whole figure")
    ap.add_argument("--scale", type=float, default=SCALE, help="pixels per metre")
    args = ap.parse_args(argv)
    globals()["SCALE"] = args.scale
    make([w for w in args.worlds.split(",") if w.strip()], args.out, args.zoom)


if __name__ == "__main__":
    main()
