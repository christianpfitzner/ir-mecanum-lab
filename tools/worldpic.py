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
    python3 tools/worldpic.py --out /tmp/w.png --massstab 28
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
from worldcheck import pruefe                                    # noqa: E402

MASSSTAB = 20.0                                    # pixels per metre, the same in every panel
POLSTER, TITEL_H, UNTER, LEGENDE = 16, 30, 54, 46  # panel padding, title and caption strips
RAND, SPALTEN = 24, 2                              # gaps between panels, columns of the grid
BG, TITEL, TEXT = (248, 248, 251), (36, 40, 52), (98, 104, 118)
BODEN, WAND = (.13, .14, .17), (.42, .45, .52)     # floor and wall, close to the GUI's gui_style
ZIEL, HELL = (255, 226, 110), (238, 241, 247)
FREI = 0.25                                        # same clearance the world checker defaults to
ENG = re.compile(r"narrowest point ([\d.]+) m free \(needs ([\d.]+) m\)")


def farben(cfg: dict) -> tuple:
    """Floor, wall, edge and paint colours — the same gui_style values the window reads."""
    stil = cfg_get(cfg or {}, "gui_style") or {}
    boden = tuple(int(255 * v) for v in stil.get("floor", BODEN))
    wand = tuple(int(255 * v) for v in stil.get("wall", WAND))
    kant = tuple((2 * c + 255) // 3 for c in wand)             # lit edge of a solid body
    malung = tuple((3 * c + 255) // 4 for c in boden)          # painted line, clearly not wall
    return boden, kant, wand, malung


def aufgaben_pro_welt() -> dict:
    """{world: "T1 T2 T3 T4"} — which tasks name which world, straight out of tasks.json."""
    try:
        with open(os.path.join(ROOT, "config", "tasks.json"), encoding="utf-8") as fh:
            aufgaben = json.load(fh)["tasks"]
    except (OSError, ValueError, KeyError):
        return {}
    welten = {}
    for auftrag in aufgaben:
        kurz = (auftrag.get("titel") or auftrag["id"]).split()[0]
        welten.setdefault(auftrag.get("welt", "production"), []).append(kurz)
    return {name: " ".join(kurz) for name, kurz in welten.items()}


def durchlass(name: str, cfg: dict) -> str:
    """'tightest passage 1.25 m, robot needs 0.46 m' — straight from the world checker."""
    try:
        meldungen, _ = pruefe(name, cfg, FREI)
    except Exception as exc:                                   # the picture is still drawn
        return f"passage width unavailable ({type(exc).__name__})"
    for zeile in meldungen:
        treffer = ENG.search(zeile)
        if treffer:
            return f"tightest passage {treffer.group(1)} m, robot needs {treffer.group(2)} m"
    return "no start->goal pair (free driving)"


def textzeilen(schrift, text: str, breite: int):
    """Wrap a caption to the panel width — a clipped sentence helps nobody."""
    zeilen, aktuelle = [], ""
    for wort in text.split():
        kandidat = f"{aktuelle} {wort}".strip()
        if aktuelle and schrift.size(kandidat)[0] > breite:
            zeilen.append(aktuelle)
            aktuelle = wort
        else:
            aktuelle = kandidat
    if aktuelle:
        zeilen.append(aktuelle)
    return zeilen


def _px(herkunft, s, x, y):
    """World metres -> pixels: origin bottom left, y grows upwards."""
    return (herkunft[0] + x * s, herkunft[1] - y * s)


def _strich(sc, a, b, farbe, stich=5, luecke=4):
    """Dashed line — the only honest way to draw paint on the floor."""
    (x0, y0), (x1, y1) = a, b
    laenge = math.hypot(x1 - x0, y1 - y0) or 1.0
    dx, dy = (x1 - x0) / laenge, (y1 - y0) / laenge
    zurueck, an = 0.0, True
    while zurueck < laenge:
        schritt = min(stich if an else luecke, laenge - zurueck)
        if an:
            pygame.draw.line(sc, farbe, (x0 + dx * zurueck, y0 + dy * zurueck),
                             (x0 + dx * (zurueck + schritt), y0 + dy * (zurueck + schritt)), 1)
        zurueck, an = zurueck + schritt, not an


def feld(sc, welt, ecke, s, paare):
    """Draw one world with its bottom left corner at `ecke` (pixels)."""
    boden, kant, wand, malung = paare
    breite, hoehe = int(welt.size[0] * s), int(welt.size[1] * s)
    rahmen = pygame.Rect(ecke[0], int(ecke[1] - hoehe), breite, hoehe)
    sc.fill(boden, rahmen)
    for mauer in welt.walls or []:
        linke = _px(ecke, s, mauer.x0, mauer.y1)
        leib = pygame.Rect(linke, (max(2, int((mauer.x1 - mauer.x0) * s)),
                                   max(2, int((mauer.y1 - mauer.y0) * s))))
        pygame.draw.rect(sc, wand, leib)
        pygame.draw.rect(sc, kant, leib, 1)              # reads as a body, not as a stroke
    for x0, y0, x1, y1 in welt.markings or []:
        _strich(sc, _px(ecke, s, x0, y0), _px(ecke, s, x1, y1), malung)
    pygame.draw.rect(sc, kant, rahmen, 2)                # where the world ends
    if welt.goal:
        mitte = _px(ecke, s, welt.goal.x, welt.goal.y)
        for ring, rad in enumerate((13, 8, 3)):
            pygame.draw.circle(sc, HELL if ring == 1 else ZIEL, mitte, rad, 2 if ring else 0)
    for nr, start in enumerate(welt.spawns or []):
        mitte, farbe = _px(ecke, s, start.x, start.y), _palette(nr)
        pygame.draw.circle(sc, farbe, mitte, 7)
        pygame.draw.line(sc, farbe, mitte, (mitte[0] + 15 * math.cos(start.theta),
                                           mitte[1] - 15 * math.sin(start.theta)), 2)
        pygame.draw.circle(sc, (18, 20, 26), mitte, 7, 1)


def _palette(nr: int) -> tuple:
    """Robot colour n from types.PALETTE (0..1 floats there, 0..255 on the surface)."""
    return tuple(min(255, int(255 * w)) for w in PALETTE[nr % len(PALETTE)][1])


def legende(sc, y, schrift, paare, s):
    """Drawn, not typeset: the default pygame font has no reliable symbol glyphs."""
    boden, kant, wand, malung = paare
    x, zeile = RAND, y
    for text, male in (("wall (collision)",
                        lambda cx, cy: pygame.draw.rect(sc, wand, (cx - 9, cy - 6, 18, 12))),
                       ("floor marking (paint, no collision)",
                        lambda cx, cy: _strich(sc, (cx - 16, cy), (cx + 16, cy), malung)),
                       ("start pose — colour = robot n",
                        lambda cx, cy: pygame.draw.circle(sc, _palette(0), (cx, cy), 6)),
                       ("goal",
                        lambda cx, cy: [pygame.draw.circle(sc, ZIEL, (cx, cy), 7),
                                        pygame.draw.circle(sc, HELL, (cx, cy), 4, 2)])):
        bild = schrift.render(text, True, TEXT)
        if x + 64 + bild.get_width() > sc.get_width() - RAND:      # flow into the next row
            x, zeile = RAND, zeile + 20
        male(x + 18, zeile)
        sc.blit(bild, (x + 40, zeile - 8))
        x += 64 + bild.get_width()
    laenge = int(5 * s)
    if x + 20 + laenge + 40 > sc.get_width() - RAND:
        x, zeile = RAND, zeile + 20
    pygame.draw.line(sc, TEXT, (x, zeile), (x + laenge, zeile), 2)   # one scale, all panels
    for rand_px in (x, x + laenge):
        pygame.draw.line(sc, TEXT, (rand_px, zeile - 5), (rand_px, zeile + 5), 2)
    sc.blit(schrift.render("5 m", True, TEXT), (x + laenge + 8, zeile - 8))


def bilde(namen, pfad: str, zoom: float = 1.0) -> str:
    """Render the given worlds at one common scale into one PNG and return the path."""
    cfg = load_config()
    welten = [(name, load_world(name, cfg=cfg)) for name in namen]
    pygame.init()
    gross, klein = pygame.font.Font(None, 32), pygame.font.Font(None, 23)
    paare, s = farben(cfg), MASSSTAB
    messen = [(int(w.size[0] * s) + 2 * POLSTER, int(w.size[1] * s) + 2 * POLSTER)
              for _, w in welten]
    spalten = max(1, min(SPALTEN, len(welten)))
    reihen = -(-len(welten) // spalten)
    spalt_b = [max([messen[i][0] for i in range(len(welten)) if i % spalten == c] or [0])
               for c in range(spalten)]
    reihen_h = [max([messen[i][1] for i in range(len(welten)) if i // spalten == r] or [0])
                for r in range(reihen)]
    streifen = TITEL_H + UNTER                       # title + caption around each panel
    sc = pygame.Surface((sum(spalt_b) + (spalten + 1) * RAND,
                         sum(reihen_h) + reihen * streifen + 2 * RAND + LEGENDE))
    sc.fill(BG)
    aufgaben = aufgaben_pro_welt()
    for nr, (name, welt) in enumerate(welten):
        spalte, reihe = nr % spalten, nr // spalten
        einschub = (spalt_b[spalte] - messen[nr][0]) // 2          # centre in the wider column
        x = RAND + sum(spalt_b[:spalte]) + (spalte + 1) * RAND + einschub
        y = RAND + sum(reihen_h[:reihe]) + reihe * (streifen + RAND)
        sc.blit(gross.render(f"{name}  —  {welt.size[0]:g} × {welt.size[1]:g} m  ·  "
                             f"cell {welt.cell:g} m", True, TITEL), (x, y))
        feld(sc, welt, (x + POLSTER, y + TITEL_H + messen[nr][1] - POLSTER), s, paare)
        rand_text = (f"{aufgaben.get(name, 'no task names it — free driving')}  ·  "
                     f"{durchlass(name, cfg)}")
        for nr_z, zeile in enumerate(textzeilen(klein, rand_text, messen[nr][0])):
            sc.blit(klein.render(zeile, True, TEXT),
                    (x, y + TITEL_H + messen[nr][1] + 10 + 17 * nr_z))
    legende(sc, sc.get_height() - LEGENDE + 20, klein, paare, s)
    if zoom != 1.0:
        sc = pygame.transform.smoothscale(sc, (int(sc.get_width() * zoom),
                                              int(sc.get_height() * zoom)))
    os.makedirs(os.path.dirname(os.path.abspath(pfad)), exist_ok=True)
    pygame.image.save(sc, pfad)
    print(f"{pfad}: {sc.get_width()}×{sc.get_height()} px · {', '.join(namen)}")
    pygame.quit()
    return pfad


def main(argv=None):
    ap = argparse.ArgumentParser(description="Draw every worlds/*.txt into one PNG.")
    ap.add_argument("--out", default=os.path.join(ROOT, "docs", "img", "worlds.png"))
    ap.add_argument("--welten", default=",".join(list_worlds()), help="comma-separated names")
    ap.add_argument("--zoom", type=float, default=1.0, help="enlarge the whole figure")
    ap.add_argument("--massstab", type=float, default=MASSSTAB, help="pixels per metre")
    args = ap.parse_args(argv)
    globals()["MASSSTAB"] = args.massstab
    bilde([w for w in args.welten.split(",") if w.strip()], args.out, args.zoom)


if __name__ == "__main__":
    main()
