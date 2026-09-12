#!/usr/bin/env python3
"""One picture of every arena — for the README and for the tutor's screen.

Draws what the GUI shows: floor, merged wall blocks, floor markings, the goal bullseye and
every spawn with its heading. The line under each panel says which tasks name that world, read
from `config/tasks.json` — the picture cannot quietly disagree with the task file.

Headless (SDL dummy), deterministic, stdlib + pygame only (CONTRACT section 1).

    python3 tools/worldpic.py                       # docs/img/worlds.png
    python3 tools/worldpic.py --out /tmp/w.png --zoom 1.4
"""
import argparse
import json
import math
import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")       # no window needed to draw a picture
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame                                           # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mecanum_lab.types import PALETTE, cfg_get, load_config      # noqa: E402
from mecanum_lab.worlds import load_world, list_worlds           # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BG, TITEL, TEXT = (248, 248, 251), (38, 42, 54), (96, 102, 116)
PANEL, RAHMEN = 496, 336                          # panel box; the world box sits inside
RAND, TITEL_H, UNTER, SPALTEN = 22, 28, 44, 2     # margins, title strip, caption strip
LEGENDE = 34                                      # one legend line under the grid
BODEN, WAND = (.13, .14, .17), (.34, .36, .42)    # same defaults as the pygame window
ZIEL, HELL = (255, 226, 110), (236, 239, 246)


def farben(cfg: dict) -> tuple:
    """Floor, wall and border colours — gui_style, but walls a bit brighter.

    The arena walls are 5 cm thin; at figure scale they would vanish in the floor, so the
    picture lightens them. The window itself keeps the original values (render.py).
    """
    stil = cfg_get(cfg or {}, "gui_style") or {}
    boden = tuple(int(255 * v) for v in stil.get("floor", BODEN))
    wand = tuple(int(255 * v) for v in stil.get("wall", WAND))
    wand = tuple(int(0.78 * c + 0.22 * h) for c, h in zip(wand, HELL))
    rand = tuple(min(255, (c + 255) // 2) for c in wand)         # a lit edge, as in the GUI
    return boden, wand, rand


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


def _px(box, s, x, y):
    """World metres -> pixels in the panel: origin bottom left, y growing upwards."""
    return (box.x + x * s, box.bottom - y * s)


def _palette(nr: int) -> tuple:
    """Robot colour n from types.PALETTE (0..1 floats there, 0..255 on the surface)."""
    return tuple(min(255, int(255 * w)) for w in PALETTE[nr % len(PALETTE)][1])


def feld(sc, welt, x, y, boden, wand, rand):
    """Draw one world into the panel box at (x, y); returns scale and the world box."""
    w, h = welt.size
    s = min((PANEL - 28) / w, (RAHMEN - 28) / h)
    box = pygame.Rect(x + (PANEL - w * s) / 2, y + (RAHMEN - h * s) / 2, w * s, h * s)
    pygame.draw.rect(sc, boden, box)
    hell = tuple(min(255, (c + 255) // 2) for c in boden)
    for x0, y0, x1, y1 in welt.markings or []:                 # floor markings, view only
        pygame.draw.line(sc, hell, _px(box, s, x0, y0), _px(box, s, x1, y1), max(1, int(.06 * s)))
    for mauer in welt.walls or []:
        ecke = _px(box, s, mauer.x0, mauer.y1)
        rechteck = pygame.Rect(ecke, (max(1, (mauer.x1 - mauer.x0) * s),
                                      max(1, (mauer.y1 - mauer.y0) * s)))
        pygame.draw.rect(sc, wand, rechteck)
        if rechteck.width < 4 or rechteck.height < 4:      # thin walls need their outline
            pygame.draw.rect(sc, rand, rechteck.inflate(2, 2), 1)
    pygame.draw.rect(sc, rand, box, 2)                         # here the world ends
    if welt.goal:
        mitte = _px(box, s, welt.goal.x, welt.goal.y)
        for ring, rad in enumerate((13, 8, 3)):                # bullseye, like the GUI
            pygame.draw.circle(sc, HELL if ring == 1 else ZIEL, mitte, rad, 2 if ring else 0)
    for nr, start in enumerate(welt.spawns or []):
        mittel, farbe = _px(box, s, start.x, start.y), _palette(nr)
        pygame.draw.circle(sc, farbe, mittel, 7)
        pygame.draw.line(sc, farbe, mittel, (mittel[0] + 14 * math.cos(start.theta),
                                            mittel[1] - 14 * math.sin(start.theta)), 2)
        pygame.draw.circle(sc, (18, 20, 26), mittel, 7, 1)
    return s, box


def massstab(sc, box, s, schrift):
    """One metre as a bar — the panels scale independently, so this keeps them honest."""
    x, y = int(box.x + 8), int(box.bottom - 10)
    pygame.draw.line(sc, HELL, (x, y), (x + s, y), 2)
    for rand_px in (x, x + s):
        pygame.draw.line(sc, HELL, (rand_px, y - 4), (rand_px, y + 4), 2)
    sc.blit(schrift.render("1 m", True, HELL), (x, y - 26))     # above the bar, not on it


def legende(sc, y, schrift, paare):
    """Drawn, not typeset: the default pygame font has no reliable glyphs for symbols."""
    x = RAND
    for nr, (text, mal) in enumerate((
            ("start pose — colour = robot n", lambda cx, cy: pygame.draw.circle(sc, _palette(0),
                                                                                (cx, cy), 6)),
            ("goal", lambda cx, cy: [pygame.draw.circle(sc, ZIEL, (cx, cy), 7),
                                     pygame.draw.circle(sc, HELL, (cx, cy), 4, 2)]),
            ("floor marking (view only, no collision)",
             lambda cx, cy: pygame.draw.line(sc, tuple(min(255, (c + 255) // 2) for c in
                                                      paare[0]), (cx - 9, cy), (cx + 9, cy), 2)))):
        mal(x + 8, y)
        bild = schrift.render(text, True, TEXT)
        sc.blit(bild, (x + 22, y - 8))
        x += 40 + bild.get_width()


def bilde(namen, pfad: str, zoom: float = 1.0) -> str:
    """Render the given worlds into one PNG and return the path."""
    welten = [(name, load_world(name)) for name in namen]
    pygame.init()
    gross, klein = pygame.font.Font(None, 32), pygame.font.Font(None, 24)
    paare = farben(load_config())
    spalten = max(1, min(SPALTEN, len(welten)))
    zeilen = -(-len(welten) // spalten)
    sc = pygame.Surface((spalten * (PANEL + RAND) + RAND,
                         zeilen * (RAHMEN + TITEL_H + UNTER + RAND) + RAND + LEGENDE))
    sc.fill(BG)
    wer = aufgaben_pro_welt()
    for nr, (name, welt) in enumerate(welten):
        x = RAND + (nr % spalten) * (PANEL + RAND)
        y = RAND + (nr // spalten) * (RAHMEN + TITEL_H + UNTER + RAND)
        sc.blit(gross.render(f"{name}  —  {welt.size[0]:g} × {welt.size[1]:g} m", True, TITEL),
                (x, y))
        s, box = feld(sc, welt, x, y + TITEL_H, *paare)
        massstab(sc, box, s, klein)
        sc.blit(klein.render(wer.get(name, "no task names it — free driving and teleop"),
                            True, TEXT), (x, y + TITEL_H + RAHMEN + 14))
    legende(sc, sc.get_height() - LEGENDE + 18, klein, paare)
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
    args = ap.parse_args(argv)
    bilde([w for w in args.welten.split(",") if w.strip()], args.out, args.zoom)


if __name__ == "__main__":
    main()
