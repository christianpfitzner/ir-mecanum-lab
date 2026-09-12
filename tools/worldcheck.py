#!/usr/bin/env python3
"""Welten-Prüfer: läuft eine worlds/*.txt wirklich für den Versuch?

Studierende dürfen eigene Umgebungen bauen (worlds/<name>.txt, Zeichen laut
worlds.py). Dieser Prüfter meldet die vier Fehler, die im Praktikum am meisten
Nerven kosten: Start oder Ziel in einer Wand, zu enge Stellen für den Roboter,
kein Weg vom Start zum Ziel, und Startposen, die sich gegenseitig blockieren.

    python3 tools/worldcheck.py                 # alle Welten
    python3 tools/worldcheck.py --welt maze --frei 0.35
Exit-Code 1, wenn eine Welt unbrauchbar ist (für tools/check.sh).
"""
import argparse
import collections
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mecanum_lab.types import Rect, cfg_get, load_config     # noqa: E402
from mecanum_lab.worlds import list_worlds, parse_grid       # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RICHTUNGEN = ((1, 0), (-1, 0), (0, 1), (0, -1))


def zeilen(pfad):
    with open(pfad, encoding="utf-8") as fh:
        text = fh.read()
    return [z for z in text.splitlines()]


def pruefe(name, cfg, frei_hebe=0.0, startseite=1.0, offen_max=None):
    """Eine Welt auf Brauchbarkeit prüfen; returns (meldungen, in_ordnung)."""
    pfad = os.path.join(ROOT, "worlds", f"{name}.txt")
    if not os.path.exists(pfad):
        return [f"{name}: Datei fehlt"], False
    zellen = [list(z) for z in zeilen(pfad)]
    breite = max(len(z) for z in zellen)
    zellen = [z + [" "] * (breite - len(z)) for z in zellen]
    hoehe, zell = len(zellen), cfg_get(cfg, "worlds.cell", 0.5)
    radius = cfg_get(cfg, "robot.footprint_r", 0.21) + frei_hebe

    def wand(r, k):
        return 0 <= r < hoehe and 0 <= k < len(zellen[r]) and zellen[r][k] == "#"

    def mittelpunkt(r, k):
        return (k + 0.5) * zell, (hoehe - 1 - r + 0.5) * zell

    welt = parse_grid("\n".join("".join(z) for z in zellen), zell, name)

    def abstand(x, y):
        """Luftlinie Punkt -> nächste Wand (Rechtecke aus worlds.py, Punkt rein_clampen)."""
        if not welt.walls:
            return math.inf
        return min(math.dist((x, y), (min(max(x, w.x0), w.x1), min(max(y, w.y0), w.y1)))
                   for w in welt.walls)

    # Freiheit je Zelle EINMAL ausrechnen (Nachbarschaftssuche fragt sie Tausende Male ab;
    # sonst wird die Prüfung quadratisch und ein Labyrinth läuft nicht mehr durch).
    freiheit = {(r, k): abstand(*mittelpunkt(r, k))
                for r in range(hoehe) for k in range(len(zellen[r])) if not wand(r, k)}

    def wandabstand(r, k):
        return freiheit.get((r, k), 0.0)

    reich = startseite + cfg_get(cfg, "robot.footprint_r", 0.21) + 0.05

    def quadratfrei(r, k):
        """Hat die Startpose `startseite` + Puffer nach allen Seiten frei? -> Zelle oder None.

        Auftrag T2 fährt sein Quadrat in Startrichtung, und die Startrichtung wechselt
        je Teilnehmer (0/90/180/270 Grad, siehe worlds.py). Geprüft wird deshalb die
        Hülle über alle vier Richtungen: ein Kasten um die Pose herum.
        """
        x, y = mittelpunkt(r, k)
        for rr in range(hoehe):
            for kk in range(len(zellen[rr])):
                if not wand(rr, kk):
                    continue
                ku, ko = kk * zell, (hoehe - 1 - rr) * zell      # Zelle: x0, y_unten
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
        meldung.append("keine Startpose (S) in der Datei")
        ok = False
    if ziel is None:
        meldung.append("kein Ziel (G) in der Datei — T3 und T4 haben kein Ziel")
    for r, k in list(starts) + ([ziel] if ziel is not None else []):
        if wand(r, k):
            meldung.append(f"{'Start' if (r, k) in starts else 'Ziel'} in Wand (Zeile {r}, Spalte {k})")
            ok = False
        elif wandabstand(r, k) < radius:
            meldung.append(f"{'Start' if (r, k) in starts else 'Ziel'} zu eng: {wandabstand(r, k):.2f} m "
                           f"frei, Roboter braucht {radius:.2f} m (Zeile {r}, Spalte {k})")
            ok = False
        elif (r, k) in starts:
            block = quadratfrei(r, k)
            if block:
                meldung.append(f"Start (Zeile {r}, Spalte {k}) braucht {reich:.2f} m Freiheit nach "
                               f"allen Seiten: Wand in Zeile {block[0]}, Spalte {block[1]} "
                               f"(T2-Quadratfahrt, Startrichtung wechselt je Teilnehmer)")
                ok = False

    # Kürzester Zellweg Start -> Ziel (4er-Nachbarschaft, reiner Zellenzusammenhang)
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
            meldung.append("Ziel vom Start aus nicht erreichbar (4er-Zusammenhang)")
            ok = False
        else:                                       # breitester Weg: größte Mindestfreiheit
            # Nicht der kürzeste Zellweg zählt (der schmiegt sich an Wände), sondern der
            # weiteste: die größte Freiheit b, bei der Start und Ziel überhaupt über Zellen
            # mit mindestens b Freiheit verbunden sind. "verbunden bei b" wird mit wachsendem
            # b immer schlechter -> binäre Suche über die vorkommenden Freiheitswerte, jede
            # Stufe ein simpler Flutungsdurchlauf. (Ein Maximin-Dijkstra mit Nachträgen
            # braucht hier Minuten, weil ein Labyrinth sehr viele verschiedene Breiten hat.)
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
            for start in starts:                    # jeder Teilnehmer startet woanders
                lo, hi = 0, len(werte)              #Invariant: alle < lo sind erfuellt
                while lo < hi:
                    mitte = (lo + hi) // 2
                    if verbunden_ab(start, werte[mitte]):
                        lo = mitte + 1
                    else:
                        hi = mitte
                eng = max(eng, werte[lo - 1] if lo else 0.0)
            meldung.append(f"breitester Weg Start->Ziel: engste Stelle {eng:.2f} m frei "
                           f"(nötig {radius:.2f} m)" + ("" if eng >= radius else "  << zu schmal"))
            if eng < radius:
                ok = False

    # Labyrinth oder Halle? Freie Zellen mit völlig freiem 3x3-Umfeld sind "offen" — in
    # einem echten Labyrinth gibt es davon kaum eine, in einer Halle mit Tischen viele.
    freie = [(r, k) for r in range(hoehe) for k in range(len(zellen[r])) if not wand(r, k)]
    if freie and offen_max is not None:
        offen = [cell for cell in freie
                 if all(not wand(cell[0] + dr, cell[1] + dk)
                        for dr in (-1, 0, 1) for dk in (-1, 0, 1))]
        anteil = len(offen) / len(freie)
        meldung.append(f"Anteil offener Zellen (3x3 Umfeld frei): {anteil:.2f} "
                       f"({len(offen)}/{len(freie)}) — Grenze {offen_max:.2f}"
                       + ("" if anteil <= offen_max else "  << zu offen, das ist eine Halle"))
        if anteil > offen_max:
            ok = False
    gruesse = (breite * zell, hoehe * zell)
    meldung.insert(0, f"{name}: {gruesse[0]:.1f} x {gruesse[1]:.1f} m, {len(welt.walls)} Rechtecke, "
                      f"{len(starts)} Starts, Ziel {'ja' if ziel else 'nein'}")
    return meldung, ok


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Welten auf Brauchbarkeit prüfen")
    ap.add_argument("--welt", default=None, help="nur eine Welt prüfen")
    ap.add_argument("--frei", type=float, default=0.25,
                    help="Zuschlag auf den Roboterradius für Spielraum (m)")
    ap.add_argument("--startseite", type=float, default=None,
                    help="Seitenlänge des Quadrats, das an jeder Startpose Platz haben muss (m); "
                         "Auftrag T2 fährt genau dieses Quadrat")
    ap.add_argument("--offen-max", type=float, default=None,
                    help="maximaler Anteil freier Zellen mit völlig freiem 3x3-Umfeld; "
                         "0.25 fuer ein Labyrinth, groesser fuer eine Halle")
    ap.add_argument("--cell", type=float, default=None)
    args = ap.parse_args(argv)
    cfg = load_config()
    if args.cell:
        cfg.setdefault("worlds", {})["cell"] = args.cell
    # Welche Welt braucht wieviel freien Platz um eine Startpose? Das sagt config/tasks.json:
    # nur Auftraege mit einer Seitenlaenge ("seite") — aktuell T2 — fahren dort ein Quadrat.
    try:
        from mecanum_lab import tasks as T
        quadrat_seiten = {}
        for auftrag in T.load_tasks().get("tasks", []):
            if auftrag.get("seite"):
                welt = auftrag.get("welt", "production")
                quadrat_seiten[welt] = max(quadrat_seiten.get(welt, 0.0), float(auftrag["seite"]))
    except Exception as exc:
        print(f"  Hinweis: config/tasks.json nicht lesbar ({exc}), Quadrat-Pruef aus")
        quadrat_seiten = {}
    fehler = 0
    for name in ([args.welt] if args.welt else list_worlds()):
        seite = args.startseite or quadrat_seiten.get(name, 0.0)   # Quadratfahrt gibt es nur

        meldung, ok = pruefe(name, cfg, args.frei, seite)
        print(("  ok   " if ok else "  FEHLT ") + meldung[0])
        for zeile in meldung[1:]:
            print(f"         {zeile}")
        fehler += 0 if ok else 1
    print(f"\n{fehler} Welten mit Problemen")
    return 1 if fehler else 0


if __name__ == "__main__":
    sys.exit(main())
