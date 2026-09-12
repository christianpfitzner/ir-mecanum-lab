#!/usr/bin/env python3
"""Messprotokoll aus `./lab ... --log DATEI.csv` auswerten — Zahlen und Diagramm.

Ohne matplotlib reicht es fuer die Zahlen im Bericht und ein ASCII-Bild der Fehler-
zeitreihe; mit matplotlib (optional, nicht installiert) entsteht auf Wunsch ein PNG.
Spalten und deren Bedeutung stehen in `mecanum_lab/logbook.py` (Semikolon, Dezimalpunkt).

    python3 tools/kfplot.py messung.csv                       # Zahlen + ASCII-Bild
    python3 tools/kfplot.py messung.csv --sensor odom         # Rohsensor vergleichen
    python3 tools/kfplot.py messung.csv --intervall 1         # 1-s-Mittel im Bild
    python3 tools/kfplot.py messung.csv --list                # Spalten zeigen
    python3 tools/kfplot.py messung.csv -o bild.png           # PNG, falls matplotlib da
"""
import argparse
import csv
import math
import sys

ROHSENSOR = {"gps": ("x_gps", "y_gps"), "odom": ("x_odom", "y_odom")}


def zahl(wert):
    """Eine CSV-Zelle als Zahl — leere Zellen und 'nan' sind None, kein Abbruch."""
    try:
        f = float(wert)
    except (TypeError, ValueError):
        return None
    return None if math.isnan(f) else f


def lese(pfad):
    try:
        with open(pfad, newline="", encoding="utf-8", errors="replace") as fh:
            reihen = list(csv.DictReader(fh, delimiter=";"))
    except OSError as exc:
        sys.exit(f"Datei '{pfad}' nicht lesbar: {exc}")
    if not reihen or "t" not in (reihen[0] or {}):
        sys.exit(f"'{pfad}' ist kein Messprotokoll — erwartet wird Semikolon-CSV mit "
                 f"Kopfzeile 't;robot;x_wahr;...' (siehe ./lab --log DATEI.csv).")
    return reihen


def nach_roboter(reihen):
    gruppen = {}
    for r in reihen:
        gruppen.setdefault(r.get("robot") or "?", []).append(r)
    for liste in gruppen.values():
        liste.sort(key=lambda r: zahl(r.get("t")) or 0.0)
    return gruppen


def messtreue(reihen, sensor):
    """Fehlerzeiten: (t, Fehler) je Zeile, NEES-Stichproben und die GPS-Fix-Stempel."""
    px, py = ROHSENSOR[sensor]
    kf, roh, nees, fixes = [], [], [], []
    for r in reihen:
        t, xw, yw = zahl(r.get("t")), zahl(r.get("x_wahr")), zahl(r.get("y_wahr"))
        if t is None or xw is None or yw is None:
            continue                                        # ohne Wahrheit nichts gemessen
        xa, ya = zahl(r.get(px)), zahl(r.get(py))
        if xa is not None:
            roh.append((t, math.hypot(xa - xw, ya - yw)))
        xk, yk = zahl(r.get("x_kf")), zahl(r.get("y_kf"))
        if xk is not None:
            kf.append((t, math.hypot(xk - xw, yk - yw)))
            s2 = [zahl(r.get(s)) for s in ("sx_kf", "sy_kf")]
            if all(s and s > 1e-9 for s in s2):
                nees.append((((xk - xw) ** 2) / s2[0] ** 2 + ((yk - yw) ** 2) / s2[1] ** 2) / 2)
        alt = zahl(r.get("t_alt_gps"))
        if alt is not None:
            fixes.append(round(t - alt, 2))                 # rueckgerechneter Fix-Stempel
    return kf, roh, nees, fixes


def fix_abstaende(fixes):
    """Abstände zweier verschiedener GPS-Fixe in s — aus den Stempeln, nicht der Rate.

    Der Fix-Stempel steht nicht in der Datei, nur sein Alter in `t_alt_gps`; zurückgerechnet
    entstehen Nachkommastellen-Runzeln, die wie neue Fixe aussehen. Deshalb auf 2 cm Zeit
    glätten und alles unter 20 ms als Jitter verwerfen.
    """
    folgen, abstaende = None, []
    for t in sorted(set(fixes)):
        if folgen is not None and t - folgen > 0.02:
            abstaende.append(t - folgen)
        folgen = t
    return abstaende


def mittle(punkte, intervall):
    """Mittelwerte je Intervall — gegen Rauschen liest eine Zeitreihe so viel besser."""
    if not punkte or not intervall or intervall <= 0:
        return punkte
    karten = {}
    for t, e in punkte:
        karten.setdefault(int(t / intervall), []).append((t, e))
    out = []
    for schluessel in sorted(karten):
        gruppe = karten[schluessel]
        out.append((sum(t for t, _ in gruppe) / len(gruppe),
                    sum(e for _, e in gruppe) / len(gruppe)))
    return out


def rmse(punkte):
    return math.sqrt(sum(e * e for _, e in punkte) / len(punkte)) if punkte else None


def ascii_bild(kurven, breite=76, hoehe=13):
    """Fehlerzeitreihen als Text: kurven = [(Name, Zeichen, punkte), ...] — Breite <= 100."""
    kurven = [(n, z, p) for n, z, p in kurven if p]
    if not kurven:
        return "  (keine Fehlerdaten)"
    tmin = min(t for _, _, p in kurven for t, _ in p)
    tmax = max(t for _, _, p in kurven for t, _ in p)
    ymax = max(max(e for _, e in p) for _, _, p in kurven) or 1.0
    raster = [[" "] * breite for _ in range(hoehe)]
    for _, zeichen, punkte in kurven:
        for t, e in punkte:
            spalte = int(round((t - tmin) / (tmax - tmin) * (breite - 1))) if tmax > tmin else 0
            zeile = int(round((ymax - e) / ymax * (hoehe - 1)))
            raster[max(0, min(hoehe - 1, zeile))][spalte] = zeichen
    legende = "   ".join(f"{z} = {n}" for n, z, _ in kurven)
    ausgabe = [f"Fehler gegen die Wahrheit [m], Skala 0 … {ymax:.2f} m   ({legende})"]
    for i, zeile in enumerate(raster):
        stab = ymax * (hoehe - 1 - i) / (hoehe - 1)
        ausgabe.append(f"{stab:6.2f} |" + "".join(zeile))
    ausgabe.append("       +" + "-" * breite)
    anfang, ende = f"{tmin:g}s", f"{tmax:g}s"
    ausgabe.append("       " + anfang + " " * max(1, breite + 1 - len(anfang) - len(ende)) + ende)
    return "\n".join(ausgabe)


def png(pfad, reihen, kurven, ziel):
    """Drei Felder: Bahn, Fehler ueber der Zeit, Messraten. Braucht matplotlib (optional)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib fehlt — Diagramme bleiben ASCII (pip install --user matplotlib).")
        return
    fig, feld = plt.subplots(1, 3, figsize=(17, 5))
    bahn = [("Wahrheit", "x_wahr", "y_wahr", "k-"), ("GPS", "x_gps", "y_gps", ".g"),
            ("Odometrie", "x_odom", "y_odom", ".b"), ("KF", "x_kf", "y_kf", "r-")]
    for name, xs, ys, stil in bahn:
        x = [zahl(r.get(xs)) for r in reihen if zahl(r.get(xs)) is not None]
        y = [zahl(r.get(ys)) for r in reihen if zahl(r.get(ys)) is not None]
        if x and y:
            feld[0].plot(x, y, stil, label=name, linewidth=1)
    feld[0].set_title(f"Bahn — {pfad}")
    feld[0].set_xlabel("x [m]"); feld[0].set_ylabel("y [m]")
    feld[0].legend(); feld[0].grid(True, alpha=.3); feld[0].set_aspect("equal")
    for name, _, punkte in kurven:
        feld[1].plot([t for t, _ in punkte], [e for _, e in punkte], label=name)
    feld[1].set_title("Fehler gegen die Wahrheit"); feld[1].set_xlabel("t [s]")
    feld[1].set_ylabel("Fehler [m]"); feld[1].legend(); feld[1].grid(True, alpha=.3)
    sekunden = [int(zahl(r.get("t")) or 0) for r in reihen]
    gps = [s for s, r in zip(sekunden, reihen) if zahl(r.get("t_alt_gps")) is not None]
    kfz = [(int(zahl(r.get("t")) or 0), zahl(r.get("n_kf"))) for r in reihen
           if zahl(r.get("n_kf")) is not None]
    kf_rate = [max(0.0, kfz[i][1] - kfz[i - 1][1]) for i in range(1, len(kfz))
               if kfz[i][0] != kfz[i - 1][0]]
    feld[2].hist([gps, kf_rate], bins=range(0, max(sekunden, default=0) + 2),
                 label=["GPS-Fixe/s", "kf/pose/s"], color=["#4faf6f", "#d1705a"])
    feld[2].set_title("Messraten"); feld[2].set_xlabel("Sekunde"); feld[2].legend()
    fig.tight_layout()
    fig.savefig(ziel, dpi=110)
    print(f"PNG geschrieben: {ziel}")


def haupt():
    e = argparse.ArgumentParser(description="Messprotokoll (./lab --log) auswerten")
    e.add_argument("protokoll", help="CSV-Datei aus ./lab ... --log DATEI.csv")
    e.add_argument("-o", "--output", metavar="DATEI.png", help="PNG schreiben (matplotlib)")
    e.add_argument("--sensor", choices=sorted(ROHSENSOR), default="gps",
                   help="Rohsensor fuer Vergleich und Bild (Standard: gps)")
    e.add_argument("--intervall", type=float, default=0.0, metavar="S",
                   help="Fehlerzeitreihe fuer das Bild ueber S Sekunden mitteln")
    e.add_argument("--robot", default="", help="nur dieser Roboter (Standard: der mit kf-Daten)")
    e.add_argument("--breite", type=int, default=76, help="Breite des ASCII-Bilds (<= 92)")
    e.add_argument("--list", action="store_true", help="Spalten des Protokolls zeigen")
    a = e.parse_args()
    reihen = lese(a.protokoll)
    if a.list:
        print(f"{len(reihen)} Zeilen, Spalten: " + ", ".join((reihen[0] or {}).keys()))
        for spalte, wert in (reihen[len(reihen) // 2] or {}).items():
            print(f"  {spalte:12} {wert!r}")
        return 0
    gruppen = nach_roboter(reihen)
    if a.robot and a.robot not in gruppen:
        sys.exit(f"Roboter '{a.robot}' ist nicht im Protokoll — drin sind: "
                 + ", ".join(sorted(gruppen)))
    name = a.robot or ("" if len(gruppen) == 1 else
                       max(gruppen, key=lambda k: sum(1 for r in gruppen[k] if zahl(r.get("x_kf")))))
    for robot in [name] if name else sorted(gruppen):
        bericht(gruppen[robot], a, robot, len(gruppen))
    return 0


def bericht(reihen, a, robot, anzahl_roboter):
    """Zahlen und ASCII-Bild fuer einen Roboter — das gehört in den Versuchsbericht."""
    kf, roh, nees, fixes = messtreue(reihen, a.sensor)
    t = sorted(x for x in (zahl(r.get("t")) for r in reihen) if x is not None)
    if not t:
        print(f"\n=== {a.protokoll} · Roboter '{robot}' ===\nkeine Zeitstempel in der Datei")
        return
    zusatz = "" if anzahl_roboter == 1 else f" (1 von {anzahl_roboter}, --robot waehlt)"
    print(f"\n=== {a.protokoll} · Roboter '{robot}'{zusatz} ===")
    print(f"Zeilen: {len(reihen)}   Zeit: {t[0]:.1f} … {t[-1]:.1f} s   Rohsensor: {a.sensor}")
    r_kf, r_roh = rmse(kf), rmse(roh)
    if not roh:
        print(f"Kein {a.sensor}-Wert gegen der Wahrheit — Spalten x_{a.sensor}/y_{a.sensor} leer?")
    else:
        print(f"RMSE wahr↔{a.sensor:<4}: {r_roh:6.3f} m   Max: {max(x[1] for x in roh):6.3f} m")
    if kf:
        print(f"RMSE wahr↔kf   : {r_kf:6.3f} m   Max: {max(x[1] for x in kf):6.3f} m")
        if r_roh and r_kf:
            print(f"Verbesserung   : {r_roh / r_kf:5.2f}   (rmse_{a.sensor} / rmse_kf)")
        if nees:
            print(f"NEES-Mittel    : {sum(nees) / len(nees):5.2f}   (1 = ehrliche Standardabweichung)")
        else:
            print("NEES-Mittel    :   —   (sx_kf/sy_kf fehlt — send_kf sigma mitgeben)")
        n = [zahl(r.get("n_kf")) for r in reihen if zahl(r.get("n_kf")) is not None]
        dauer = kf[-1][0] - kf[0][0]
        if dauer > 0:
            meldungen = int(max(n) - min(n)) if len(n) > 1 else len(kf)
            print(f"kf/pose-Rate   : {meldungen / dauer:5.2f} Hz  ({meldungen} Meldungen)")
    else:
        print("Kein einziger kf/pose-Wert im Protokoll: laeuft dein Knoten, und ruft er "
              "rob.send_kf() auf? Diese Datei enthaelt nur die Rohsensorik.")
    abstaende = fix_abstaende(fixes)
    if abstaende:
        mittel = sum(abstaende) / len(abstaende)
        mitte = sorted(abstaende)[len(abstaende) // 2]
        print(f"GPS-Fix-Abstand  : {mittel:.2f} s Mittel, {mitte:.2f} s Median, "
              f"größter {max(abstaende):.2f} s  ({len(set(fixes))} Fixe)")
    else:
        print(f"GPS-Fix-Abstand  : —   ({len(set(fixes))} Fixe)")
    kurven = [(f"{a.sensor}−wahr", "o", mittle(roh, a.intervall)),
              ("kf−wahr", "*", mittle(kf, a.intervall))]
    print()
    print(ascii_bild(kurven, breite=max(20, min(92, a.breite))))
    if a.output:
        png(a.protokoll, reihen, kurven, a.output)


if __name__ == "__main__":
    sys.exit(haupt())
