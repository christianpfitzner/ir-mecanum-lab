#!/usr/bin/env python3
"""LOC-Waache: vergleicht die Groessen der Module mit dem Budget aus docs/CONTRACT.md.

Nur fuer den Betreuer (nicht Teil der Studierenden-Uebung):
    python3 tools/loc.py            # Tabelle + Abweichungen
    python3 tools/loc.py --strict   # Exit 1 bei uberzogenen Rahmen
"""
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# Datei -> Budget (CONTRACT §7). Kommentare und Leerzeilen zaehlen mit, das ist die
# Groesse, die ein Studierender beim Lesen sieht.
# Versuch 1 (CONTRACT §7) plus Versuch 2 (CONTRACT-KF §6): die Rahmen des zweiten
# Versuchs sind begruendet, weil IMU-Modell, KF-Bewertung, Konfigurations-Schichten und
# Messprotokoll neue Randbedingungen sind — nicht, weil Module aufgeblaht werden duerften.
# Was Studierende lesen muessen (robot_io.py, Themen in types.py), bleibt klein.
#
# Nachtrag Integration Versuch 2 (Begruendung nach CONTRACT §7, Zahlen gemessen):
#   engine.py  300 -> 340   ein Roboter wird je KF-Auftrag an der Spawn-Pose abgesetzt
#                           (reset_robot), Sensoruhren folgen der Simulationszeit
#                           (_zeitbezug), Auftrag mit eigenem Funkloch (set_task),
#                           config_json/robots_info fuer Themen und Protokoll.
#   node.py    465 -> 520   Halle aus den Aufgaben (cfg_get_welt), erster_auftrag fuer
#                           /sim/task, die Roboterliste jetzt laufend (simlauf) und die
#                           einlaufende kf/pose im Messprotokoll. Das ist Taktgebung, die
#                           vorher fehlte — deshalb war Versuch 1 stumm kaputt.
#   grade.py   610 -> 620   KF-Schrittplan, blinde Kommandofahrt, RMSE/NEES/Funkloch.
#                           Die Alternative (zweite Datei kf_grade.py) verschiebt die
#                           Zeilen nur und verdoppelt den Stichprobenpfad.
#   student/kf_template.py 160 -> 190   Der Anschlusscode erklaert jetzt auch, warum ein
#                           Fix vom vorigen Auftrag zu verwerfen ist — eine Falle, die
#                           sonst jeder Lauf ins Stolpern bringt.
#   tools/kfplot.py        190 -> 250   ASCII-Diagramm inkl. Achsenbeschriftung und
#                           Diagnosezeilen; ersetzt matplotlib, das es hier nicht gibt.
BUDGET = {
    "mecanum_lab/types.py": 330, "mecanum_lab/stub.py": 115,
    "mecanum_lab/engine.py": 340, "mecanum_lab/worlds.py": 110,
    "mecanum_lab/physics.py": 140, "mecanum_lab/sensors.py": 295,
    "mecanum_lab/render.py": 335, "mecanum_lab/ros_bridge.py": 455,
    "mecanum_lab/node.py": 520, "mecanum_lab/robot_io.py": 255,
    "mecanum_lab/tasks.py": 170, "mecanum_lab/grade.py": 620,
    "mecanum_lab/logbook.py": 100,
    "student/controller_template.py": 125, "student/solution.py": 310,
    "student/kf_template.py": 190, "student/kf_solution.py": 295,
    "lab": 65, "launch/sim.launch.py": 60, "launch/student.launch.py": 50,
    "launch/lab.launch.py": 60, "launch/kf.launch.py": 160,
    "tools/kfplot.py": 250, "tools/fastgrade.py": 140,
}
SIM_CORE = [k for k in BUDGET if k.startswith("mecanum_lab/")]
CORE_TOTAL = 3700


def loc(path):
    full = os.path.join(ROOT, path)
    if not os.path.exists(full):
        return None
    with open(full, encoding="utf-8", errors="replace") as fh:
        lines = fh.read().splitlines()
    code = [l for l in lines if l.strip() and not l.strip().startswith("#")]
    return len(lines), len(code)


def main():
    strict = "--strict" in sys.argv
    over, core = [], 0
    print(f"{'Datei':34} {'Zeilen':>7} {'Code':>6} {'Budget':>7}  Status")
    for path, budget in BUDGET.items():
        n = loc(path)
        if n is None:
            print(f"{path:34} {'—':>7} {'—':>6} {budget:>7}  fehlt")
            continue
        total, code = n
        if path in SIM_CORE:
            core += total
        flag = "ok" if total <= budget else f"+{total - budget} ({100 * total / budget:.0f} %)"
        if total > budget:
            over.append((path, total, budget))
        print(f"{path:34} {total:>7} {code:>6} {budget:>7}  {flag}")
    rest = sum(loc(p)[0] for p in BUDGET if p not in SIM_CORE and loc(p))
    print(f"\nSimulator-Kern (mecanum_lab/): {core} Zeilen (Rahmen {CORE_TOTAL})")
    print(f"Rest (CLI, Launch, Studierende): {rest} Zeilen")
    print(f"Gesamt: {core + rest} Zeilen")
    if over:
        print("\nueber dem Budget: " + ", ".join(f"{p} ({t}/{b})" for p, t, b in over))
    return 1 if (strict and (over or core > CORE_TOTAL)) else 0


if __name__ == "__main__":
    sys.exit(main())
