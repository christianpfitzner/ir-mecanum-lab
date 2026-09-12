#!/usr/bin/env python3
"""Beschleunigte Bewertung für den Betreuer — gleiche Logik, aber ohne Wanduhr-Takt.

`./lab grade` läuft in Echtzeit (wie im Praktikum). Zum Nachprüfen aller vier Aufträge
sind das schnell fünf Minuten Wartezeit. Dieses Werkzeug schaltet die Sleeps des
Stub-Busses ab und schiebt dieselbe Simulation durch denselben `Grader`-Zustandsautomaten
in `--speed`-facher Simulationszeit pro Sekunde. Voraussetzung: die Knoten rechnen ihre
Fristen über die Simulationszeit in den Messungen (`mess.t`) und nicht über die Wanduhr —
genau so macht es student/solution.py.

    python3 tools/fastgrade.py --task alle --speed 8
    python3 tools/fastgrade.py --task quadrat --controller student/controller_template.py
"""
import argparse
import importlib.util
import json
import os
import sys
import threading
import time

os.environ["MECANUM_ROS"] = "stub"                       # bewusst ohne ROS
os.environ.setdefault("MECANUM_FAST", "1")
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanum_lab import grade, robot_io, setup_logging, tasks as T   # noqa: E402
from mecanum_lab.engine import SimEngine                  # noqa: E402
from mecanum_lab.stub import get_bus                      # noqa: E402
from mecanum_lab.types import Twist, load_config, topic   # noqa: E402
from mecanum_lab.worlds import load_world                 # noqa: E402


def lade_pfad(pfad: str):
    """Studierendenknoten wie robot_io.serve() aus einer Datei importieren."""
    spec = importlib.util.spec_from_file_location("studierender", pfad)
    modul = importlib.util.module_from_spec(spec)
    sys.modules["studierender"] = modul
    spec.loader.exec_module(modul)
    return modul


def knoten(roboter: str, modul, fertig: threading.Event) -> None:
    """Der Studierende im echten Runner — exakt derselbe Pfad wie ./lab run."""
    robot_io.serve(modul, name=roboter, bus=get_bus())


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description="Beschleunigte Bewertung (kein Echtzeit-Takt)")
    ap.add_argument("--robot", default="muster")
    ap.add_argument("--task", default="alle", help="kinematik|quadrat|korridor|gps_anfahrt|alle")
    ap.add_argument("--controller", default="student/solution.py")
    ap.add_argument("--speed", type=float, default=8.0, help="Simulationssekunden pro Sekunde")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--json", default=None)
    ap.add_argument("--world", default=None, help="Standard: die Welt, die die Aufgaben brauchen")
    ap.add_argument("--wanduhr-max", type=float, default=600.0)
    ap.add_argument("--debug", action="store_true", help="alle 2 s Simzeit den inneren Zustand zeigen")
    args = ap.parse_args(argv)
    setup_logging("debug" if args.debug else None)     # MECANUM_LOG kann ueberschreiben

    cfg_aufgaben = T.load_tasks()
    ids = None if args.task in ("alle", "") else [args.task]
    welt_name = args.world or T.welt_fuer(cfg_aufgaben, ids)
    cfg = load_config(None, T.sim_profil(cfg_aufgaben, ids))   # Startprofil: Auftrag 1, je Auftrag kommt sein eigenes
    cfg["gui"] = False

    bus, engine = get_bus(), SimEngine(load_world(welt_name), cfg, seed=args.seed)
    engine.spawn(args.robot)
    # Bus <-> Engine verdrahten — das macht im normalen Betrieb ros_bridge/StubBus in node.py.
    bus.sub("twist", args.robot, lambda t: engine.set_cmd_vel(args.robot, t))
    bus.sub("wheels", args.robot, lambda w: engine.set_wheel_speeds(args.robot, w))
    bus.sub("mission", args.robot, lambda s: engine.set_mission(args.robot, s or ""))
    bus.sub("kf", args.robot, lambda k: engine.set_kf(args.robot, k))
    auftrge = {a["id"]: a for a in cfg_aufgaben["tasks"]}

    def neuer_auftrag(s) -> None:            # je Aufgabe ihr Prüfprofil, wie in node.py
        name = str(s or "")
        auftrag = auftrge.get(name) or {}
        if auftrag.get("art") == "kf" and name != engine.task:
            engine.reset_robot(args.robot)   # Blindfahrt beginnt an der Spawn-Pose
        engine.set_task(name)
        if auftrag.get("sim"):
            engine.set_sensor_profil(auftrag["sim"])

    bus.sub("task", None, neuer_auftrag)
    engine.step(0.001)                                   # erste Messungen erzeugen lassen
    for kind, roboter, payload in engine.drain():
        bus.publish(topic(kind, roboter), payload)
    bus.publish(topic("world"), engine.world_json())
    bus.publish(topic("config"), engine.config_json())
    bus.publish(topic("robots"), json.dumps(engine.robots_info()))
    bus.publish(topic("task"), "")

    bew = grade.Grader(args.robot, ids, bus, cfg_aufgaben,
                       world_info=json.loads(engine.world_json())).start()
    fertig = threading.Event()
    threading.Thread(target=knoten, args=(args.robot, lade_pfad(args.controller), fertig),
                     daemon=True).start()

    dt = 1.0 / float(cfg["rate"])
    pause = dt / max(args.speed, 0.1)            # ein Schritt ist dt Sek. "wert"
    start, letzte_liste, letzte_rueckgabe = time.monotonic(), 0.0, 0.0
    while not bew.tick(dt):
        engine.step(dt)
        for kind, roboter, payload in engine.drain():
            bus.publish(topic(kind, roboter), payload)
        if engine.t - letzte_liste > 0.2:                # /sim/robots mit 5 Hz (Abstand,
            letzte_liste = engine.t                      # Weg, Berührungen, mission_state)
            bus.publish(topic("robots"), json.dumps(engine.robots_info()))
            bus.publish(topic("config"), engine.config_json())
        if args.debug and engine.t - letzte_rueckgabe > 2.0:
            letzte_rueckgabe = engine.t
            r0 = engine.robots.get(args.robot)
            schritt = bew.plan[bew.i] if bew.i < len(bew.plan) else {}
            print(f"  t={engine.t:6.1f} art={schritt.get('art','?'):13} "
                  f"mission={r0.mission_state if r0 else '-':28} mode={r0.mode if r0 else '-':12} "
                  f"raeder={[round(w, 1) for w in (r0.wheel_cmd or [])] if r0 else '-'} "
                  f"alt(kommando)={engine.t - (r0.t_cmd if r0 else 0):.2f}s "
                  f"pose=({r0.pose.x:.2f},{r0.pose.y:.2f}) weg={r0.distance:.2f} "
                  f"ber={r0.contacts} odom=({r0.odom.x if r0.odom else 0:.2f},"
                  f"{r0.odom.y if r0.odom else 0:.2f})" if r0 else "")
        time.sleep(pause)
        if time.monotonic() - start > args.wanduhr_max:
            print(f"Abbruch nach {args.wanduhr_max:.0f} s Wanduhr — der Auftrag läuft vermutlich "
                  f"in eine Endlosschleife.")
            break
    fertig.set()

    bericht = bew.report()
    print(f"\n{grade.format_report(bericht)}\n")
    print(f"Simulationszeit {bew.t:.1f} s in {time.monotonic() - start:.1f} s Wanduhr · "
          f"Welt {welt_name} · Seed {args.seed} · Controller {args.controller}")
    if args.json:
        with open(args.json, "w") as fh:
            json.dump(bericht, fh, indent=1)
    return 0 if bericht["bestanden"] else 2


if __name__ == "__main__":
    sys.exit(main())
