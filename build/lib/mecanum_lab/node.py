"""Kommandozeile des Simulators — `./lab <befehl>` (CONTRACT §6.9, CONTRACT-KF §4).

Ein einziger Simulationslauf-Kern (`simlauf`) steckt hinter allen Varianten: mit und
ohne Fenster, mit und ohne ROS, mit Studierendenknoten und mit Bewerter. Der Unterschied
ist nur der Bus: `run` benutzt den In-Prozess-Bus (kein ROS nötig), `sim` nimmt echtes
ROS, wenn rclpy da ist.

    ./lab run  --robot alice --controller student/controller_template.py   # Schnelleinstieg
    ./lab sim  --world track --robots alice,bob                            # Simulator (ROS)
    ./lab grade --robot alice --task alle                                  # Bewerten
    ./lab run  --task kf_gps --controller student/kf_template.py --truth    # Versuch 2
    ./lab spawn --name bob ; ./lab robots ; ./lab docs

Einstellungen ohne JSON-Datei: `--set gps.sigma_xy=0.8 --set imu.rate=400` beliebig
oft; das Auswerte-Protokoll schreibt `--log messung.csv`.
"""
import argparse
import importlib.util
import json
import logging
import math
import os
import sys
import threading
import time

from . import render as R
from . import ros_bridge, robot_io, stub, tasks as T
from .engine import SpawnError, SimEngine
from .logbook import Logbuch
from .robot_io import RobotIO
from .types import MSG_SPECS, Twist, load_config, topic
from .worlds import list_worlds, load_world

log = logging.getLogger("mecanum.node")
WELTEN = ", ".join(list_worlds())


# --------------------------------------------------------------------- Lauf-Kern


def simlauf(eng, bus, rend=None, graders=(), seconds=0.0, teleop=False, hz=60.0, tap=None):
    """Ein Takt: Simulation einen Schritt, Messungen auf den Bus, Fenster, Bewerter, Protokoll."""
    pubs, letzte, t_clock, t_json = {}, time.monotonic(), 0.0, 0.0
    letzte_schaetzung = {}                      # damit das Protokoll jede kf/pose-Meldung nur einmal zaehlt
    letzte_robots, t_robots = None, 0.0
    send_task = bus.pub("task")
    while bus.ok() and (rend is None or rend.ok) and (not seconds or eng.t < seconds):
        jetzt = time.monotonic()
        dt = min(jetzt - letzte, 0.25)
        letzte = jetzt
        if rend is None or not rend.paused:
            eng.step(dt)
        for kind, robot, payload in eng.drain():          # Messungen -> Themen
            if tap:
                tap.tap(kind, robot, payload)
            pubs.setdefault((kind, robot), bus.pub(kind, robot))(payload)
        # Roboterliste (mission_state, distance, contacts): der Bewerter haengt daran, und ein
        # spaet angemeldeter ROS-Client sieht ohnehin nur, was danach gesendet wird — die Engine
        # stoesst die Liste nur bei spawn/reset an. Also hier: sofort bei Aenderung (sonst
        # wartet der Bewerter eine Sekunde auf das Mission-Ende), aber mindestens
        # einmal pro Sekunde — sonst haengt `ros2 topic echo /sim/robots --once` im Leeren.
        robots = json.dumps(eng.robots_info())
        if robots != letzte_robots or jetzt - t_robots > 1.0:
            pubs.setdefault(("robots", None), bus.pub("robots"))(robots)
            letzte_robots, t_robots = robots, jetzt
        if tap:                                   # kf/pose kommt von den Studierenden, nicht von der Sim
            for name in eng.robots:
                schatzung, _ = bus.last("kf", name)
                if schatzung is not None and schatzung is not letzte_schaetzung.get(name):
                    letzte_schaetzung[name] = schatzung
                    tap.tap("kf", name, schatzung)
            tap.tick()
        if jetzt - t_clock > 0.05:                        # /clock für use_sim_time
            bus.pub("clock")(eng.t)
            t_clock = jetzt
        if jetzt - t_json > 1.0:                          # Weltinfo, Roboterliste, Auftrag, Profil
            bus.pub("world")(eng.world_json())
            bus.pub("config")(eng.config_json())
            send_task(eng.task)
            t_json = jetzt
        if teleop and rend is not None:
            tw = tasten()
            if tw and tw != (0, 0, 0):
                for name in eng.robots:
                    pubs.setdefault(("twist", name), bus.pub("twist", name))(Twist(*tw))
        if rend is not None:
            if rend.poll()["quit"]:
                break
            rend.draw()
        for g in graders:
            g.tick(dt)
        if graders and all(getattr(g, "fertig", False) for g in graders):
            break                                       # Bewerter ist durch -> Lauf beenden
        bus.spin(1.0 / hz if rend is None else 0.002)
    return eng


def tasten() -> tuple:
    """Pfeiltasten in Körpergeschwindigkeit: hoch/vor, runter/zurück, links/reits, q/e Gier."""
    import pygame
    k = pygame.key.get_pressed()
    return (0.35 * (k[pygame.K_UP] - k[pygame.K_DOWN]), 0.35 * (k[pygame.K_LEFT] - k[pygame.K_RIGHT]),
            0.9 * (k[pygame.K_e] - k[pygame.K_q]))


def parse_set(text: str) -> tuple:
    """`--set gps.sigma_xy=0.8` -> ("gps", {"sigma_xy": 0.8}); Wert wird als JSON geparst.

    Zahlen, boolsche Werte und Listen gehen direkt (`imu.rate=400`, `debug_truth=true`,
    `gps.bias_xy=[0.4,-0.2]`), sonst bleibt der Rest ein String. Zwei Punkte im Pfad
    erlauben beliebig tiefe Einstellungen — damit ist das Launch-File der ganze Wunschzettel.
    """
    pfad, gleich, wert = str(text).partition("=")
    if not gleich or not pfad.strip():
        raise ValueError(f"--set will 'pfad.unter.pfad=Wert', bekommen: '{text}'")
    try:
        wert = json.loads(wert.strip())
    except ValueError:
        pass
    stufen = pfad.strip().split(".")
    baum = {stufen[-1]: wert}
    for stufe in reversed(stufen[:-1]):
        baum = {stufe: baum}
    return stufen[0], baum


def sets_zusammenfassen(roh: list) -> dict:
    """Alle `--set`-Angaben zu einem Uberschreibungsbaum; zuletzt genannt gewinnt."""
    baum = {}
    for angabe in roh or []:
        schluessel, zweig = parse_set(angabe)
        _vertiefe(baum, schluessel, zweig[schluessel])
    return baum


def _vertiefe(baum: dict, schluessel: str, wert) -> None:
    ziel = baum.setdefault(schluessel, {})
    if not isinstance(wert, dict):
        baum[schluessel] = wert
        return
    if not isinstance(ziel, dict):
        baum[schluessel] = ziel = {}
    for teil, unter in wert.items():
        _vertiefe(ziel, teil, unter)


def mach_engine(args):
    """Config-Schichten: DEFAULT <- config/default.json <- --config <- Prüfprofil <- --set."""
    uberschreiben = {"world": args.world, "gui": not args.headless}
    if getattr(args, "truth", False):
        uberschreiben["debug_truth"] = True
    if args.task:
        try:
            uberschreiben.update(T.sim_profil(T.load_tasks(), args.task))
        except (ValueError, FileNotFoundError) as exc:
            log.warning("Prüfprofil nicht gelesen (%s) — gemessen wird, wie die Sim läuft.", exc)
    uberschreiben.update(sets_zusammenfassen(getattr(args, "set", None)))
    cfg = load_config(getattr(args, "config", None), uberschreiben)
    welt = cfg_get_welt(cfg, args, uberschreiben)
    eng = SimEngine(load_world(welt), cfg, seed=args.seed)
    # Was per --set angegeben wurde, darf kein Auftragsprofil mehr überbieten: die Engine
    # bekommt die Angaben als dauernde Übersteuerung mit.
    eng.erzwungen = {k: v for k, v in sets_zusammenfassen(getattr(args, "set", None)).items()
                     if k != "world"}
    for name in [r for r in (args.robots or "").split(",") if r.strip()]:
        try:
            eng.spawn(name.strip())
        except (SpawnError, ValueError) as exc:
            log.error("Roboter '%s': %s", name, exc)
    if args.task:
        eng.set_task(erster_auftrag(args.task))
    return eng


def erster_auftrag(kette: str) -> str:
    """Die --task-Angabe auf ihren ersten echten Auftrag reduzieren: "alle" -> "kinematik".

    Der Auftrag wandert auf /sim/task, und der Runner der Studierenden schaltet danach um.
    Eine Gruppe ist aber keine Aufgabe: "alle" als Auftrag gemeldet ergibt beim Knoten
    "unbekannter Auftrag" und eine Fehlermeldung, die niemand verschuldet hat.
    """
    try:
        auftrge = T.resolve(T.load_tasks(), kette)
    except (ValueError, FileNotFoundError):
        return kette
    return auftrge[0]["id"] if auftrge else kette


def cfg_get_welt(cfg: dict, args, profil: dict) -> str:
    """Welt wählen: Angesagtes zuerst, dann die Empfehlung der Aufgaben, dann der Standard.

    Die Aufgaben sagen ihre Halle über `"welt"` in config/tasks.json — Versuch 1 verlangt
    `production`, Versuch 2 `arena`. Diese Empfehlung zählt auch ohne `--world auto`, sonst
    bewertet man eine Quadratfahrt in der falschen Halle und wundert sich über Wände.
    `--world auto` ist dasselbe, nur explizit; ein genannter Name gewinnt immer.
    """
    if args.world not in (None, "", "auto"):
        return args.world
    standard = cfg.get("world", "maze")
    if not args.task:                                     # kein Auftrag angesagt -> keine Empfehlung
        return standard
    try:
        auftrge = T.resolve(T.load_tasks(), args.task)
    except (ValueError, FileNotFoundError):
        return standard
    if not auftrge:
        return standard
    kf = all(a.get("art") == "kf" for a in auftrge)      # Blindfahrt ohne Weltangabe: arena
    return T.welt_fuer({"tasks": auftrge}, [a["id"] for a in auftrge],
                       default="arena" if kf else standard)


def bus_fuer(args, node_name):
    """Stub, wenn --stub oder kein ROS; sonst der echte ROS-Bus."""
    bus = ros_bridge.make_bus("stub" if args.stub else "auto", node_name=node_name)
    if bus is None:
        bus = stub.get_bus()
        log.info("Kein ROS 2 aktiv — In-Prozess-Bus (läuft genauso, nur ohne ros2 topic).")
    return bus


def abo(bus, eng, name: str) -> None:
    """Die Kommando- und Schätzungs-Themen eines Roboters in die Simulation verbinden."""
    bus.sub("twist", name, lambda t, n=name: eng.set_cmd_vel(n, t))
    bus.sub("wheels", name, lambda w, n=name: eng.set_wheel_speeds(n, w))
    bus.sub("mission", name, lambda s, n=name: eng.set_mission(n, s))
    bus.sub("kf", name, lambda k, n=name: eng.set_kf(n, k))


def auftrag_profile() -> dict:
    """{Auftrags-id: Auftrag} — Prüfprofil und Fahrart je Aufgabe, nicht global."""
    try:
        return {a["id"]: a for a in T.load_tasks()["tasks"]}
    except Exception as exc:
        log.warning("Aufträge nicht lesbar (%s) — keine automatischen Prüfprofile.", exc)
        return {}


def verbinde_auftrag(bus, eng, profile: dict | None = None, robot: str | None = None) -> None:
    """/sim/task in die Simulation legen: Auftrag, Sensorprofil, Startpose.

    `robot` ist der Roboter, den ein Bewerter unter einer blinden Kommandofahrt hat: Bei
    einem KF-Auftrag wird er dafür an der Spawn-Pose abgesetzt. Die Kommandofolge wird nicht
    zurückgemeldet, also muss sie dort beginnen, wo die Welt den Roboter abstellt — sonst
    fährt sie in der zweiten Aufgabe gegen eine Wand, weil die erste irgendwo geendet hat.
    """
    profile = auftrag_profile() if profile is None else profile

    def neu(text) -> None:
        name = str(text or "")
        auftrag = profile.get(name) or {}
        if robot and auftrag.get("art") == "kf" and name != eng.task:
            eng.reset_robot(robot)
        eng.set_task(name)
        if auftrag.get("sim"):
            eng.set_sensor_profil(auftrag["sim"])
    bus.sub_topic(topic("task"), neu)


def fuege_knoten(datei, name, bus):
    """Studierendenknoten als Thread: Modul importieren, dann robot_io.serve() laufen lassen."""
    spec = importlib.util.spec_from_file_location("student_knoten", datei)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    def lauf():
        try:
            robot_io.serve(mod, name=name, bus=bus)
        except Exception:
            log.exception("Knoten %s ist gelaufen", datei)

    t = threading.Thread(target=lauf, daemon=True, name=f"knoten-{name}")
    t.start()
    log.info("Knoten %s für Roboter '%s' gestartet", os.path.basename(datei), name)
    return t


# --------------------------------------------------------------------- Befehle


def protokoll(args, eng):
    """CSV-Protokoll öffnen, wenn `--log` gegeben wurde — sonst None (nichts wird geschrieben)."""
    if not getattr(args, "log", None):
        return None
    try:
        return Logbuch(eng, args.log, getattr(args, "log_intervall", 0.05) or 0.05)
    except OSError as exc:
        log.error("Protokoll '%s' nicht anlegbar: %s", args.log, exc)
        return None


def cmd_run(args):
    """Simulator + Studierendenknoten in einem Prozess, ohne ROS — der Schnelleinstieg."""
    args.stub = True
    eng, bus = mach_engine(args), stub.get_bus()
    tap = protokoll(args, eng)
    if args.robot and args.robot not in eng.robots:
        # `--robot` ist der Name des eigenen Roboters — ohne ihn explizit zu spawnen wäre
        # die Halle leer, und der Studierende sähe nur eine leere Karte mit laufendem Knoten.
        try:
            eng.spawn(args.robot)
        except (SpawnError, ValueError) as exc:
            log.error("Roboter '%s': %s", args.robot, exc)
    graders = [_grader(args.robot, args.task, bus, eng)] if args.grade else []
    verbinde_auftrag(bus, eng, robot=args.robot if graders else None)
    knoten = [fuege_knoten(c, args.robot, bus) for c in (args.controller or [])]
    rend = None if args.headless else R.Renderer(eng, eng.cfg)
    simlauf(eng, bus, rend, graders, args.seconds, teleop=not args.no_teleop, tap=tap)
    if rend:
        rend.close()
    for k in knoten:
        k.join(timeout=0.5)
    if tap:
        tap.close()
    return _berichte(graders)


def _berichte(graders, json_pfad: str | None = None) -> int:
    """Bewertungsberichte ausgeben; 2 heisst: mindestens eine Aufgabe nicht bestanden."""
    from .grade import format_report
    ohne_bestehen = False
    for g in graders:
        rep = g.report()
        print(format_report(rep))
        ohne_bestehen = ohne_bestehen or not rep.get("bestanden")
        if json_pfad:
            with open(json_pfad, "w", encoding="utf-8") as fh:
                json.dump(rep, fh, indent=1, ensure_ascii=False)
    return 2 if (graders and ohne_bestehen) else 0


def _grader(robot, task, bus, eng):
    from .grade import Grader
    welt = json.loads(eng.world_json()) if eng else {}
    g = Grader(robot, task or "alle", bus, T.load_tasks(), welt)
    return g.start()


def cmd_sim(args):
    """Simulator solo — mit echtem ROS, wenn vorhanden (der Regelfall im Praktikumsraum)."""
    eng, bus = mach_engine(args), bus_fuer(args, "mecanum_sim")
    tap = protokoll(args, eng)
    for name in list(eng.robots):
        abo(bus, eng, name)

    def spawn(req):
        out = ros_bridge.spawn_handler(eng)(req)
        if out.get("success"):
            abo(bus, eng, str(req.get("name", "")))       #auch der neue Roboter ist hörbar
        return out

    bus.service(topic("spawn"), spawn)
    bus.service(topic("despawn"), ros_bridge.despawn_handler(eng))
    verbinde_auftrag(bus, eng, robot=args.robot if args.grade else None)
    bus.service(topic("reset"), lambda req: (eng.reset(), {"success": True,
                                                           "message": "Welt zurückgesetzt"})[1])
    graders = [_grader(args.robot, args.grade, bus, eng)] if args.grade else []
    rend = None if args.headless else R.Renderer(eng, eng.cfg)
    if not args.stub:
        log.info("Topics: %s/<cmd_vel,wheel_speeds,odom,scan,gps,imu,kf/pose>  "
                 "/sim/<robots,world,task,config>  "
                 "/sim/<spawn_robot,despawn_robot,reset>  /clock", "/<robot>")
    simlauf(eng, bus, rend, graders, args.seconds, teleop=not args.no_teleop, tap=tap)
    if rend:
        rend.close()
    bus.shutdown()
    if tap:
        tap.close()
    return _berichte(graders)


def cmd_controller(args):
    """Nur ein Studierendenknoten (der Simulator läuft in einem anderen Terminal)."""
    if not args.controller:
        log.error("--controller datei.py fehlt")
        return 2
    fuege_knoten(args.controller[0] if isinstance(args.controller, list) else args.controller,
                 args.robot, bus_fuer(args, f"knoten_{args.robot}")).join()
    return 0


def cmd_client(args, befehl):
    """spawn/despawn/reset/robots/task: kurz ein ROS-Problem, nicht mehr."""
    bus = ros_bridge.make_bus("auto", node_name=f"client_{befehl}")
    if bus is None:
        log.error("Kein ROS 2 aktiv. Im Stub-Lauf gehören diese Befehle in denselben Prozess: "
                  "./lab run --robots %s ...", args.name or "alice")
        return 2
    if befehl == "robots":
        for _ in range(60):
            bus.spin(0.02)
            payload, alter = bus.last("robots")
            if payload:
                for r in json.loads(payload):
                    print(f"{r['name']:14} {r['color']:8} {r['variant']:6} {r['mode']:12} "
                          f"pose=({r['pose'][0]:6.2f},{r['pose'][1]:6.2f}) "
                          f"weg={r['distance']:6.2f} m  anstösse={r['contacts']}  {r['mission']}")
                return 0
        log.error("Keine Antwort auf /sim/robots — läuft der Simulator?")
        return 2
    if befehl == "task":
        bus.pub("task")(args.task or args.name or "")
        print(f"Auftrag '{args.task or args.name}' gesendet.")
        bus.spin(0.1)
        return 0
    antwort = bus.call(topic(befehl), {"name": args.name or "", "variant": args.variant or ""})
    print(("ok   — " if antwort.get("success") else "fehler — ") + str(antwort.get("message", "")))
    if befehl == "spawn" and antwort.get("success"):
        print(f"     {antwort.get('color')} / {antwort.get('marker')} / {antwort.get('variant')}"
              f" an ({antwort.get('x', 0):.2f}, {antwort.get('y', 0):.2f})")
    bus.shutdown()
    return 0 if antwort.get("success") else 1


def cmd_grade(args):
    """Bewerten. Der Takt kommt aus dem Simulator dazu — hier im Stub-Lauf, ein Prozess."""
    args.stub = True
    eng, bus = mach_engine(args), stub.get_bus()
    tap = protokoll(args, eng)
    if args.robot not in eng.robots:
        eng.spawn(args.robot)
    abo(bus, eng, args.robot)                      # kf/pose muss in die Sim zurückkommen
    verbinde_auftrag(bus, eng, robot=args.robot)   # je Aufgabe ihr Prüfprofil, KF: Spawn-Pose
    knoten = [fuege_knoten(c, args.robot, bus)
              for c in (args.controller or []) if not c.endswith(".json")]
    g = _grader(args.robot, args.task or "alle", bus, eng)
    rend = None if args.headless else R.Renderer(eng, eng.cfg)
    simlauf(eng, bus, rend, [g], args.seconds, teleop=False, tap=tap)
    if rend:
        rend.close()
    if tap:
        tap.close()
    return _berichte([g], args.json)


def cmd_docs(args):
    print(f"Umgebungen: {WELTEN}\n")
    print("Themen pro Roboter:")
    for kind in ("twist", "wheels", "odom", "scan", "gps", "imu", "kf", "kfinfo", "mission"):
        print(f"  {topic(kind, 'alice'):24} {MSG_SPECS[kind][0]}")
    print("  /alice/truth (nur mit --truth bzw. debug_truth, dann mit truth.rate)")
    print("  /sim/robots /sim/world /sim/task /sim/config  (std_msgs/String, JSON)")
    print("  /sim/spawn_robot /sim/despawn_robot (mecanum_lab_interfaces/srv/SpawnRobot "
          "oder JSON-Handshake)  /sim/reset (std_srvs/srv/Trigger)  /clock")
    print("\nAufgaben: " + T.short_help(T.load_tasks()))
    print("Gruppen: --task alle | kf_alle | v1 | v2 | einzelner Auftrag")
    print("\nBeispiele Versuch 2 (Kalman-Filter):")
    print("  ./lab grade --task kf_alle --controller student/kf_solution.py --log messung.csv")
    print("  ./lab run --world arena --task kf_gps --robot alice \\")
    print("        --controller student/kf_template.py --truth --log messung.csv")
    print("  ros2 launch launch/kf.launch.py aufgabe:=kf_fusion headless:=true")
    return 0


# --------------------------------------------------------------------- Argumente


def parser():
    p = argparse.ArgumentParser(prog="lab", description="Kleiner Mecanum-Simulator (CONTRACT §6.9)")
    p.add_argument("--world", default=None,
                   help=f"Umgebung: {WELTEN}, 'auto' = Empfehlung der Aufgaben "
                        "(KF-Aufträge holen ihre Halle automatisch)")
    p.add_argument("--robots", default="", help="Komma-getrennte Roboternamen beim Start")
    p.add_argument("--robot", default="alice", help="Dein Robotername (Knoten, Bewerter)")
    p.add_argument("--controller", action="append", help="Studierendenknoten, mehrfach erlaubt")
    p.add_argument("--task", default="", help=f"Auftrag oder Gruppe: {', '.join(T.task_ids(T.load_tasks()))}, kf_alle, v1, v2")
    p.add_argument("--seconds", type=float, default=0.0, help="nach N s Simulationszeit enden")
    p.add_argument("--headless", action="store_true", help="ohne Pygame-Fenster")
    p.add_argument("--stub", action="store_true", help="In-Prozess-Bus statt ROS")
    p.add_argument("--no-teleop", action="store_true", help="Tastatursteuerung aus")
    p.add_argument("--seed", type=int, default=1, help="Rausch-Seed (Nachfahrbarkeit)")
    p.add_argument("--config", default=None, help="JSON-Config zusätzlich zu config/default.json")
    p.add_argument("--set", action="append", metavar="PFAD=WERT",
                   help="Einzelne Einstellung, z. B. --set gps.sigma_xy=0.8 (mehrfach)")
    p.add_argument("--truth", action="store_true",
                   help="exakte Pose auf /<robot>/truth veröffentlichen (Versuch 2)")
    p.add_argument("--log", default=None, metavar="DATEI.csv",
                   help="Messprotokoll (truth/gps/odom/kf/imu) als CSV schreiben")
    p.add_argument("--log-intervall", type=float, default=0.05,
                   help="Abstand der Protokollzeilen in s Simzeit (Standard 0,05)")
    p.add_argument("--grade", nargs="?", const="alle", default=None,
                   help="Bewerter mitschicken: Aufträge oder Gruppe für --robot (Standard: alle)")
    p.add_argument("--json", default=None, help="Bewertungsbericht als JSON")
    p.add_argument("--name", default="", help="Robotername für spawn/despawn")
    p.add_argument("--variant", default="", help="Motorvariante: stock|slow|fast|agile")
    return p


BEFEHLE = {"run": cmd_run, "sim": cmd_sim, "controller": cmd_controller, "grade": cmd_grade,
           "spawn": lambda a: cmd_client(a, "spawn"),
           "despawn": lambda a: cmd_client(a, "despawn"),
           "reset": lambda a: cmd_client(a, "reset"),
           "robots": lambda a: cmd_client(a, "robots"),
           "task": lambda a: cmd_client(a, "task"), "docs": cmd_docs}


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    befehl = argv[0] if argv and not argv[0].startswith("-") else "sim"
    if befehl in ("-h", "--help", "help"):
        parser().print_help()
        print("\nBefehle: " + ", ".join(sorted(BEFEHLE)))
        return 0
    args = parser().parse_args(argv[1:] if argv and argv[0] in BEFEHLE else argv)
    from . import setup_logging
    setup_logging()
    if befehl not in BEFEHLE:
        log.error("unbekannter Befehl '%s' — ./lab -h", befehl)
        return 2
    return BEFEHLE[befehl](args)


if __name__ == "__main__":
    raise SystemExit(main())
