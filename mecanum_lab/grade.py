"""Grading for both experiments: a state machine that the simulation tick drives.

The grader is deliberately neither its own process nor a ROS node: the simulation run
(node.py) calls `tick(dt)`, everything else runs over topics. That makes the grader in a
stub run (one process, no ROS) exactly as blind or as sharp as later in ROS — it sees
odometry, GPS, LIDAR, IMU, /sim/robots and /sim/task, but never the robot's world from the
inside. Grading here means: give commands and measure behaviour.

Experiment 1 (`art` missing): the students drive, the grader measures path changes.
Experiment 2 (`art: "kf"`): the **grader** drives the command sequence written in the task
(pass-through mode), the students only estimate and report `kf/pose`. Measured: RMSE
against `truth`, improvement over the raw sensor, maximum error during the GPS outage and
the consistency of the stated standard deviation (NEES).

Use in node.py:
    g = Grader("alice", "alle", bus, tasks.load_tasks()).start()
    while not g.tick(dt): engine.step(dt); bus-publish(engine.drain())
    print(format_report(g.report()))
"""
import argparse
import json
import logging
import math

from . import robot_io
from . import tasks as T
from .types import Gps, Odom, Twist, topic, wrap_angle

log = logging.getLogger("mecanum.grade")


class Grader:
    """One task after another, split into steps; each step measures and reaches a verdict."""

    def __init__(self, robot: str, ids, bus, cfg_tasks: dict, world_info: dict | None = None):
        self.name = robot
        self.bus = bus
        self.aufgaben = T.resolve(cfg_tasks, ids)
        self.welt = world_info or {}
        self.t = 0.0
        self.plan, self.i, self.fertig = [], 0, False
        self.erg = {}
        self._step_t, self._marke, self._saw_running, self._abstand = 0.0, None, False, None
        self._mission_marke = None      # baseline at task START, not at the end
        self._grund = ""
        self._cmd = self.bus.pub("twist", self.name)
        self._serie = []                                 # (t, e_kf, e_gps, nees) per sample
        self._kf_seen = 0                                # kf/pose messages that arrived
        self._kf_letzte, self._kf_stempel = None, None   # of those each one counts once
        self._luecke, self._gps_stempel, self._info_start = [], None, {}
        self._probe_t = -1e9
        bus.sub("kf", self.name, lambda msg, *_: self._kf_saehen(msg))

    def _kf_saehen(self, msg) -> None:
        """Counts real messages only — the bus calls a callback again on every spin().

        Without this check the measured rate would be the grader's poll rate, and a node that
        sends nothing at all would reach 38 Hz.
        """
        stempel = getattr(msg, "t", None)
        if msg is self._kf_letzte or (stempel is not None and stempel == self._kf_stempel):
            return
        self._kf_letzte, self._kf_stempel, self._kf_seen = msg, stempel, self._kf_seen + 1

    # ------------------------------------------------------------------ state handling

    def start(self):
        self.plan, self.i, self.t, self.fertig, self.erg = self._plan(), 0, 0.0, False, {}
        self._grund, self._saw_running, self._abstand, self._mission_marke = "", False, None, None
        for a in self.aufgaben:
            self.erg[a["id"]] = {"id": a["id"], "titel": a["titel"], "max_punkte": a["punkte"],
                                 "punkte": 0.0, "bestanden": False, "begruendung": "not evaluated",
                                 "messwerte": {}, "phasen": {}}
        if self.plan:
            self._oeffne(self.plan[0])
        else:
            self.fertig = True
        log.info("grading started for '%s': %s", self.name,
                 ", ".join(a["id"] for a in self.aufgaben) or "none")
        return self

    def _plan(self) -> list:
        plan = []
        for a in self.aufgaben:
            if a["id"] == "kinematik":
                plan.append({"a": a, "art": "anlauf", "dauer": 1.0})
                for ph in a["phasen"]:
                    plan.append({"a": a, "art": "phase", "dauer": float(ph["dauer"]), "ph": ph})
                    plan.append({"a": a, "art": "pause", "dauer": float(a.get("haltezeit", 1.0))})
            elif a.get("art") == "kf":                   # experiment 2: the GRADER drives
                plan.append({"a": a, "art": "mission_start", "dauer": 1.5})
                plan.append({"a": a, "art": "kf_fahrt", "dauer": float(a["timeout"])})
                plan.append({"a": a, "art": "kf_ende", "dauer": 0.3})
            else:
                plan.append({"a": a, "art": "mission_start", "dauer": 0.5})
                plan.append({"a": a, "art": "mission", "dauer": float(a["timeout"])})
                plan.append({"a": a, "art": "mission_ende", "dauer": 0.5})
        return plan

    def tick(self, dt: float) -> bool:
        """Advance one simulation step; True when the grading is done."""
        if self.fertig:
            return True
        self.t += dt
        schritt = self.plan[self.i]
        self._step_t += dt
        self._wirke(schritt, dt)
        if (self._step_t >= schritt["dauer"] or self._mission_vorbei(schritt)
                or self._fahrt_vorbei(schritt)):
            self._schliessen(schritt)
            self.i += 1
            if self.i >= len(self.plan):
                self.fertig = True
                self._cmd(Twist())
            else:
                self._oeffne(self.plan[self.i])
        return self.fertig

    def _oeffne(self, s) -> None:
        self._step_t, self._abstand = 0.0, None
        self._marke = {"odom": self.bus.last("odom", self.name)[0],
                       "gps": self.bus.last("gps", self.name)[0],
                       "kf": self.bus.last("kf", self.name)[0],
                       "robot": robot_io.robot_info(self.bus, self.name), "t": self.t}
        if s["art"] == "mission_start":
            self._saw_running, self._grund = False, ""   # the previous task's reason is none
            self.bus.publish(topic("task"), s["a"]["id"])      # students switch over
            self.bus.publish(topic("mission", self.name), "")  # invalidate the old "done"
        if s["art"] == "mission":
            self._mission_marke = dict(self._marke)   # this baseline holds until the end
        if s["art"] == "kf_fahrt":
            self._serie, self._kf_seen, self._probe_t = [], 0, -1e9
            self._kf_letzte, self._kf_stempel = None, None
            self._luecke, self._gps_stempel = [], None
            self._info_start = dict(self._marke["robot"] or {})

    def _wirke(self, s, dt: float) -> None:
        """Send commands only during the kinematics phases — otherwise leave the students alone.

        Exception experiment 2 (`kf_fahrt`): here the grader drives the run itself
        (pass-through mode) and collects measurement pairs truth ↔ estimate in parallel.
        """
        art = s["art"]
        if art == "phase":
            ph = s["ph"]
            self._cmd(Twist(vx=ph["vx"], vy=ph["vy"], omega=ph["omega"]))
        elif art in ("anlauf", "pause"):
            self._cmd(Twist())
        elif art == "kf_fahrt":
            self._cmd(kommando_fahrt(s["a"], self._step_t))
            self._probe(s)
        elif art == "mission":
            seit = T.seitlicher_abstand(self.bus.last("scan", self.name)[0])
            odom = self.bus.last("odom", self.name)[0]
            if seit is not None and odom is not None and abs(odom.omega) <= s["a"].get(
                    "geradeaus_omega_max", 0.25) and abs(odom.vx) > 0.05:
                self._abstand = seit if self._abstand is None else min(self._abstand, seit)

    def _mission_vorbei(self, s) -> bool:
        if s["art"] != "mission":
            return False
        state = robot_io.robot_info(self.bus, self.name).get("mission", "") or \
            str(self.bus.last("mission", self.name)[0] or "")
        if state.startswith("running"):
            self._saw_running = True
        if state.startswith("failed"):
            self._grund = state
            return True
        if state == "done":
            if not self._saw_running and self._step_t < 3.0:
                return False                             # "done" may still come from the last task
            return True
        return False

    def _fahrt_vorbei(self, s) -> bool:
        """The KF run is over once the command sequence has been driven (plus 1 s of quiet)."""
        return (s["art"] == "kf_fahrt"
                and self._step_t >= fahrt_dauer(s["a"]) + 1.0)

    def _probe(self, s) -> None:
        """One measurement pair: last truth, last estimate, last raw sensor — into the series.

        The GPS outage is logged on the side: if the raw sensor stamp stays the same, no new
        measurement arrived — which is what happens at an arc edge of the arena.
        """
        a = s["a"]
        wenn = self._step_t
        if wenn < float(a.get("einlauf", 3.0)) or wenn - self._probe_t < 0.04:
            return
        wahr = self.bus.last("truth", self.name)[0]
        if wahr is None:
            return
        self._probe_t = wenn
        schatzung = self.bus.last("kf", self.name)[0]
        roh = self.bus.last(a.get("sensor", "gps"), self.name)[0]
        e_kf = math.hypot(schatzung.x - wahr.x, schatzung.y - wahr.y) if schatzung else 1e9
        e_roh = math.hypot(roh.x - wahr.x, roh.y - wahr.y) if roh else 1e9
        nees = float("nan")
        if schatzung and schatzung.sx > 1e-6 and schatzung.sy > 1e-6:
            nees = (((schatzung.x - wahr.x) ** 2 / schatzung.sx ** 2
                     + (schatzung.y - wahr.y) ** 2 / schatzung.sy ** 2) / 2.0)
        self._serie.append((wenn, e_kf, e_roh, nees))
        self._funkloch(wenn, wahr, roh, e_kf)

    def _funkloch(self, wenn: float, wahr, roh, fehler: float) -> None:
        """No fresh raw sensor fix? Then the estimate runs on reserves — we measure that.

        The comparison runs over the **message stamps** (simulation time), not over the wall
        clock: tools/fastgrade.py runs 25 simulation seconds per second, and a wall-clock
        test would miss a GPS outage that everyone else can see.
        """
        if roh is None:
            self._luecke.append((wenn, fehler))
            return
        als_zeit = getattr(wahr, "t", None)
        stempel = getattr(roh, "t", None)
        if als_zeit is None or stempel is None:
            return
        if als_zeit - stempel > 2.0:
            self._luecke.append((wenn, fehler))

    # ------------------------------------------------------------------------ evaluation

    def _schliessen(self, s) -> None:
        if s["art"] == "phase":
            self._wertePhase(s)
        elif s["art"] == "mission_ende":
            self._werteMission(s)
        elif s["art"] == "kf_ende":
            self._werteKf(s)

    def _wertePhase(self, s) -> None:
        ph, now = s["ph"], self.bus.last("odom", self.name)[0]
        ergebnis, start = self.erg[s["a"]["id"]], self._marke["odom"]
        if now is None or start is None:
            phasen_ok, mess, gruende = False, {}, "no odometry received"
        else:
            mess = _delta(start, now)
            gruende = _pruefe(ph.get("erwarte", {}), mess)
            phasen_ok = not gruende
        ergebnis["phasen"][ph["id"]] = {"ok": phasen_ok, "messwerte":
                                        {k: round(v, 3) for k, v in mess.items()},
                                        "begruendung": "; ".join(gruende)}
        je = s["a"]["punkte"] / len(s["a"]["phasen"])
        ergebnis["punkte"] = round(ergebnis["punkte"] + (je if phasen_ok else 0.0), 1)
        log.info("phase %-6s %s  %s", ph["id"], "ok" if phasen_ok else "FAIL", mess)

    def _werteMission(self, s) -> None:
        a, m = s["a"], self.erg[s["a"]["id"]]
        now_robot = robot_io.robot_info(self.bus, self.name)
        messung = a.get("messung", "odom")
        basis = self._mission_marke or self._marke      # NOT _marke: that one gets a new
        start = basis[messung]                          # value in the mission_ende step
        now = self.bus.last(messung, self.name)[0]
        info_start = (self._mission_marke or self._marke)["robot"] or {}
        grund = [] if not self._grund else [f"student reports: {self._grund}"]
        mess = {}
        if now is None or start is None:
            grund.append(f"no {messung.upper()} data")
        else:
            mess["zeit"] = round(self.t - basis["t"], 1)
            mess["weg"] = round(float(now_robot.get("distance", 0)) - float(info_start.get("distance", 0)), 2)
            mess["kontakte"] = int(now_robot.get("contacts", 0)) - int(info_start.get("contacts", 0))
            if self._abstand is not None:
                mess["seitlicher_abstand"] = round(self._abstand, 3)
            if a.get("ziel") == "spawn" or a["id"] == "quadrat":
                pass
            if a["id"] == "quadrat":
                mess["abschluss"] = round(T.pos_fehler(start, now), 3)
                mess["winkel_deg"] = round(math.degrees(T.winkel_fehler(start, now)), 1)
                if mess["abschluss"] > a["abschluss_max"]:
                    grund.append(f"completion error {mess['abschluss']} m > {a['abschluss_max']}")
                if mess["winkel_deg"] > a["winkel_max_deg"]:
                    grund.append(f"heading error {mess['winkel_deg']}° > {a['winkel_max_deg']}°")
            else:
                ziel = self._ziel(a, now_robot)
                if ziel is None:
                    grund.append("world goal unknown (is the world topic empty?)")
                else:
                    mess["zielfehler"] = round(T.pos_fehler(ziel, now), 3)
                    mess["ziel"] = [round(v, 2) for v in ziel[:2]]
                    if mess["zielfehler"] > a["ziel_max"]:
                        grund.append(f"target missed: {mess['zielfehler']} m > {a['ziel_max']} m")
            if mess.get("weg", 0) < a.get("weg_min", 0):
                grund.append(f"path {mess['weg']} m below {a['weg_min']} m — barely moved?")
            if mess.get("weg", 0) > a.get("weg_max", 1e9):
                grund.append(f"path {mess['weg']} m above {a['weg_max']} m — a detour?")
            if mess.get("kontakte", 0) > a.get("kontakte_max", 0):
                grund.append(f"{mess['kontakte']} wall contacts (allowed {a['kontakte_max']})")
            if self._abstand is not None and a.get("abstand_min") and self._abstand < a["abstand_min"]:
                grund.append(f"lateral {round(self._abstand, 2)} m below {a['abstand_min']} m")
            if mess.get("zeit", 0) > a["timeout"]:
                grund.append(f"time {mess['zeit']} s above limit {a['timeout']} s")
        m["messwerte"], m["begruendung"] = mess, ("; ".join(grund) or "meets requirements")
        m["bestanden"] = not grund
        m["punkte"] = a["punkte"] if m["bestanden"] else 0.0

    def _ziel(self, a: dict, robot_info: dict):
        """World goal: T3 the gate (goal), T4 a spawns entry — deliberately another target."""
        welt = self.welt or json.loads(self.bus.last("world")[0] or "{}")
        if a.get("ziel") == "spawn":
            starts = welt.get("spawns") or []
            if not starts:
                return None
            index = a.get("ziel_index")
            if index is None:
                return starts[min(int(robot_info.get("index", 0)), len(starts) - 1)]
            return starts[int(index) % len(starts)]
        return welt.get("goal")

    def _luecke_dauer(self) -> float:
        """How long no fresh raw sensor fix arrived during the run (in s)."""
        if not self._luecke:
            return 0.0
        probe = (self._serie[-1][0] - self._serie[0][0]) / max(len(self._serie) - 1, 1)
        return len(self._luecke) * probe

    def _werteKf(self, s) -> None:
        """Experiment 2: RMSE, improvement, maximum error, consistency — all against `truth`."""
        a, m = s["a"], self.erg[s["a"]["id"]]
        serie, grund, mess = self._serie, [], {}
        info = robot_io.robot_info(self.bus, self.name) or {}
        if self._grund:
            grund.append(f"student reports: {self._grund}")
        if len(serie) < 10:
            mess["stichproben"] = len(serie)
            grund.append("no measurement pairs — is your node publishing /<robot>/kf/pose? Is "
                         "the sim publishing truth on /<robot>/truth? (automatic under ./lab "
                         "grade, otherwise pass --truth)")
        else:
            dauer = max(serie[-1][0] - serie[0][0], 1.0)
            quad = lambda i: math.sqrt(sum(x[i] ** 2 for x in serie) / len(serie))  # noqa: E731
            rmse, rmse_roh = quad(1), quad(2)
            mess["stichproben"] = len(serie)
            mess["zeit"] = round(dauer, 1)
            mess["rmse"] = round(min(rmse, 999.0), 3)     # 1e9 = never an estimate: say so readably
            mess[f"rmse_{a.get('sensor', 'gps')}"] = round(rmse_roh, 3)
            mess["verbesserung"] = round(rmse_roh / rmse, 2) if rmse > 1e-9 else 0.0
            mess["max_fehler"] = round(min(max(x[1] for x in serie), 999.0), 3)
            mess["rate_hz"] = round(self._kf_seen / dauer, 1)
            mess["kontakte"] = int(info.get("contacts", 0)) - int(self._info_start.get("contacts", 0))
            nees = [x[3] for x in serie if x[3] == x[3]]
            if nees:
                mess["nees"] = round(sum(nees) / len(nees), 2)
                mess["nees_deckel"] = round(sum(1 for v in nees if v > 5.99) / len(nees), 3)
            else:
                mess["nees"] = None
                if not self._kf_seen:
                    grund.append("no kf/pose message received at all — is your node running "
                                 "under the same robot name, and does it call rob.send_kf(x, y, "
                                 "theta, sx, sy, sth)?")
                else:
                    grund.append("no standard deviations in kf/pose (sx/sy are mandatory)")
            grund += _pruefe_kf(a, mess)
            luecke = a.get("luecke")
            if luecke:
                dauer = self._luecke_dauer()
                mess["luecke_dauer"] = round(dauer, 1)
                mess["luecke_max"] = (round(max(e for _, e in self._luecke), 3)
                                      if self._luecke else None)
                if dauer < float(luecke.get("dauer_min", 1.0)):
                    grund.append(f"no GPS outage measurable ({dauer:.1f} s without a fix, "
                                 f"expected from {luecke['dauer_min']} s) — profile not driven?")
                elif mess["luecke_max"] > float(luecke["fehler_max"]):
                    grund.append(f"during the GPS outage {mess['luecke_max']} m > "
                                 f"{luecke['fehler_max']} m")
            grund += _profil_pruefen(a, self.bus)
        m["messwerte"], m["begruendung"] = mess, ("; ".join(grund) or "meets requirements")
        m["bestanden"] = not grund
        m["punkte"] = a["punkte"] if m["bestanden"] else 0.0
        log.info("KF %-14s %s  rmse=%s verb=%s", a["id"], "ok" if m["bestanden"] else "FAIL",
                 mess.get("rmse"), mess.get("verbesserung"))

    # --------------------------------------------------------------------------- result

    def report(self) -> dict:
        ein = list(self.erg.values())
        for e in ein:
            if e["phasen"]:
                ok = all(p["ok"] for p in e["phasen"].values())
                e["bestanden"] = ok and e["punkte"] >= e["max_punkte"] - 0.05
                e["begruendung"] = "; ".join(
                    f'{pid}: {p["begruendung"]}' for pid, p in e["phasen"].items() if not p["ok"]
                ) or f"all {len(e['phasen'])} phases meet requirements"
        return {"robot": self.name, "zeit": round(self.t, 1), "tasks": ein,
                "punkte": round(sum(e["punkte"] for e in ein), 1),
                "max_punkte": sum(e["max_punkte"] for e in ein),
                "bestanden": bool(ein) and all(e["bestanden"] for e in ein)}


def fahrt_segmente(a: dict) -> list:
    """Command segments of a grading run (experiment 2) — empty if the task drives freely."""
    return a.get("fahrt") or []


def fahrt_dauer(a: dict) -> float:
    """How long the command sequence takes; without segments: timeout minus 1 s of quiet time."""
    seg = fahrt_segmente(a)
    if not seg:
        return max(float(a.get("timeout", 30.0)) - 1.0, 1.0)
    return sum(float(s["dauer"]) for s in seg) * max(int(a.get("wiederhole", 1)), 1)


def kommando_fahrt(a: dict, t: float) -> Twist:
    """Command at run time `t`: segments with a base value, optionally a superposed sine.

    `sinus` superposes `sinus·sin(2π·frequenz·t)` on the yaw rate — that turns a straight
    line into a driven curve, without the task text containing secret waypoints. After the
    last segment the robot holds still.
    """
    seg = fahrt_segmente(a)
    if not seg:
        return Twist(float(a.get("vx", 0.3)), 0.0, float(a.get("omega", 0.0)))
    zyklus = sum(float(s["dauer"]) for s in seg)
    rest = t % zyklus if zyklus > 0 else 0.0
    for segment in seg:
        dauer = float(segment["dauer"])
        if rest >= dauer:
            rest -= dauer
            continue
        omega = float(segment.get("omega", 0.0))
        if segment.get("sinus"):
            omega += float(segment["sinus"]) * math.sin(
                2 * math.pi * float(segment.get("frequenz", 0.1)) * rest)
        return Twist(float(segment.get("vx", 0.0)), float(segment.get("vy", 0.0)), omega)
    return Twist()


def _pruefe_kf(a: dict, mess: dict) -> list:
    """Apply the thresholds of a KF task — the message always names the measured number."""
    grund = []
    pruefe = [("rmse", "rmse_max", "accuracy"), ("max_fehler", "max_fehler_max", "max error"),
              ("verbesserung", "verbesserung_min", "improvement over raw sensor"),
              ("rate_hz", "rate_min", "rate of kf/pose"),
              ("kontakte", "kontakte_max", "wall contacts")]
    for wert, schluessel, name in pruefe:
        grenze = a.get(schluessel)
        if grenze is None or mess.get(wert) is None:
            continue
        zu_wenig = schluessel.endswith("_min") and mess[wert] < grenze
        zu_viel = schluessel.endswith("_max") and mess[wert] > grenze
        if zu_wenig or zu_viel:
            grund.append(f"{name} {mess[wert]} violates {schluessel}={grenze}")
    if a.get("nees") and mess.get("nees") is not None:
        lo, hi = [float(v) for v in a["nees"]]
        if not lo <= mess["nees"] <= hi:
            grund.append(f"NEES {mess['nees']} outside [{lo}, {hi}] — the stated standard "
                         "deviation does not match the actual error")
    return grund


def _profil_pruefen(a: dict, bus) -> list:
    """Was the run really driven against the task's test profile? (otherwise the numbers are moot)"""
    soll, fahr = a.get("sim") or {}, {}
    try:
        fahr = json.loads(bus.last("config")[0] or "{}")
    except (ValueError, TypeError):
        return []
    grund = []
    for bereich in ("gps", "imu", "odom"):
        for schluessel, wert in (soll.get(bereich) or {}).items():
            hier = (fahr.get(bereich) or {}).get(schluessel)
            if not isinstance(wert, (int, float)) or not isinstance(hier, (int, float)) \
                    or abs(wert) < 1e-12:
                continue
            verhaeltnis = hier / wert
            if verhaeltnis < 0.67 or verhaeltnis > 1.5:
                grund.append(f"test profile not driven: {bereich}.{schluessel} = {hier} "
                             f"instead of {wert} (start with ./lab grade … or kf.launch.py)")
    return grund


def _delta(start, now) -> dict:
    """Path change in the start body frame: independent of how the robot began."""
    dx, dy = now.x - start.x, now.y - start.y
    c, s = math.cos(start.theta), math.sin(start.theta)
    return {"dx": c * dx + s * dy, "dy": -s * dx + c * dy,
            "winkel": wrap_angle(now.theta - start.theta)}


def _pruefe(soll: dict, mess: dict) -> list:
    """Apply the thresholds from tasks.json: dx_min, dy_betrag_max, winkel_min, …"""
    gruende = []
    for schluessel, grenze in soll.items():
        art, richtung = schluessel.split("_", 1)
        wert = mess.get(art)
        if wert is None:
            continue
        fehler = (wert < grenze) if richtung == "min" else (abs(wert) > grenze)
        if fehler:
            gruende.append(f"{art}={wert:+.2f} violates {schluessel}={grenze}")
    return gruende


def format_report(rep: dict) -> str:
    """Text table for the console — the same view the students get."""
    zeilen = [f"grading robot '{rep['robot']}'  ({rep['zeit']} s)",
              "-" * 66]
    for e in rep["tasks"]:
        zeilen.append(f"{'PASS' if e['bestanden'] else 'FAIL'}  {e['punkte']:5.1f}/"
                      f"{e['max_punkte']:3d} pts  {e['titel']}")
        for pid, p in e["phasen"].items():
            zeilen.append(f"   {'ok ' if p['ok'] else 'FAIL'}  {pid:7s} "
                          f"{p['messwerte']} {'' if p['ok'] else p['begruendung']}")
        if e["messwerte"]:
            zeilen.append(f"        measured: {e['messwerte']}")
        zeilen.append(f"        verdict: {e['begruendung']}")
    zeilen += ["-" * 66,
               f"Points reached: {rep['punkte']} / {rep['max_punkte']}"
               f"  ->  {'all tasks meet requirements' if rep['bestanden'] else 'needs rework'}"]
    return "\n".join(zeilen)


class FakeRoboter:
    """Helper object for tests and the self test: a robot that drives commands ideally.

    `vy_hebel = -1` simulates the classic sign error (lateral to the right instead of the
    left) — that is exactly how T1 has to fail. It drives missions as a straight-leg waypoint
    list, without a controller; it therefore shows the *result* of a solution, not its style.
    """

    def __init__(self, bus, name: str = "alice", vy_hebel: float = 1.0, welt: dict | None = None):
        self.bus, self.name, self.vy_hebel = bus, name, float(vy_hebel)
        self.welt = welt or {"name": "fake", "size": [12, 9], "goal": [4.0, 3.0, 0.0],
                             "spawns": [[1.0, 1.0, 0.0], [5.0, 5.0, 0.0]], "walls": 12}
        self.x, self.y, self.th = (*self.welt["spawns"][0][:2], 0.0)
        self.vx = self.vy = self.om = 0.0
        self.distance, self.contacts = 0.0, 0
        self.mode, self.mission, self.task = "wheels", "idle", ""
        self.start, self.weg = (self.x, self.y, self.th), []
        self.befehl = Twist()
        bus.sub_topic(topic("twist", name), lambda t: setattr(self, "befehl", t))
        bus.sub_topic(topic("task"), self._auftrag)
        self.melde()

    def _auftrag(self, name) -> None:
        name = str(name or "")
        if name == self.task:
            return                                         # the task was only repeated
        self.task = name
        if name in ("", "kinematik"):
            self.mission, self.weg, self.befehl = "idle", [], Twist()
        else:
            self.start, self.weg, self.mission = (self.x, self.y, self.th), self._wege(name), "running"

    def _wege(self, auftrag: str) -> list:
        """Legs of a task — a square from the start pose, otherwise the world goal."""
        if auftrag == "quadrat":
            x, y, th = self.start
            c, s = math.cos(th), math.sin(th)
            rand = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
            return [(x + a * c - b * s, y + a * s + b * c) for a, b in rand]
        if auftrag == "gps_anfahrt":
            return [self.welt["spawns"][-1][:2]]          # loading bay = last spawns entry
        return [self.welt["goal"][:2]] if self.welt.get("goal") else []

    def melde(self) -> None:
        self.bus.publish(topic("world"), json.dumps(self.welt))
        self.bus.publish(topic("robots"), json.dumps(
            [{"name": self.name, "index": 0, "mode": self.mode, "contacts": self.contacts,
              "distance": round(self.distance, 3), "mission": self.mission, "task": self.task}]))

    def tick(self, dt: float) -> None:
        if self.mission == "running":
            self._mission()
        vx, vy, om = self.befehl.vx, self.befehl.vy * self.vy_hebel, self.befehl.omega
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = (vx * c - vy * s) * dt, (vx * s + vy * c) * dt      # body into world frame
        self.x, self.y, self.th = self.x + dx, self.y + dy, self.th + om * dt
        self.vx, self.vy, self.om, self.befehl = vx, vy, om, Twist()
        self.distance += math.hypot(dx, dy)
        self.bus.publish(topic("odom", self.name), self._mess(Odom()))
        self.bus.publish(topic("gps", self.name), self._mess(Gps()))
        self.melde()

    def _mess(self, m):
        m.x, m.y, m.theta, m.vx, m.vy, m.omega = self.x, self.y, self.th, self.vx, self.vy, self.om
        return m

    def _mission(self, v: float = 0.4) -> None:
        """Drive one leg after the other — the reference solution does it with more detours."""
        while self.weg and math.hypot(self.weg[0][0] - self.x, self.weg[0][1] - self.y) < 0.06:
            self.weg.pop(0)
        if not self.weg:
            self.mission, self.befehl = "done", Twist()
            return
        dx, dy = self.weg[0][0] - self.x, self.weg[0][1] - self.y
        dist = math.hypot(dx, dy)
        self.befehl = Twist(vx=dx / dist * v, vy=dy / dist * v)


def main(argv=None) -> int:
    """Grading against a fake robot (no simulator) — the real run lives in node.py."""
    from . import stub
    ap = argparse.ArgumentParser(description="grader run with a fake robot (self test)")
    ap.add_argument("--robot", default="alice")
    ap.add_argument("--task", default="alle", help="task id or 'alle'")
    ap.add_argument("--json", help="write the report as JSON to this path")
    args = ap.parse_args(argv)
    fake = FakeRoboter(stub.get_bus(), args.robot)
    gr = Grader(args.robot, args.task, fake.bus, T.load_tasks()).start()
    while not gr.tick(0.02):
        fake.tick(0.02)
    rep = gr.report()
    print(format_report(rep))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, indent=1, ensure_ascii=False)
    return 0 if rep["bestanden"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
