"""Grading for both experiments: a state machine that the simulation tick drives.

The grader is deliberately neither its own process nor a ROS node: the simulation run
(node.py) calls `tick(dt)`, everything else runs over topics. That makes the grader in a
stub run (one process, no ROS) exactly as blind or as sharp as later in ROS — it sees
odometry, GPS, LIDAR, IMU, /sim/robots and /sim/task, but never the robot's world from the
inside. Grading here means: give commands and measure behaviour.

Experiment 1 (`kind` missing): the students drive, the grader measures path changes.
Experiment 2 (`kind: "kf"`): the **grader** drives the command sequence written in the task
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
        self.tasks = T.resolve(cfg_tasks, ids)
        self.world_info = world_info or {}
        self.t = 0.0
        self.plan, self.i, self.done = [], 0, False
        self.results = {}
        self._step_t, self._mark, self._saw_running, self._lateral = 0.0, None, False, None
        self._mission_mark = None      # baseline at task START, not at the end
        self._reason = ""
        self._cmd = self.bus.pub("twist", self.name)
        self._series = []                                 # (t, e_kf, e_gps, nees) per sample
        self._kf_seen = 0                                # kf/pose messages that arrived
        self._kf_last, self._kf_stamp = None, None   # of those each one counts once
        self._outage, self._gps_stamp, self._info_start = [], None, {}
        self._probe_t = -1e9
        bus.sub("kf", self.name, lambda msg, *_: self._kf_arrived(msg))

    def _kf_arrived(self, msg) -> None:
        """Counts real messages only — the bus calls a callback again on every spin().

        Without this check the measured rate would be the grader's poll rate, and a node that
        sends nothing at all would reach 38 Hz.
        """
        stamp = getattr(msg, "t", None)
        if msg is self._kf_last or (stamp is not None and stamp == self._kf_stamp):
            return
        self._kf_last, self._kf_stamp, self._kf_seen = msg, stamp, self._kf_seen + 1

    # ------------------------------------------------------------------ state handling

    def start(self):
        self.plan, self.i, self.t, self.done, self.results = self._plan(), 0, 0.0, False, {}
        self._reason, self._saw_running, self._lateral, self._mission_mark = "", False, None, None
        for a in self.tasks:
            self.results[a["id"]] = {"id": a["id"], "title": a["title"], "max_points": a["points"],
                                 "points": 0.0, "passed": False, "reason": "not evaluated",
                                 "measured": {}, "criteria": [], "phases": {}}
        if self.plan:
            self._open_step(self.plan[0])
        else:
            self.done = True
        log.info("grading started for '%s': %s", self.name,
                 ", ".join(a["id"] for a in self.tasks) or "none")
        return self

    def _plan(self) -> list:
        plan = []
        for a in self.tasks:
            if a["id"] == "kinematik":
                plan.append({"task": a, "kind": "ramp", "limit": 1.0})
                for ph in a["phases"]:
                    plan.append({"task": a, "kind": "phase", "limit": float(ph["duration"]), "ph": ph})
                    plan.append({"task": a, "kind": "pause", "limit": float(a.get("hold_time", 1.0))})
            elif a.get("kind") == "kf":                   # experiment 2: the GRADER drives
                plan.append({"task": a, "kind": "mission_start", "limit": 1.5})
                plan.append({"task": a, "kind": "kf_drive", "limit": float(a["timeout"])})
                plan.append({"task": a, "kind": "kf_eval", "limit": 0.3})
            else:
                plan.append({"task": a, "kind": "mission_start", "limit": 0.5})
                plan.append({"task": a, "kind": "mission", "limit": float(a["timeout"])})
                plan.append({"task": a, "kind": "mission_end", "limit": 0.5})
        return plan

    def tick(self, dt: float) -> bool:
        """Advance one simulation step; True when the grading is done."""
        if self.done:
            return True
        self.t += dt
        step = self.plan[self.i]
        self._step_t += dt
        self._act(step, dt)
        if (self._step_t >= step["limit"] or self._mission_over(step)
                or self._drive_over(step)):
            self._close_step(step)
            self.i += 1
            if self.i >= len(self.plan):
                self.done = True
                self._cmd(Twist())
            else:
                self._open_step(self.plan[self.i])
        return self.done

    def _open_step(self, s) -> None:
        self._step_t, self._lateral = 0.0, None
        self._mark = {"odom": self.bus.last("odom", self.name)[0],
                       "gps": self.bus.last("gps", self.name)[0],
                       "kf": self.bus.last("kf", self.name)[0],
                       "robot": robot_io.robot_info(self.bus, self.name), "t": self.t}
        if s["kind"] == "mission_start":
            self._saw_running, self._reason = False, ""   # the previous task's reason is none
            self.bus.publish(topic("task"), s["task"]["id"])      # students switch over
            self.bus.publish(topic("mission", self.name), "")  # invalidate the old "done"
        if s["kind"] == "mission":
            self._mission_mark = dict(self._mark)   # this baseline holds until the end
        if s["kind"] == "kf_drive":
            self._series, self._kf_seen, self._probe_t = [], 0, -1e9
            self._kf_last, self._kf_stamp = None, None
            self._outage, self._gps_stamp = [], None
            self._info_start = dict(self._mark["robot"] or {})

    def _act(self, s, dt: float) -> None:
        """Send commands only during the kinematics phases — otherwise leave the students alone.

        Exception experiment 2 (`kf_fahrt`): here the grader drives the run itself
        (pass-through mode) and collects measurement pairs truth ↔ estimate in parallel.
        """
        kind = s["kind"]
        if kind == "phase":
            ph = s["ph"]
            self._cmd(Twist(vx=ph["vx"], vy=ph["vy"], omega=ph["omega"]))
        elif kind in ("ramp", "pause"):
            self._cmd(Twist())
        elif kind == "kf_drive":
            self._cmd(drive_command(s["task"], self._step_t))
            self._probe(s)
        elif kind == "mission":
            seit = T.lateral_distance(self.bus.last("scan", self.name)[0])
            odom = self.bus.last("odom", self.name)[0]
            if seit is not None and odom is not None and abs(odom.omega) <= s["task"].get(
                    "straight_omega_max", 0.25) and abs(odom.vx) > 0.05:
                self._lateral = seit if self._lateral is None else min(self._lateral, seit)

    def _mission_over(self, s) -> bool:
        if s["kind"] != "mission":
            return False
        state = robot_io.robot_info(self.bus, self.name).get("mission", "") or \
            str(self.bus.last("mission", self.name)[0] or "")
        if state.startswith("running"):
            self._saw_running = True
        if state.startswith("failed"):
            self._reason = state
            return True
        if state == "done":
            if not self._saw_running and self._step_t < 3.0:
                return False                             # "done" may still come from the last task
            return True
        return False

    def _drive_over(self, s) -> bool:
        """The KF run is over once the command sequence has been driven (plus 1 s of quiet)."""
        return (s["kind"] == "kf_drive"
                and self._step_t >= drive_duration(s["task"]) + 1.0)

    def _probe(self, s) -> None:
        """One measurement pair: last truth, last estimate, last raw sensor — into the series.

        The GPS outage is logged on the side: if the raw sensor stamp stays the same, no new
        measurement arrived — which is what happens at an arc edge of the arena.
        """
        a = s["task"]
        now_t = self._step_t
        if now_t < float(a.get("warmup", 3.0)) or now_t - self._probe_t < 0.04:
            return
        truth = self.bus.last("truth", self.name)[0]
        if truth is None:
            return
        self._probe_t = now_t
        estimate = self.bus.last("kf", self.name)[0]
        raw = self.bus.last(a.get("sensor", "gps"), self.name)[0]
        e_kf = math.hypot(estimate.x - truth.x, estimate.y - truth.y) if estimate else 1e9
        e_raw = math.hypot(raw.x - truth.x, raw.y - truth.y) if raw else 1e9
        nees = float("nan")
        if estimate and estimate.sx > 1e-6 and estimate.sy > 1e-6:
            nees = (((estimate.x - truth.x) ** 2 / estimate.sx ** 2
                     + (estimate.y - truth.y) ** 2 / estimate.sy ** 2) / 2.0)
        self._series.append((now_t, e_kf, e_raw, nees))
        self._note_outage(now_t, truth, raw, e_kf)

    def _note_outage(self, now_t: float, truth, raw, error: float) -> None:
        """No fresh raw sensor fix? Then the estimate runs on reserves — we measure that.

        The comparison runs over the **message stamps** (simulation time), not over the wall
        clock: tools/fastgrade.py runs 25 simulation seconds per second, and a wall-clock
        test would miss a GPS outage that everyone else can see.
        """
        if raw is None:
            self._outage.append((now_t, error))
            return
        as_time = getattr(truth, "t", None)
        stamp = getattr(raw, "t", None)
        if as_time is None or stamp is None:
            return
        if as_time - stamp > 2.0:
            self._outage.append((now_t, error))

    # ------------------------------------------------------------------------ evaluation

    def _close_step(self, s) -> None:
        if s["kind"] == "phase":
            self._eval_phase(s)
        elif s["kind"] == "mission_end":
            self._eval_mission(s)
        elif s["kind"] == "kf_eval":
            self._eval_kf(s)

    def _eval_phase(self, s) -> None:
        ph, now = s["ph"], self.bus.last("odom", self.name)[0]
        ergebnis, start = self.results[s["task"]["id"]], self._mark["odom"]
        if now is None or start is None:
            phase_ok, measured, reasons, lines = False, {}, ["no odometry received"], []
        else:
            measured = _delta(start, now)
            reasons, lines = _check(ph.get("expect", {}), measured)
            phase_ok = not reasons
        ergebnis["phases"][ph["id"]] = {"ok": phase_ok, "measured":
                                        {k: round(v, 3) for k, v in measured.items()},
                                        "criteria": lines, "reason": "; ".join(reasons)}
        per_phase = s["task"]["points"] / len(s["task"]["phases"])
        ergebnis["points"] = round(ergebnis["points"] + (per_phase if phase_ok else 0.0), 1)
        log.info("phase %-6s %s  %s", ph["id"], "ok" if phase_ok else "FAIL", measured)

    def _eval_mission(self, s) -> None:
        a, m = s["task"], self.results[s["task"]["id"]]
        now_robot = robot_io.robot_info(self.bus, self.name)
        source = a.get("source", "odom")
        basis = self._mission_mark or self._mark      # NOT _mark: that one gets a new
        start = basis[source]                          # value in the mission_ende step
        now = self.bus.last(source, self.name)[0]
        info_start = (self._mission_mark or self._mark)["robot"] or {}
        reasons = [] if not self._reason else [f"student reports: {self._reason}"]
        measured = {}
        if now is None or start is None:
            reasons.append(f"no {source.upper()} data")
        else:
            measured["time"] = round(self.t - basis["t"], 1)
            measured["path"] = round(float(now_robot.get("distance", 0)) - float(info_start.get("distance", 0)), 2)
            measured["contacts"] = int(now_robot.get("contacts", 0)) - int(info_start.get("contacts", 0))
            if self._lateral is not None:
                measured["lateral_distance"] = round(self._lateral, 3)     # smallest gap, see _act
            if a["id"] == "quadrat":
                measured["closure"] = round(T.pos_error(start, now), 3)
                measured["yaw_deg"] = round(math.degrees(T.yaw_error(start, now)), 1)
            else:
                target = self._target(a, now_robot)
                if target is None:
                    reasons.append("world goal unknown (is the world topic empty?)")
                else:
                    measured["target_error"] = round(T.pos_error(target, now), 3)
                    measured["target"] = [round(v, 2) for v in target[:2]]
        limits, criteria = _apply(MISSION_CRITERIA, measured, a)
        reasons += limits
        m["measured"], m["reason"] = measured, ("; ".join(reasons) or "meets requirements")
        m["criteria"] = criteria
        m["passed"] = not reasons
        m["points"] = a["points"] if m["passed"] else 0.0

    def _target(self, a: dict, robot_info: dict):
        """World goal: T3 the gate (goal), T4 a spawns entry — deliberately another target."""
        world_info = self.world_info or json.loads(self.bus.last("world")[0] or "{}")
        if a.get("target") == "spawn":
            starts = world_info.get("spawns") or []
            if not starts:
                return None
            index = a.get("target_index")
            if index is None:
                return starts[min(int(robot_info.get("index", 0)), len(starts) - 1)]
            return starts[int(index) % len(starts)]
        return world_info.get("goal")

    def _outage_duration(self) -> float:
        """How long no fresh raw sensor fix arrived during the run (in s)."""
        if not self._outage:
            return 0.0
        probe = (self._series[-1][0] - self._series[0][0]) / max(len(self._series) - 1, 1)
        return len(self._outage) * probe

    def _eval_kf(self, s) -> None:
        """Experiment 2: RMSE, improvement, maximum error, consistency — all against `truth`."""
        a, m = s["task"], self.results[s["task"]["id"]]
        series, reasons, measured, lines = self._series, [], {}, []
        info = robot_io.robot_info(self.bus, self.name) or {}
        if self._reason:
            reasons.append(f"student reports: {self._reason}")
        if len(series) < 10:
            measured["samples"] = len(series)
            reasons.append("no measurement pairs — is your node publishing /<robot>/kf/pose? Is "
                         "the sim publishing truth on /<robot>/truth? (automatic under ./lab "
                         "grade, otherwise pass --truth)")
        else:
            span = max(series[-1][0] - series[0][0], 1.0)
            rms = lambda i: math.sqrt(sum(x[i] ** 2 for x in series) / len(series))  # noqa: E731
            rmse, rmse_raw = rms(1), rms(2)
            measured["samples"] = len(series)
            measured["time"] = round(span, 1)
            measured["rmse"] = round(min(rmse, 999.0), 3)     # 1e9 = never an estimate: say so readably
            measured[f"rmse_{a.get('sensor', 'gps')}"] = round(rmse_raw, 3)
            measured["improvement"] = round(rmse_raw / rmse, 2) if rmse > 1e-9 else 0.0
            measured["max_error"] = round(min(max(x[1] for x in series), 999.0), 3)
            measured["rate_hz"] = round(self._kf_seen / span, 1)
            measured["contacts"] = int(info.get("contacts", 0)) - int(self._info_start.get("contacts", 0))
            nees = [x[3] for x in series if x[3] == x[3]]
            if nees:
                measured["nees"] = round(sum(nees) / len(nees), 2)
                measured["nees_over"] = round(sum(1 for v in nees if v > 5.99) / len(nees), 3)
            else:
                measured["nees"] = None
                if not self._kf_seen:
                    reasons.append("no kf/pose message received at all — is your node running "
                                 "under the same robot name, and does it call rob.send_kf(x, y, "
                                 "theta, sx, sy, sth)?")
                else:
                    reasons.append("no standard deviations in kf/pose (sx/sy are mandatory)")
            more, lines = _check_kf(a, measured)
            reasons += more
            outage = a.get("outage")
            if outage:
                span = self._outage_duration()
                measured["outage_duration"] = round(span, 1)
                measured["outage_max"] = (round(max(e for _, e in self._outage), 3)
                                      if self._outage else None)
                lines.append(f"outage {span:g} s ≥ {float(outage.get('duration_min', 1.0)):g} s")
                if span < float(outage.get("duration_min", 1.0)):
                    reasons.append(f"no GPS outage measurable ({span:.1f} s without a fix, "
                                 f"expected from {outage['duration_min']} s) — profile not driven?")
                elif measured["outage_max"] is not None:
                    lines.append(f"outage error {measured['outage_max']:g} ≤ "
                                 f"{float(outage['error_max']):g}")
                    if measured["outage_max"] > float(outage["error_max"]):
                        reasons.append(f"during the GPS outage {measured['outage_max']} m > "
                                       f"{outage['error_max']} m")
            reasons += _check_profile(a, self.bus)
        m["measured"], m["reason"] = measured, ("; ".join(reasons) or "meets requirements")
        m["criteria"] = lines
        m["passed"] = not reasons
        m["points"] = a["points"] if m["passed"] else 0.0
        log.info("KF %-14s %s  rmse=%s verb=%s", a["id"], "ok" if m["passed"] else "FAIL",
                 measured.get("rmse"), measured.get("improvement"))

    # --------------------------------------------------------------------------- result

    def report(self) -> dict:
        rows = list(self.results.values())
        for e in rows:
            if e["phases"]:
                ok = all(p["ok"] for p in e["phases"].values())
                e["passed"] = ok and e["points"] >= e["max_points"] - 0.05
                e["reason"] = "; ".join(
                    f'{pid}: {p["reason"]}' for pid, p in e["phases"].items() if not p["ok"]
                ) or f"all {len(e['phases'])} phases meet requirements"
        return {"robot": self.name, "time": round(self.t, 1), "tasks": rows,
                "points": round(sum(e["points"] for e in rows), 1),
                "max_points": sum(e["max_points"] for e in rows),
                "passed": bool(rows) and all(e["passed"] for e in rows)}


def drive_segments(a: dict) -> list:
    """Command segments of a grading run (experiment 2) — empty if the task drives freely."""
    return a.get("drive") or []


def drive_duration(a: dict) -> float:
    """How long the command sequence takes; without segments: timeout minus 1 s of quiet time."""
    seg = drive_segments(a)
    if not seg:
        return max(float(a.get("timeout", 30.0)) - 1.0, 1.0)
    return sum(float(s["duration"]) for s in seg) * max(int(a.get("repeat", 1)), 1)


def drive_command(a: dict, t: float) -> Twist:
    """Command at run time `t`: segments with a base value, optionally a superposed sine.

    `sinus` superposes `sinus·sin(2π·frequenz·t)` on the yaw rate — that turns a straight
    line into a driven curve, without the task text containing secret waypoints. After the
    last segment the robot holds still.
    """
    seg = drive_segments(a)
    if not seg:
        return Twist(float(a.get("vx", 0.3)), 0.0, float(a.get("omega", 0.0)))
    cycle = sum(float(s["duration"]) for s in seg)
    rest = t % cycle if cycle > 0 else 0.0
    for segment in seg:
        dur = float(segment["duration"])
        if rest >= dur:
            rest -= dur
            continue
        omega = float(segment.get("omega", 0.0))
        if segment.get("sine"):
            omega += float(segment["sine"]) * math.sin(
                2 * math.pi * float(segment.get("frequency", 0.1)) * rest)
        return Twist(float(segment.get("vx", 0.0)), float(segment.get("vy", 0.0)), omega)
    return Twist()


def _check_kf(a: dict, measured: dict) -> tuple:
    """Apply the thresholds of a KF task — the message always names the measured number.

    Answers the violations and the rows for the report; `_apply` says why the two come out of one
    loop. The names here are the ones in CONTRACT-KF §5, not the task-file keys, because the table in
    the handout and this line are the same promise seen from two sides.
    """
    reasons, lines = [], []
    limits = [("rmse", "rmse_max", "accuracy"), ("max_error", "max_error_max", "max error"),
              ("improvement", "improvement_min", "improvement over raw sensor"),
              ("rate_hz", "rate_min", "rate of kf/pose"),
              ("contacts", "contacts_max", "wall contacts")]
    for value, key, name in limits:
        bound = a.get(key)
        if bound is None or measured.get(value) is None:
            continue
        too_low = key.endswith("_min") and measured[value] < bound
        too_high = key.endswith("_max") and measured[value] > bound
        lines.append(f"{name} {measured[value]:g} {'≥' if key.endswith('_min') else '≤'} {bound:g}")
        if too_low or too_high:
            reasons.append(f"{name} {measured[value]} violates {key}={bound}")
    if a.get("nees") and measured.get("nees") is not None:
        lo, hi = [float(v) for v in a["nees"]]
        lines.append(f"NEES {measured['nees']:g} in [{lo:g} … {hi:g}]")
        if not lo <= measured["nees"] <= hi:
            reasons.append(f"NEES {measured['nees']} outside [{lo}, {hi}] — the stated standard "
                         "deviation does not match the actual error")
    return reasons, lines


def _check_profile(a: dict, bus) -> list:
    """Was the run really driven against the task's test profile? (otherwise the numbers are moot)"""
    want, driven = a.get("sim") or {}, {}
    try:
        driven = json.loads(bus.last("config")[0] or "{}")
    except (ValueError, TypeError):
        return []
    reasons = []
    for section in ("gps", "imu", "odom"):
        for key, value in (want.get(section) or {}).items():
            actual = (driven.get(section) or {}).get(key)
            if not isinstance(value, (int, float)) or not isinstance(actual, (int, float)) \
                    or abs(value) < 1e-12:
                continue
            ratio = actual / value
            if ratio < 0.67 or ratio > 1.5:
                reasons.append(f"test profile not driven: {section}.{key} = {actual} "
                             f"instead of {value} (start with ./lab grade … or kf.launch.py)")
    return reasons


def _delta(start, now) -> dict:
    """Path change in the start body frame: independent of how the robot began."""
    dx, dy = now.x - start.x, now.y - start.y
    c, s = math.cos(start.theta), math.sin(start.theta)
    return {"dx": c * dx + s * dy, "dy": -s * dx + c * dy,
            "yaw": wrap_angle(now.theta - start.theta)}


def _check(want: dict, measured: dict) -> list:
    """Apply the thresholds from tasks.json: dx_min, dy_betrag_max, winkel_min, …

    A `…_max` key bounds the *magnitude* (`abs(value)`), which is why the printed row shows the
    magnitude. Answers the violations and the report rows, as `_apply` does for a mission.
    """
    reasons, lines = [], []
    for key, bound in want.items():
        kind, richtung = key.split("_", 1)
        value = measured.get(kind)
        if value is None:
            continue
        violated = (value < bound) if richtung == "min" else (abs(value) > bound)
        lines.append(f"{kind} {abs(value):.2f} {'≥' if richtung == 'min' else '≤'} {bound:g}")
        if violated:
            reasons.append(f"{kind}={value:+.2f} violates {key}={bound}")
    return reasons, lines


# One row per mission criterion: the measured number, the key that bounds it in the task file, the
# sense of that bound, the bound a task without the key falls back to, and the sentence a violation
# makes. `contacts_max` falls back to 0 because that is what a task that says nothing means: no
# contact is welcome. `_eval_mission` applies exactly these rows and `format_report` prints exactly
# these rows, so the limit a student reads is the limit that was applied. Kept apart, the report was
# free to drift away from the check — and a 0/90 line that names neither the number nor the threshold
# it was compared against is where the K3 and T4 questions started.
#
# `max` bounds the magnitude: that is the convention `_check` already uses for the phase keys
# (`dy_betrag_max`) and the one the task text uses ("a final error < 0.25 m and a heading error
# < 20 degrees", T2). `yaw_deg` is the one signed number in the table, and comparing it without abs()
# let a solution that came 25° the wrong way home pass a task whose own description calls that a
# violation. The signed value stays in `measured`, so the direction is still on the report.
MISSION_CRITERIA = [
    ("target_error", "target_max", "max", None, "target missed: {v} m > {b} m"),
    ("closure", "closure_max", "max", None, "completion error {v} m > {b}"),
    ("yaw_deg", "yaw_max_deg", "max", None, "heading error {v}° > {b}°"),
    ("path", "path_min", "min", None, "path {v} m below {b} m — barely moved?"),
    ("path", "path_max", "max", None, "path {v} m above {b} m — a detour?"),
    ("contacts", "contacts_max", "max", 0, "{v} wall contacts (allowed {b})"),
    ("lateral_distance", "lateral_min", "min", None, "lateral {v} m below {b} m"),
    ("time", "timeout", "max", None, "time {v} s above limit {b} s"),
]


def _apply(criteria: list, measured: dict, a: dict) -> tuple:
    """Measured numbers against the task's own limits: `(violations, one row per criterion)`.

    A row is printed whether the criterion passed or failed (`path 2.14 m ≥ 1.5 m`): a passing run
    then shows what it had to meet, a failing one shows the number beside the threshold instead of
    only the number.
    """
    reasons, lines = [], []
    for key, limit_key, sense, fallback, template in criteria:
        bound, value = a.get(limit_key, fallback), measured.get(key)
        if bound is None or value is None:
            continue                                   # not a criterion of this task
        shown = abs(value) if sense == "max" else value      # what the row and the message quote
        missed = shown > bound if sense == "max" else value < bound
        lines.append(f"{key} {shown:g} {'≤' if sense == 'max' else '≥'} {bound:g}")
        if missed:
            reasons.append(template.format(v=shown, b=bound))
    return reasons, lines


def format_report(rep: dict) -> str:
    """Text table for the console — the same view the students get.

    Every line shows what was measured beside what was required (`_apply` builds the rows from the
    task file), because the verdict alone is the one thing a student cannot act on.
    """
    lines = [f"grading robot '{rep['robot']}'  ({rep['time']} s)",
              "-" * 66]
    for e in rep["tasks"]:
        lines.append(f"{'PASS' if e['passed'] else 'FAIL'}  {e['points']:5.1f}/"
                      f"{e['max_points']:3d} pts  {e['title']}")
        for pid, p in e["phases"].items():
            lines.append(f"   {'ok ' if p['ok'] else 'FAIL'}  {pid:7s} "
                          f"{' | '.join(p['criteria']) or p['measured']}")
            if not p["ok"]:
                lines.append(f"          why: {p['reason']}")
        for row in e["criteria"]:
            lines.append(f"        required: {row}")
        if e["measured"] and not e["criteria"]:
            lines.append(f"        measured: {e['measured']}")
        lines.append(f"        verdict: {e['reason']}")
    lines += ["-" * 66,
               f"Points reached: {rep['points']} / {rep['max_points']}"
               f"  ->  {'all tasks meet requirements' if rep['passed'] else 'needs rework'}"]
    return "\n".join(lines)


class FakeRobot:
    """Helper object for tests and the self test: a robot that drives commands ideally.

    `vy_flip = -1` simulates the classic sign error (lateral to the right instead of the
    left) — that is exactly how T1 has to fail. It drives missions as a straight-leg waypoint
    list, without a controller; it therefore shows the *result* of a solution, not its style.
    """

    def __init__(self, bus, name: str = "alice", vy_flip: float = 1.0, world_info: dict | None = None):
        self.bus, self.name, self.vy_flip = bus, name, float(vy_flip)
        self.world_info = world_info or {"name": "fake", "size": [12, 9], "goal": [4.0, 3.0, 0.0],
                             "spawns": [[1.0, 1.0, 0.0], [5.0, 5.0, 0.0]], "walls": 12}
        self.x, self.y, self.th = (*self.world_info["spawns"][0][:2], 0.0)
        self.vx = self.vy = self.om = 0.0
        self.distance, self.contacts = 0.0, 0
        self.mode, self.mission, self.task = "wheels", "idle", ""
        self.start, self.legs = (self.x, self.y, self.th), []
        self.command = Twist()
        bus.sub_topic(topic("twist", name), lambda t: setattr(self, "command", t))
        bus.sub_topic(topic("task"), self._task)
        self.report()

    def _task(self, name) -> None:
        name = str(name or "")
        if name == self.task:
            return                                         # the task was only repeated
        self.task = name
        if name in ("", "kinematik"):
            self.mission, self.legs, self.command = "idle", [], Twist()
        else:
            self.start, self.legs, self.mission = (self.x, self.y, self.th), self._leg_list(name), "running"

    def _leg_list(self, task: str) -> list:
        """Legs of a task — a square from the start pose, otherwise the world goal."""
        if task == "quadrat":
            x, y, th = self.start
            c, s = math.cos(th), math.sin(th)
            rand = [(0, 0), (1, 0), (1, 1), (0, 1), (0, 0)]
            return [(x + a * c - b * s, y + a * s + b * c) for a, b in rand]
        if task == "gps_anfahrt":
            return [self.world_info["spawns"][-1][:2]]          # loading bay = last spawns entry
        return [self.world_info["goal"][:2]] if self.world_info.get("goal") else []

    def report(self) -> None:
        self.bus.publish(topic("world"), json.dumps(self.world_info))
        self.bus.publish(topic("robots"), json.dumps(
            [{"name": self.name, "index": 0, "mode": self.mode, "contacts": self.contacts,
              "distance": round(self.distance, 3), "mission": self.mission, "task": self.task}]))

    def tick(self, dt: float) -> None:
        if self.mission == "running":
            self._drive_legs()
        vx, vy, om = self.command.vx, self.command.vy * self.vy_flip, self.command.omega
        c, s = math.cos(self.th), math.sin(self.th)
        dx, dy = (vx * c - vy * s) * dt, (vx * s + vy * c) * dt      # body into world frame
        self.x, self.y, self.th = self.x + dx, self.y + dy, self.th + om * dt
        self.vx, self.vy, self.om, self.command = vx, vy, om, Twist()
        self.distance += math.hypot(dx, dy)
        self.bus.publish(topic("odom", self.name), self._fill(Odom()))
        self.bus.publish(topic("gps", self.name), self._fill(Gps()))
        self.report()

    def _fill(self, m):
        m.x, m.y, m.theta, m.vx, m.vy, m.omega = self.x, self.y, self.th, self.vx, self.vy, self.om
        return m

    def _drive_legs(self, v: float = 0.4) -> None:
        """Drive one leg after the other — the reference solution does it with more detours."""
        while self.legs and math.hypot(self.legs[0][0] - self.x, self.legs[0][1] - self.y) < 0.06:
            self.legs.pop(0)
        if not self.legs:
            self.mission, self.command = "done", Twist()
            return
        dx, dy = self.legs[0][0] - self.x, self.legs[0][1] - self.y
        dist = math.hypot(dx, dy)
        self.command = Twist(vx=dx / dist * v, vy=dy / dist * v)


def main(argv=None) -> int:
    """Grading against a fake robot (no simulator) — the real run lives in node.py."""
    from . import stub
    ap = argparse.ArgumentParser(description="grader run with a fake robot (self test)")
    ap.add_argument("--robot", default="alice")
    ap.add_argument("--task", default="alle", help="task id or 'alle'")
    ap.add_argument("--json", help="write the report as JSON to this path")
    args = ap.parse_args(argv)
    fake = FakeRobot(stub.get_bus(), args.robot)
    gr = Grader(args.robot, args.task, fake.bus, T.load_tasks()).start()
    while not gr.tick(0.02):
        fake.tick(0.02)
    rep = gr.report()
    print(format_report(rep))
    if args.json:
        with open(args.json, "w", encoding="utf-8") as fh:
            json.dump(rep, fh, indent=1, ensure_ascii=False)
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
