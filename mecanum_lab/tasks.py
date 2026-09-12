"""Tasks for the lab: one read access to config/tasks.json plus small calculations.

The grader (grade.py) and the command line (node.py) both take everything from here;
the thresholds live **only** in the JSON file, so the supervisor can adjust them
without touching code. No ROS and no simulator dependency.
"""
import json
import math
import os

from .types import ROOT, wrap_angle

TASKS_PATH = os.path.join(ROOT, "config", "tasks.json")


def load_tasks(path: str | None = None) -> dict:
    """Read the task file; a clear error when it is missing (then the repo is incomplete)."""
    way = path or TASKS_PATH
    if not os.path.exists(way):
        raise FileNotFoundError(f"task file missing: {way}")
    with open(way, encoding="utf-8") as fh:
        cfg = json.load(fh)
    if not cfg.get("tasks"):
        raise ValueError(f"no tasks in {way}")
    return cfg


def task_ids(cfg: dict) -> list:
    return [t["id"] for t in cfg["tasks"]]


def by_id(cfg: dict) -> dict:
    return {t["id"]: t for t in cfg["tasks"]}


def resolve(cfg: dict, ids=None) -> list:
    """Pick tasks in their intended order: None/"alle"/list/"a,b"/groups such as "kf_alle".

    Groups (a reading aid only, no file of their own): `kf_alle` or `kf_*` = every task
    with the `kf` prefix, `v2`/`versuch2` = everything with `"versuch": 2`. `alle` means the
    tasks of **Experiment 1** — that name predates Experiment 2, and Experiment 1's grading
    runs hang on one world; `beide`/`alle_versuche` really takes all of them. Unknown names
    are an error — otherwise half a grading run goes through in silence.
    """
    order = cfg.get("reihenfolge") or task_ids(cfg)
    known = by_id(cfg)
    if ids is None or (isinstance(ids, str) and not ids.strip()):
        wanted = list(order)
    else:
        roh = [ids] if isinstance(ids, str) else list(ids)
        if len(roh) == 1 and "," in roh[0]:
            roh = [w.strip() for w in roh[0].split(",") if w.strip()]
        wanted, offen = [], []
        for w in roh:
            if w in known:
                wanted.append(w)
            elif w in ("alle", "all"):
                wanted += [i for i in order if known[i].get("versuch", 1) == 1]
            elif w in ("beide", "alle_versuche"):
                wanted += list(order)
            elif w in ("kf_alle", "alle_kf") or w.startswith("kf_") and w.endswith("*"):
                wanted += [i for i in order if i.startswith("kf_")]
            elif w in ("v2", "versuch2", "versuch_2"):
                wanted += [i for i in order if known[i].get("versuch") == 2]
            elif w in ("v1", "versuch1", "versuch_1"):
                wanted += [i for i in order if known[i].get("versuch", 1) == 1]
            elif w.endswith("*"):
                wanted += [i for i in order if i.startswith(w[:-1])]
            else:
                offen.append(w)
        if offen:
            raise ValueError(f"unknown task(s) {offen}; available: {', '.join(order)} "
                             "(groups: alle, kf_alle, v1, v2, beide)")
    return [known[i] for i in order if i in dict.fromkeys(wanted)]


def sim_profil(cfg: dict, ids=None) -> dict:
    """Starting simulation profile: the `sim` block of the first selected task.

    Any task may carry a `"sim"` block (`gps`, `imu`, `odom`, `debug_truth`, …); node.py
    applies it before the engine is built, and per task the grader sets its own test
    profile afterwards. At start-up only *one* profile may apply: merging the blocks of
    several tasks would be nonsense — kf_gps wants 5 Hz GPS, kf_fusion 1 Hz with an
    outage, and in the end kf_gps would silently have to measure at 1 Hz with an outage.
    """
    auftrge = resolve(cfg, ids)
    return dict(auftrge[0].get("sim") or {}) if auftrge else {}


def titel(cfg: dict, task_id: str) -> str:
    return by_id(cfg).get(task_id, {}).get("titel", task_id)


def short_help(cfg: dict | None = None) -> str:
    """One-liner for --help: 'kinematik (30 P) | quadrat (30 P) | ...'."""
    cfg = cfg or load_tasks()
    return " | ".join(f'{t["id"]} ({t["punkte"]} P)' for t in cfg["tasks"])


def welt_fuer(cfg: dict, ids=None, default: str = "production") -> str:
    """Which world suits these tasks (most frequent recommendation; first one on a tie)."""
    hints = [t.get("welt") for t in resolve(cfg, ids) if t.get("welt")]
    if not hints:
        return default
    return max(set(hints), key=hints.count)


# ------------------------------------------------------------------------ measurements


def pos_fehler(a, b) -> float:
    """Air distance between two poses (a, b with .x/.y or [x, y, theta])."""
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return math.hypot(ax - bx, ay - by)


def winkel_fehler(a, b) -> float:
    """Largest magnitude of the rotation to credit (radians, signed b-a)."""
    return wrap_angle(_th(b) - _th(a))


def grad(fehlers: float) -> float:
    return math.degrees(fehlers)


def seitlicher_abstand(scan, stichproben: int = 5) -> float | None:
    """Distance to the nearest side wall, measured between 60 and 120 degrees.

    The LIDAR numbers beam 0 at the front and then counter-clockwise. To an angle a
    belong the indices i and N-i — mirrored front and back, whichever side is nearer.
    Beams without a hit (inf) are not counted.
    """
    if scan is None or not getattr(scan, "ranges", None):
        return None
    n = len(scan.ranges)
    step = scan.angle_increment or (2 * math.pi / n)
    nah = []
    for k in range(stichproben):
        a = math.pi / 3 + (math.pi / 3) * k / max(stichproben - 1, 1)
        i = int(round(a / step)) % n
        for cand in (i, (-i) % n):
            r = scan.ranges[cand]
            if math.isfinite(r) and r < scan.range_max:
                nah.append(r)
    return min(nah) if nah else None


def _xy(p):
    if p is None:
        return (float("nan"), float("nan"))
    if isinstance(p, (list, tuple)):
        return (float(p[0]), float(p[1]))
    return (float(getattr(p, "x", 0.0)), float(getattr(p, "y", 0.0)))


def _th(p) -> float:
    if p is None:
        return 0.0
    if isinstance(p, (list, tuple)):
        return float(p[2]) if len(p) > 2 else 0.0
    return float(getattr(p, "theta", 0.0))
