"""Aufträge für Versuch 1: ein Lesenzugriff auf config/tasks.json plus Rechentricks.

Der Bewerter (grade.py) und die Kommandozeile (node.py) holen sich hier alles her;
die Schwellen stehen **nur** in der JSON-Datei, damit Betreuer sie ohne Code-Änderung
anpassen können. Keine ROS- und keine Simulator-Abhängigkeit.
"""
import json
import math
import os

from .types import ROOT, wrap_angle

TASKS_PATH = os.path.join(ROOT, "config", "tasks.json")


def load_tasks(path: str | None = None) -> dict:
    """Auftragsdatei lesen; klarer Fehler, wenn sie fehlt (dann ist das Repo unvollständig)."""
    way = path or TASKS_PATH
    if not os.path.exists(way):
        raise FileNotFoundError(f"Auftragsdatei fehlt: {way}")
    with open(way, encoding="utf-8") as fh:
        cfg = json.load(fh)
    if not cfg.get("tasks"):
        raise ValueError(f"Keine Aufträge in {way}")
    return cfg


def task_ids(cfg: dict) -> list:
    return [t["id"] for t in cfg["tasks"]]


def by_id(cfg: dict) -> dict:
    return {t["id"]: t for t in cfg["tasks"]}


def resolve(cfg: dict, ids=None) -> list:
    """Aufträge in Soll-Reihenfolge auswählen: None/"alle"/Liste/"a,b"/Gruppen wie "kf_alle".

    Gruppen (nur Lesehilfe, keine eigene Datei): `kf_alle` bzw. `kf_*` = alle Aufträge
    mit `kf`-Präfix, `v2`/`versuch2` = alles mit `"versuch": 2`. `alle` meint die Aufgaben
    von **Versuch 1** — der Name war vor Versuch 2 da, und die Bewertungsläufe von Versuch 1
    hängen an einer Welt; `beide`/`alle_versuche` nimmt wirklich alle. Unbekannte Namen sind
    ein Fehler — sonst läuft still eine halbe Bewertung durch.
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
            raise ValueError(f"Unbekannte(r) Auftrag {offen}; es gibt: {', '.join(order)} "
                             "(Gruppen: alle, kf_alle, v1, v2, beide)")
    return [known[i] for i in order if i in dict.fromkeys(wanted)]


def sim_profil(cfg: dict, ids=None) -> dict:
    """Startprofil der Simulation: der `sim`-Block des ersten gewählten Auftrags.

    Jede Aufgabe darf einen `"sim"`-Block tragen (`gps`, `imu`, `odom`, `debug_truth`, …);
    node.py wendet ihn vor der Engine an, und je Auftrag setzt der Bewerter sein eigenes
    Prüfprofil nach. Beim Start darf aber nur *ein* Profil gelten: die Blöcke mehrerer
    Aufträge zu verschmelzen wäre unsinnig — kf_gps will 5 Hz GPS, kf_fusion 1 Hz mit
    Funkloch, und am Ende hätte kf_gps still mit 1 Hz und Funkloch messen müssen.
    """
    auftrge = resolve(cfg, ids)
    return dict(auftrge[0].get("sim") or {}) if auftrge else {}


def titel(cfg: dict, task_id: str) -> str:
    return by_id(cfg).get(task_id, {}).get("titel", task_id)


def short_help(cfg: dict | None = None) -> str:
    """Einzeiler für --help: 'kinematik (30 P) | quadrat (30 P) | ...'."""
    cfg = cfg or load_tasks()
    return " | ".join(f'{t["id"]} ({t["punkte"]} P)' for t in cfg["tasks"])


def welt_fuer(cfg: dict, ids=None, default: str = "production") -> str:
    """Welche Welt für diese Aufträge passt (häufigste Empfehlung; bei Konflikt die erste)."""
    hints = [t.get("welt") for t in resolve(cfg, ids) if t.get("welt")]
    if not hints:
        return default
    return max(set(hints), key=hints.count)


# ------------------------------------------------------------------------ Messgrößen


def pos_fehler(a, b) -> float:
    """Luftlinie zwischen zwei Posen (a, b mit .x/.y oder [x, y, theta])."""
    ax, ay = _xy(a)
    bx, by = _xy(b)
    return math.hypot(ax - bx, ay - by)


def winkel_fehler(a, b) -> float:
    """Größter Betrag der anzurechnenden Drehung (radiant, vorzeichenbehaftet b-a)."""
    return wrap_angle(_th(b) - _th(a))


def grad(fehlers: float) -> float:
    return math.degrees(fehlers)


def seitlicher_abstand(scan, stichproben: int = 5) -> float | None:
    """Abstand zur nächstgelegenen Seitenwand, gemessen zwischen 60 und 120 Grad.

    Der LIDAR nummeriert Strahl 0 vorn und dann gegen den Uhrzeigersinn. Zu einem
    Winkel a gehören die Indizes i und N-i — vorn wie hinten gespiegelt, egal welche
    Seite näher ist. Strahlen ohne Treffer (inf) zählen nicht mit.
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
