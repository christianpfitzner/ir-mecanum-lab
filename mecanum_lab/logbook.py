"""Measurement log as CSV — the data basis for the lab report (lab 2).

`tap()` hangs on the simulation run and sees every measurement; one line per robot is
written every `intervall` seconds of **simulation time**. Why simulation time: the analysis
must be able to say later "at t = 23 s the last GPS fix was 0.8 s old" — with a wall clock
in that column a slow computer would draw different curves.

Format: semicolon-separated, decimal point, header row — so that `csv`, pandas and Excel
all handle it the same way:

    import csv
    with open("messung.csv") as fh:
        reihen = list(csv.DictReader(fh, delimiter=";"))

`tools/kfplot.py` belongs to it and evaluates the same file (RMSE, time series, and an
ASCII diagram for computers without matplotlib).
"""
import csv
import logging

log = logging.getLogger("mecanum.logbook")

SPALTEN = ["t", "robot",
           "x_wahr", "y_wahr", "th_wahr", "vx_wahr", "vy_wahr", "omega_wahr",
           "x_gps", "y_gps", "th_gps", "t_alt_gps",
           "x_odom", "y_odom", "th_odom",
           "x_kf", "y_kf", "th_kf", "sx_kf", "sy_kf", "n_kf",
           "ax_imu", "ay_imu", "gz_imu"]

# Which field of which message goes into which column (dataclass field names, see types.py)
FELDER = {"truth": {"x": "x_wahr", "y": "y_wahr", "theta": "th_wahr"},
          "gps": {"x": "x_gps", "y": "y_gps", "theta": "th_gps", "t": "t_gps"},
          "odom": {"x": "x_odom", "y": "y_odom", "theta": "th_odom"},
          "kf": {"x": "x_kf", "y": "y_kf", "theta": "th_kf", "sx": "sx_kf", "sy": "sy_kf"},
          "imu": {"ax": "ax_imu", "ay": "ay_imu", "gz": "gz_imu"}}


class Logbuch:
    """Writes what the robot saw — and what it inferred from those readings."""

    def __init__(self, engine, pfad: str, intervall: float = 0.05):
        self.eng = engine
        self.intervall = max(float(intervall), 0.005)
        self.probe_t, self.n_kf, self.zeilen = -1e9, 0, 0
        self.werte: dict[str, dict] = {}
        self.datei = open(pfad, "w", encoding="utf-8", newline="")
        self.schreiber = csv.writer(self.datei, delimiter=";")
        self.schreiber.writerow(SPALTEN)
        self.pfad = pfad

    def tap(self, kind: str, robot: str | None, payload) -> None:
        """One message from the simulation outbox; only buffered here."""
        if robot is None or payload is None:
            return
        w = self.werte.setdefault(robot, {})
        for feld, spalte in FELDER.get(kind, {}).items():
            if hasattr(payload, feld):
                w[spalte] = _zahl(payload, feld)
        if kind == "kf":
            self.n_kf += 1
            w["n_kf"] = self.n_kf

    def tick(self) -> None:
        """One line per robot once `intervall` seconds of simulation time have passed."""
        t = float(self.eng.t)
        if t - self.probe_t < self.intervall:
            return
        self.probe_t = t
        for name, w in self.werte.items():
            tw = getattr(getattr(self.eng.robots.get(name), "twist", None), "__dict__", {})
            w.update(vx_wahr=tw.get("vx"), vy_wahr=tw.get("vy"), omega_wahr=tw.get("omega"),
                     n_kf=w.get("n_kf", ""),
                     t_alt_gps=(t - w["t_gps"]) if w.get("t_gps") is not None else None)
            self.schreiber.writerow([round(t, 3), name] + [_r(w.get(s)) for s in SPALTEN[2:]])
            self.zeilen += 1

    def close(self) -> None:
        if not self.datei.closed:
            self.datei.close()
            log.info("measurement log: %d lines -> %s", self.zeilen, self.pfad)


def _zahl(obj, feld: str):
    """Field value from a dataclass — tolerant, a log must never break the run."""
    try:
        return float(getattr(obj, feld))
    except (TypeError, ValueError):
        return None


def _r(wert) -> str:
    try:
        return f"{float(wert):.4f}"
    except (TypeError, ValueError):
        return ""
