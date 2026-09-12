"""Measurement log as CSV — the data basis for the lab report (lab 2).

`tap()` hangs on the simulation loop and sees every measurement; one line per robot is
written every `interval` seconds of **simulation time**. Why simulation time: the analysis
must be able to say later "at t = 23 s the last GPS fix was 0.8 s old" — with a wall clock
in that column a slow computer would draw different curves.

Format: semicolon-separated, decimal point, header row — so that `csv`, pandas and Excel
all handle it the same way:

    import csv
    with open("messung.csv") as fh:
        rows = list(csv.DictReader(fh, delimiter=";"))

`tools/kfplot.py` belongs to it and evaluates the same file (RMSE, time series, and an
ASCII diagram for computers without matplotlib).
"""
import csv
import logging

log = logging.getLogger("mecanum.logbook")

COLUMNS = ["t", "robot",
           "x_truth", "y_truth", "th_truth", "vx_truth", "vy_truth", "omega_truth",
           "x_gps", "y_gps", "th_gps", "t_age_gps",
           "x_odom", "y_odom", "th_odom",
           "x_kf", "y_kf", "th_kf", "sx_kf", "sy_kf", "n_kf",
           "ax_imu", "ay_imu", "gz_imu"]

# Which field of which message goes into which column (dataclass field names, see types.py)
FIELDS = {"truth": {"x": "x_truth", "y": "y_truth", "theta": "th_truth"},
          "gps": {"x": "x_gps", "y": "y_gps", "theta": "th_gps", "t": "t_gps"},
          "odom": {"x": "x_odom", "y": "y_odom", "theta": "th_odom"},
          "kf": {"x": "x_kf", "y": "y_kf", "theta": "th_kf", "sx": "sx_kf", "sy": "sy_kf"},
          "imu": {"ax": "ax_imu", "ay": "ay_imu", "gz": "gz_imu"}}


class Logbook:
    """Writes what the robot saw — and what it inferred from those readings."""

    def __init__(self, engine, path: str, interval: float = 0.05):
        self.eng = engine
        self.interval = max(float(interval), 0.005)
        self.last_t, self.n_kf, self.lines = -1e9, 0, 0
        self.values: dict[str, dict] = {}
        self.file = open(path, "w", encoding="utf-8", newline="")
        self.writer = csv.writer(self.file, delimiter=";")
        self.writer.writerow(COLUMNS)
        self.path = path

    def tap(self, kind: str, robot: str | None, payload) -> None:
        """One message from the simulation outbox; only buffered here."""
        if robot is None or payload is None:
            return
        latest = self.values.setdefault(robot, {})
        for field, column in FIELDS.get(kind, {}).items():
            if hasattr(payload, field):
                latest[column] = _num(payload, field)
        if kind == "kf":
            self.n_kf += 1
            latest["n_kf"] = self.n_kf

    def tick(self) -> None:
        """One line per robot once `interval` seconds of simulation time have passed."""
        t = float(self.eng.t)
        if t - self.last_t < self.interval:
            return
        self.last_t = t
        for name, latest in self.values.items():
            tw = getattr(getattr(self.eng.robots.get(name), "twist", None), "__dict__", {})
            latest.update(vx_truth=tw.get("vx"), vy_truth=tw.get("vy"),
                          omega_truth=tw.get("omega"), n_kf=latest.get("n_kf", ""),
                          t_age_gps=(t - latest["t_gps"]) if latest.get("t_gps") is not None else None)
            self.writer.writerow([round(t, 3), name] + [_r(latest.get(c)) for c in COLUMNS[2:]])
            self.lines += 1

    def close(self) -> None:
        if not self.file.closed:
            self.file.close()
            log.info("measurement log: %d lines -> %s", self.lines, self.path)


def _num(obj, field: str):
    """Field value from a dataclass — tolerant, a log must never break the run."""
    try:
        return float(getattr(obj, field))
    except (TypeError, ValueError):
        return None


def _r(value) -> str:
    try:
        return f"{float(value):.4f}"
    except (TypeError, ValueError):
        return ""
