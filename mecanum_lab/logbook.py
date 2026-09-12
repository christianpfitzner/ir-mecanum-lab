"""Messprotokoll als CSV — die Datenbasis für den Versuchsbericht (Versuch 2).

`tap()` hängt am Simulationslauf und bekommt jede Messung mit; geschrieben wird alle
`intervall` Sekunden **Simulationszeit** je Roboter eine Zeile. Warum Simulationszeit:
die Auswertung muss hinterher sagen können „bei t = 23 s war der letzte GPS-Fix 0,8 s
alt" — mit einer Wanduhr in der Spalte würde ein langsamer Rechner andere Kurven malen.

Format: Semikolon-getrennt, Dezimalpunkt, Kopfzeile — damit `csv`, pandas und Excel
gleichermaßen klarkommen:

    import csv
    with open("messung.csv") as fh:
        reihen = list(csv.DictReader(fh, delimiter=";"))

Dazu gehört `tools/kfplot.py`, das dieselbe Datei auswertet (RMSE, Zeitreihen, und ein
ASCII-Diagramm für Rechner ohne matplotlib).
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

# Was aus welcher Nachricht in welche Spalte wandert (Feldnamen der Dataclasses, s. types.py)
FELDER = {"truth": {"x": "x_wahr", "y": "y_wahr", "theta": "th_wahr"},
          "gps": {"x": "x_gps", "y": "y_gps", "theta": "th_gps", "t": "t_gps"},
          "odom": {"x": "x_odom", "y": "y_odom", "theta": "th_odom"},
          "kf": {"x": "x_kf", "y": "y_kf", "theta": "th_kf", "sx": "sx_kf", "sy": "sy_kf"},
          "imu": {"ax": "ax_imu", "ay": "ay_imu", "gz": "gz_imu"}}


class Logbuch:
    """Schreibt, was der Roboter gesehen hat — und was er sich daraus zusammengereimt hat."""

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
        """Eine Nachricht aus der Simulations-Outbox; hier wird nur zwischengespeichert."""
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
        """Eine Zeile je Roboter, sobald `intervall` Sekunden Simzeit vergangen sind."""
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
            log.info("Messprotokoll: %d Zeilen -> %s", self.zeilen, self.pfad)


def _zahl(obj, feld: str):
    """Feldwert aus einer Dataclass — tolerant, ein Protokoll darf nie abbrechen."""
    try:
        return float(getattr(obj, feld))
    except (TypeError, ValueError):
        return None


def _r(wert) -> str:
    try:
        return f"{float(wert):.4f}"
    except (TypeError, ValueError):
        return ""
