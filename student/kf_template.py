#!/usr/bin/env python3
"""Versuch 2 — Zustandsschätzung mit einem Kalman-Filter. Das hier ist deine Abgabedatei.

Der Bewerter fährt deinen Roboter, du steuerst nichts: kein inverse_kinematics, kein
send_wheels, kein publish_cmd_vel (CONTRACT-KF §8.1). Du liest GPS, Odometrie und IMU und
meldest deine Schätzung auf /<robot>/kf/pose — samt 1σ, denn K3 bewertet genau diese Zahl.
Konvention (§2): x, y Weltmeter, x vorn, y LINKS, theta gegen den Uhrzeiger. Die Odometrie
liefert die Körpergeschwindigkeit (vx vorn, vy links), die IMU die Gierrate gz; az enthält +g
und ax/ay sind verbiasst — eine IMU nie offen integrieren (K2, Hilfen). Alles im WELTframe,
sonst wandert die Drift mit, die du gerade loswerden willst:

    x = (x, y, vx, vy)   Prädiktion x' = F·x, P' = F·P·Fᵀ + Q(dt)
                         Positions-Update   GPS       H = [I 0], R = sigma_xy²
                         Bewegungs-Update   Odometrie H = [0 I], R = sigma_v²
                         Meldung: sx = √P_xx, sy = √P_yy    (1σ, nicht Varianz!)

Starten (die Sim fährt selbst, --truth zeigt die exakte Pose als Geisterkontur):
    ./lab run --world arena --task kf_gps --robot alice --controller student/kf_template.py --truth
Prüfen wie später: ./lab grade --task kf_gps --controller student/kf_template.py
Musterlösung: student/kf_solution.py — erst rechnen, dann vergleichen. Die sieben TODOs sind die
ganze Rechenaufgabe; mit den TODOs läuft der Knoten schon und meldet immer wieder seine Startpose.
"""
import math
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import wrap_angle

Q_ACC = 12.0          # m²/s³  Prozessrausch: alles, was das CV-Modell nicht darf
SIGMA_V = 0.08        # m/s    Streuen der gemessenen Radgeschwindigkeit (Bewegungs-Update)
SIGMA_TH = 0.03       # rad/√s Richtungsunsicherheit des Gyros — nur für die Meldung sth
BIAS_PROBEN, STILLSTAND = 80, 0.02     # IMU-Mittelung im Stillstand: Stichproben, Stehschwelle
P0_POS, P0_VEL, P0_TH = 0.50, 0.50, 0.05   # Anfangs-1σ: Position, Geschwindigkeit, Richtung
MELDE_DT = 0.02       # s = 50 Hz: Meldetakt von kf/pose (K4 verlangt mindestens 10 Hz)

# ---------------------------------------------------------- Matrixhilfe von Hand
# Anschlusscode, fertig: ein Kalman-Filter ist ein Haufen Matrizenmultiplikationen.

def mul(A, B):
    """A·B für kleine Matrizen (Listen von Zeilen) — drei Schleifen, mehr braucht es nicht."""
    C = [[0.0] * len(B[0]) for _ in range(len(A))]
    for i, zeile in enumerate(A):
        for k, a in enumerate(zeile):
            if a:
                for j, b in enumerate(B[k]):
                    C[i][j] += a * b
    return C

def mv(A, v):
    """A·x — Matrix mal Spaltenvektor."""
    return [sum(a * b for a, b in zip(zeile, v)) for zeile in A]

def transpose(A):
    """Aᵀ — aus Spalten werden Zeilen."""
    return [list(spalte) for spalte in zip(*A)]

def madd(A, B):
    """A + B und A − B elementweise (Zeile für Zeile)."""
    return [[a + b for a, b in zip(x, y)] for x, y in zip(A, B)]

def msub(A, B):
    return [[a - b for a, b in zip(x, y)] for x, y in zip(A, B)]

def inv2(M):
    """Inverse einer 2×2 — Formel aus der Vorlesung. Entartet: Identität statt Crash."""
    d = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(d) < 1e-12:
        return [[1.0, 0.0], [0.0, 1.0]]
    return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]]

def cv_matrix(dt):
    """TODO 1: baue F des CV-Modells über dt Sekunden.

    Erwartung: die Position wächst mit v·dt, die Geschwindigkeit bleibt — Einheitsmatrix mit
    dt auf den beiden Kopplungsplätzen. Probe: mv(cv_matrix(0.5), [0,0,2,4]) == [1,2,2,4].
    """
    return [[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],      # TODO 1: durch F ersetzen
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]

def q_matrix(q, dt):
    """TODO 2: baue Q aus der Zeitdifferenz dt (weißes Beschleunigungsrausch, Dichte q m²/s³).

    Erwartung: je Achse die 2×2-Form q·[[dt³/3, dt²/2],[dt²/2, dt]], beide Achsen unabhängig,
    also zweimal diagonal verschränkt in die 4×4. Proben: positiv semidefinit, für dt = 0
    überall null. Großes Q = „mehr Modellfehler eingeräumt" = der Filter folgt dem GPS
    schneller: K1 wird schlechter, K3-NEES sinkt — das ist der Zielkonflikt, nicht Hexenwerk.
    """
    return [[0.0] * 4 for _ in range(4)]                      # TODO 2: durch Q ersetzen


class KF:
    """Kalman-Filter für (x, y, vx, vy) im Weltframe — 4×4, mit der Hand ausgerollt."""

    def __init__(self, x, y, vx, vy, theta, t):
        self.x = [x, y, vx, vy]
        d = [P0_POS ** 2, P0_POS ** 2, P0_VEL ** 2, P0_VEL ** 2]
        self.P = [[d[i] if i == j else 0.0 for j in range(4)] for i in range(4)]
        self.t, self.theta, self.var_th = float(t), theta, P0_TH ** 2
        self.start = float(t)          # Mission beginnt hier: fruehere Messungen gehoeren ihr nicht
        self.omega, self.n, self.innov = 0.0, 0, 0.0

    def prediction(self, dt, omega):
        """Ein Schritt über dt: Richtung aus dem Gyro, dann x' = F·x und P' = F·P·Fᵀ + Q."""
        if dt <= 0.0:
            return
        self.omega = omega
        self.theta = wrap_angle(self.theta + omega * dt)
        self.var_th += (SIGMA_TH * dt) ** 2          # ohne Messung weiß man immer weniger
        F, Q = cv_matrix(dt), q_matrix(Q_ACC, dt)
        self.x = mv(F, self.x)
        self.P = madd(mul(mul(F, self.P), transpose(F)), Q)
        self.t += dt

    def update(self, stellen, messwert, R):
        """Messupdate auf den Zuständen `stellen`: (0,1) = GPS-Position, (2,3) = Geschwindigkeit.

        H hat genau auf diesen Stellen Einsen, deshalb sind S = P[Auszug] + R eine 2×2 (die
        einzige Inverse, die wir brauchen) und K = P·Hᵀ·S⁻¹ eine 4×2.
        """
        S = [[self.P[i][j] + (R if i == j else 0.0) for j in stellen] for i in stellen]
        Si = inv2(S)
        # TODO 3: baue K (4×2):  K[i][j] = sum(P[i][stellen[k]] * Si[k][j] for k in (0,1))
        K = [[0.0, 0.0] for _ in range(4)]                     # TODO 3: durch K ersetzen
        y = [messwert[m] - self.x[stellen[m]] for m in range(2)]  # Innovation = Messung − Prognose
        # TODO 4: Zustand korrigieren — für jedes i: self.x[i] += K[i][0]*y[0] + K[i][1]*y[1]
        # TODO 5: Kovarianz korrigieren — self.P = msub(self.P, mul(mul(K, S), transpose(K)))
        #   (hält P symmetrisch; wer P unverändert lässt, schätzt für immer mit der Anfangs-1σ)
        self.n += 1
        return y

    def schritt(self, dt, omega, vx_welt, vy_welt, R_v):
        """Prädiktion plus Bewegungs-Update: die Odometrie hält vx, vy auf wenige cm/s fest."""
        self.prediction(dt, omega)
        self.update((2, 3), [vx_welt, vy_welt], R_v)

    def sigmas(self):
        """TODO 6: die gemeldete 1σ — sx = √P[0][0], sy = √P[1][1], sth = √var_θ.

        Erwartung: sx fällt in den ersten Sekunden von P0_POS auf einige Zentimeter und wächst
        im Funkloch wieder an. Ohne diese Zahlen ist K3 nicht bewertbar.
        """
        return 0.0, 0.0, 0.0                                       # TODO 6: durch √P ersetzen


# --------------------------------------------------------------------- Anschlusscode

def mission(rob, task):
    """Schätzen, solange dieser Auftrag läuft — der Runner meldet danach "done".

    Der Loop taktet über Messstempel: gerechnet wird nur, wenn eine Messung später gestempelt
    ist als der eigene Zustand. Nie über die Wanduhr — tools/fastgrade.py fährt 25× schneller,
    dann liefe ein wanduhr-getakteter Filter in die falsche Richtung.
    """
    f, sigma_xy, bias, letzter_fix, letzter_meldung = None, 0.5, 0.0, 0.0, -1e9
    bias_summe, bias_n, R_v = 0.0, 0, SIGMA_V ** 2
    while rob.running() and rob.task() == task:
        rob.spin(0.005)
        o, i, gps = rob.odom(), rob.imu(), rob.gps()
        if o is None:
            continue
        # TODO 7: Gyro-Bias im Stillstand mitteln — solange hypot(o.vx, o.vy) < STILLSTAND und
        #   bias_n < BIAS_PROBEN: bias_summe += i.gz, bias_n += 1, bias = bias_summe / bias_n.
        #   Ohne Mittelung zieht ein Bias von 5 mrad/s die Richtung in 10 s um 3 Grad weg.
        if f is None:                                   # Start aus der Odometrie: Pose ist gut
            f = KF(o.x, o.y, o.vx, o.vy, o.theta, o.t)
            sigma_xy = float(rob.config("gps.sigma_xy", 0.5) or 0.5)   # raten ist unnötig
        gierrate = (i.gz - bias) if i is not None else o.omega
        t_mess = max(o.t, i.t if i is not None else 0.0)
        if t_mess > f.t:                     # Körpergeschwindigkeit mit theta in die Welt drehen
            c, s = math.cos(f.theta), math.sin(f.theta)
            f.schritt(t_mess - f.t, gierrate, c * o.vx - s * o.vy, s * o.vx + c * o.vy, R_v)
        # Positions-Update: jeder Fix genau einmal. Und nur Fixes aus diesem Auftrag — der
        # Bus merkt sich die letzte Meldung, ein Fix vom Auftrag davor (vor dem Respawn) wäre
        # für diese Mission eine Lüge. Rückfrage an f.start genügt.
        if gps is not None and gps.t >= f.start and gps.t > letzter_fix:
            letzter_fix = gps.t
            f.prediction(max(gps.t - f.t, 0.0), f.omega)  # bis zum Messzeitpunkt vorrechnen
            f.innov = math.hypot(*f.update((0, 1), [gps.x, gps.y], sigma_xy ** 2))
        if f.t - letzter_meldung >= MELDE_DT:            # Schätzung inkl. 1σ auf kf/pose
            letzter_meldung = f.t
            sx, sy, sth = f.sigmas()
            rob.send_kf(f.x[0], f.x[1], f.theta, sx, sy, sth,
                        info={"t": round(f.t, 2), "q_acc": Q_ACC, "innov": round(f.innov, 3)})


if __name__ == "__main__":
    # Der Runner übernimmt Bus und Lebenszyklus und ruft mission() je Auftrag genau einmal auf.
    robot_io.serve(sys.modules[__name__])
