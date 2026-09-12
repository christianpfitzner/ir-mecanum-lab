#!/usr/bin/env python3
"""Musterlösung Versuch 2 — ein Kalman-Filter, vier Sensorprofile, kein numpy.

Der Bewerter fährt, du schätzt: dieser Knoten sendet keine Räder und kein cmd_vel
(pass-through-Modus, CONTRACT-KF §1). Er liest GPS, Odometrie und IMU und meldet seine
Schätzung mit deren Unsicherheit auf /<robot>/kf/pose.

Zustand im WELTframe (im Körperframe wandert die Drift mit, die man gerade loswill):

    x = (x, y, vx, vy)                          Position und Geschwindigkeit [m, m/s]

    Prädiktion   x' = F·x,  P' = F·P·Fᵀ + Q(dt)         F = CV-Modell, Q aus dt
    Richtung     θ' = θ + ω·dt,  ω aus dem IMU-Gyro (Bias vorher im Stillstand gemittelt)
    Update 1     Positions-Update:   GPS (x, y)      R = sigma_xy²      → H = [I 0]
    Update 2     Bewegungs-Update:   Odometrie (vx, vy), mit θ in die Welt gedreht
                                     → H = [0 I], R = sigma_v²
    Meldung      sx = √P_xx, sy = √P_yy, sth = √var_θ   — 1σ, nicht Varianz (K3!)

Update 2 ist der Grund, warum das hier auch ohne GPS funktioniert: die Zustandsgrößen
vx, vy werden von der Odometrie bei jedem Takt auf wenige mm/s festgehalten, die
Prädiktion schiebt die Position also mit der *gemessenen* Körpergeschwindigkeit fort.
Im Funkloch von K2 bleibt genau dieses Update übrig — der Filter rechnet dann sauber
weiter, statt geradeaus auszubrechen. Die Beschleunigung der IMU wird bewusst NICHT
integriert: ihr Bias von 0,05 m/s² wird doppelintegriert nach 10 s zu 2,5 m (K2, Hilfen).

Drei Details, die in der Praxis den Unterschied machen:

  * Zeit ist Simulationszeit. Der Filter taktet über die Stempel der Messungen
    (odom.t, imu.t, fix.t), nie über die Wanduhr — tools/fastgrade.py fährt 25× schneller.
  * Ein Fix ist beim Empfang schon ein paar Millisekunden alt. Er wird auf seinen
    Messzeitpunkt zurückgerechnet (geparkte Schritte zurückspulen, nachfahren), nicht
    auf "jetzt" verbrochen; bei 0,85 m/s sind 100 ms Rückstand sonst 8,5 cm Folgefehler.

  * Die eigene Angabe muss zum Fehler passen: alle 20 Fixes vergleicht der Filter die
    Streuung seiner Innovation mit dem angekündigten S und korrigiert Q in kleinen
    Schritten (K3, der einzige Weg ohne Raten zu NEES ≈ 1). Ebenso zwei Einstellungen,
    die K2 sonst reißen: Q_GAP im Funkloch (Odometrie-Drift ist kein weißes Rauschen) und
    verwerfen von Fixes vor Missionsbeginn — der Bus hält die des vorigen Auftrags bereit.

Nachprüfen (beides ohne ROS, ohne Fenster, dieselbe Bewertung):

    python3 tools/fastgrade.py --task kf_alle --controller student/kf_solution.py --speed 25
    ./lab grade --task kf_alle --controller student/kf_solution.py
"""
import math
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import wrap_angle

# ------------------------------------------------------------------- Parameter
Q_ACC = 6.0          # m²/s³  Prozessrausch: alles, was das CV-Modell nicht darf
Q_GAP = 3.0          # Faktor, solange kein GPS korrigiert (Funktzeit > 2 s)
SIGMA_V = 0.08        # m/s    Streuen der gemessenen Radgeschwindigkeit (Update 2)
SIGMA_TH = 0.03       # rad/√s Richtungsunsicherheit des Gyros — nur für die Meldung sth
BIAS_PROBEN = 80      # IMU-Stichproben im Stillstand, dann ist der Gyro-Bias gemittelt
STILLSTAND = 0.02     # m/s: ab hier gilt der Roboter als stehend (Kalibrierfenster)
P0_POS = 0.50         # m      Anfangs-1σ Position — bewusst groß, das GPS darf erst führen
P0_VEL = 0.50         # m/s    Anfangs-1σ Geschwindigkeit
P0_TH = 0.05          # rad    Anfangs-1σ Richtung
MELDE_DT = 0.02       # s = 50 Hz: Meldetakt von kf/pose (Schwelle: mindestens 10 Hz)
PUFFER = 120          # Schritte, die für verspätete Fixe zurückgespult werden können
VERJAEHT = 2.0        # s: ältere Fixe gehören nicht mehr in diesen Auftrag


# ------------------------------------------------------- Matrixhilfe von Hand
# Absichtliche Handarbeit: vier mal vier, Listen von Zeilen, keine Bibliothek. Wer die
# Rechnung in Matrixform sieht, erkennt sie in der Vorlesung wieder — und sieht sofort,
# dass ein Kalman-Filter nichts weiter ist als ein paar Matrizenmultiplikationen.

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
    return [[a + b for a, b in zip(x, y)] for x, y in zip(A, B)]


def msub(A, B):
    return [[a - b for a, b in zip(x, y)] for x, y in zip(A, B)]


def inv2(M):
    """Inverse einer 2×2 — die Formel aus der Vorlesung. Entartet: Identität statt Crash."""
    d = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    if abs(d) < 1e-12:
        return [[1.0, 0.0], [0.0, 1.0]]
    return [[M[1][1] / d, -M[0][1] / d], [-M[1][0] / d, M[0][0] / d]]


def cv_matrix(dt):
    """F des CV-Modells: die Position wächst mit v·dt, die Geschwindigkeit bleibt."""
    return [[1.0, 0.0, dt, 0.0], [0.0, 1.0, 0.0, dt],
            [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]


def q_matrix(q, dt):
    """Q aus der Zeitdifferenz dt: weißes Beschleunigungsrausch mit Dichte q [m²/s³].

    Je Achse die exakt integrierte Form q·[[dt³/3, dt²/2], [dt²/2, dt]] (die dt⁴/4-Variante
    der Aufgabenhilfe ist die Näherung dafür); die beiden Achsen sind unabhängig, also steht
    die 2×2-Form zweimal diagonal verschränkt in der 4×4. Ist positiv semidefinit:
    Determinant q²·dt⁴/12 > 0 und Spur > 0.
    """
    a, b, c = q * dt ** 3 / 3.0, q * dt ** 2 / 2.0, q * dt
    return [[a, 0.0, b, 0.0], [0.0, a, 0.0, b],
            [b, 0.0, c, 0.0], [0.0, b, 0.0, c]]


class KF:
    """Kalman-Filter für (x, y, vx, vy) im Weltframe — 4×4, mit der Hand ausgerollt."""

    def __init__(self, x, y, vx, vy, theta, t):
        self.x = [x, y, vx, vy]
        d = [P0_POS ** 2, P0_POS ** 2, P0_VEL ** 2, P0_VEL ** 2]
        self.P = [[d[i] if i == j else 0.0 for j in range(4)] for i in range(4)]
        self.t, self.theta, self.var_th = float(t), theta, P0_TH ** 2
        self.start = float(t)          # Mission beginnt hier: fruehere Messungen gehoeren ihr nicht
        self.omega, self.n, self.innov = 0.0, 0, 0.0
        self.letzter_fix = float(t)              # wann zuletzt ein GPS-Fix korrigiert hat
        self.q_faktor, self.yy, self.ss, self.zaehler = 1.0, 0.0, 0.0, 0   # Konsistenzregel (K3)
        self.schritte = []                     # (t_ende, dt, ω, vx, vy, R_v, Zustand vorher)

    # ------------------------------------------------------------------ Prädiktion
    def prediction(self, dt, omega):
        """Ein Schritt über dt Sekunden: Richtung aus dem Gyro, dann F·x und F·P·Fᵀ + Q."""
        if dt <= 0.0:
            return
        self.omega = omega
        self.theta = wrap_angle(self.theta + omega * dt)
        self.var_th += (SIGMA_TH * dt) ** 2          # ohne Messung weiß man immer weniger
        #  Bleibt die Korrektur lange aus (GPS-Funkloch), driftet die Odometrie weiter — und
        #  ihre Drift ist gerade *kein* weißes Rauschen (Vorfrage 3). Darf das Restrausch
        #  schneller wachsen als das CV-Modell sagt; sonst wird P zu klein und die eigene
        #  Angabe unehrlich (das NEES steigt über 3).
        Q = q_matrix(Q_ACC * self.q_faktor * (Q_GAP if self.t - self.letzter_fix > 2.0 else 1.0), dt)
        F = cv_matrix(dt)
        self.x = mv(F, self.x)
        self.P = madd(mul(mul(F, self.P), transpose(F)), Q)
        self.t += dt

    def update(self, stellen, messwert, R, regel=False):
        """Ein Messupdate auf den Zuständen `stellen` (Diagonal-Messung, R ist deren Varianz).

        H hat genau auf diesen Stellen Einsen, also ist S = P[Auszug] + R und K = P·Hᵀ·S⁻¹.
        Danach wie im Buch: x += K·y und P -= K·S·Kᵀ (Joseph-Form, symmetrisch gehalten).
        Wird zweimal benutzt: (0, 1) für das GPS, (2, 3) für die Odometrie.
        """
        S = [[self.P[i][j] + (R if i == j else 0.0) for j in stellen] for i in stellen]
        Si = inv2(S)                                  # die einzige Inverse, die wir brauchen
        K = [[sum(self.P[i][stellen[k]] * Si[k][j] for k in range(2)) for j in range(2)]
             for i in range(4)]
        y = [messwert[m] - self.x[stellen[m]] for m in range(2)]
        if regel:
            self.konsistenz(y, S[0][0] + S[1][1])
        self.x = [self.x[i] + K[i][0] * y[0] + K[i][1] * y[1] for i in range(4)]
        self.P = msub(self.P, mul(mul(K, S), transpose(K)))
        self.n += 1
        return y

    def konsistenz(self, y, s):
        """K3 in drei Zeilen: passt die Innovationsstreuung zu dem, was der Filter angekündigt hat?

        Ist die Innovation grosser als `S`, stimmt das Modell nicht — Q war zu klein. Ist sie
        kleiner, war Q zu gross und die angegebene Kovarianz ist aufgeblaht. Korrigiert wird
        alle 20 Fixes um einen festen Faktor: schnell genug zum Lernen, traege genug, um nicht
        hinter jedem Ausreisser herzulaufen.
        """
        self.yy += y[0] ** 2 + y[1] ** 2
        self.ss, self.zaehler = self.ss + s, self.zaehler + 1
        if self.zaehler < 20:
            return
        verhaeltnis = self.yy / max(self.ss, 1e-9)
        if verhaeltnis > 1.4:
            self.q_faktor = min(self.q_faktor * 1.4, 12.0)
        elif verhaeltnis < 0.7:
            self.q_faktor = max(self.q_faktor * 0.75, 0.05)
        self.yy = self.ss = 0.0
        self.zaehler = 0

    def schritt(self, dt, omega, vx_welt, vy_welt, R_v):
        """Prädiktion + Bewegungs-Update als eine Einheit — sie wird bei spätem Fix nachgefahren."""
        vor = (list(self.x), [z[:] for z in self.P], self.theta, self.var_th)
        self.prediction(dt, omega)
        self.update((2, 3), [vx_welt, vy_welt], R_v)
        self.schritte.append((self.t, dt, omega, vx_welt, vy_welt, R_v, vor))
        del self.schritte[:-PUFFER]

    def spule(self, t):
        """Zustand auf den letzten Schritt vor `t` zurücksetzen; liefert die Schritte danach."""
        zurueck = []
        while self.schritte and self.schritte[-1][0] > t:
            ende, dt, omega, vx_welt, vy_welt, R_v, vor = self.schritte.pop()
            self.x, self.P, self.theta, self.var_th = vor[0], vor[1], vor[2], vor[3]
            self.t, self.omega = ende - dt, omega
            zurueck.append((dt, omega, vx_welt, vy_welt, R_v))
        return zurueck

    def richtung_update(self, th_gps, R_th):
        """Skalar-Update der Richtung aus dem GPS-Winkel: hält θ langfristig, ohne es zu erzwingen."""
        k = self.var_th / (self.var_th + R_th)
        self.theta = wrap_angle(self.theta + k * wrap_angle(th_gps - self.theta))
        self.var_th *= 1.0 - k

    def sigmas(self):
        """1σ aus P — die Zahl, die K3 bewertet, und der einzige Grund für P überhaupt."""
        return (math.sqrt(max(self.P[0][0], 0.0)), math.sqrt(max(self.P[1][1], 0.0)),
                math.sqrt(max(self.var_th, 0.0)))


# --------------------------------------------------------------------- Anschlusscode

def mission(rob, task):
    """Schätzen, solange dieser Auftrag läuft. Gesteuert wird nicht (CONTRACT-KF §8.1).

    Der Loop taktet über Messstempel: neu gerechnet wird, wenn eine Messung da ist, die
    später gestempelt ist als der Zustand. `rob.spin()` hält die Busse lebendig, Ende ist
    der Auftragwechsel — der Runner meldet dann "done" für den Bewerter.
    """
    f, sigma_xy, sigma_th = None, 0.5, 0.2
    bias_summe, bias_n, bias = 0.0, 0, 0.0
    letzter_fix, letzter_meldung = 0.0, -1e9
    R_v = SIGMA_V ** 2
    while rob.running() and rob.task() == task:
        rob.spin(0.005)
        o, i, gps = rob.odom(), rob.imu(), rob.gps()
        if o is None:
            continue

        # 1) Gyro-Bias im Stillstand mitteln — kurze Kalibrierung, dann ist er eingefroren.
        #    Der Rest des Bias-Themas (Random Walk) steckt in Q, nachjustieren wäre Schummeln.
        if bias_n < BIAS_PROBEN and i is not None:
            if math.hypot(o.vx, o.vy) < STILLSTAND and abs(i.gz) < 0.2:
                bias_summe, bias_n = bias_summe + i.gz, bias_n + 1
                bias = bias_summe / bias_n

        if f is None:                                  # Start aus der Odometrie: Pose ist gut
            f = KF(o.x, o.y, o.vx, o.vy, o.theta, o.t)
            sigma_xy = float(rob.config("gps.sigma_xy", 0.5) or 0.5)
            sigma_th = float(rob.config("gps.sigma_theta", 0.2) or 0.2)

        # 2) Prädiktion bis zum neuesten Messstempel, mit Gierrate und Weltgeschwindigkeit
        gierrate = (i.gz - bias) if i is not None else o.omega
        t_mess = max(o.t, i.t if i is not None else 0.0)
        if t_mess > f.t:
            c, s = math.cos(f.theta), math.sin(f.theta)
            f.schritt(t_mess - f.t, gierrate, c * o.vx - s * o.vy, s * o.vx + c * o.vy, R_v)

        # 3) Positions-Update auf den Messzeitpunkt des Fixes — nicht auf "jetzt"
        #    ein Fix vom Auftrag davor (der Bus merkt sich die letzte Meldung!) waere eine Luege.
        if gps is not None and gps.t >= f.start and gps.t > letzter_fix and gps.t >= f.t - VERJAEHT:
            letzter_fix = f.letzter_fix = gps.t
            nachfahren = f.spule(gps.t)                # zurück auf fix.t ...
            if gps.t > f.t:
                f.prediction(gps.t - f.t, f.omega)     # ... eventuell die Lücke schließen
            y = f.update((0, 1), [gps.x, gps.y], sigma_xy ** 2, regel=True)
            f.innov = math.hypot(*y)
            f.richtung_update(gps.theta, sigma_th ** 2)
            for dt, omega, vx_w, vy_w, rv in reversed(nachfahren):
                f.schritt(dt, omega, vx_w, vy_w, rv)    # ... und die Zeit bis jetzt nachfahren

        # 4) Melden: Schätzung plus 1σ aus P, im Meldetakt über die Simulationszeit
        if f.t - letzter_meldung >= MELDE_DT:
            letzter_meldung = f.t
            sx, sy, sth = f.sigmas()
            rob.send_kf(f.x[0], f.x[1], f.theta, sx, sy, sth,
                        info={"t": round(f.t, 2), "q_acc": Q_ACC, "sigma_xy": sigma_xy,
                              "bias_gz": round(bias, 5), "innov": round(f.innov, 3),
                              "updates": f.n})


if __name__ == "__main__":
    # Kein inverse_kinematics, kein send_wheels, kein publish_cmd_vel: der Bewerter fährt.
    robot_io.serve(sys.modules[__name__])
