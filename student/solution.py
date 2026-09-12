#!/usr/bin/env python3
"""Musterlösung Versuch 1 — Mecanum-Kinematik, Odometrie, LIDAR, GPS.

Liest sich wie ein Skriptabschnitt: erst die Rechnung (T1), dann ein Regler, der diese
Rechnung benutzt (T2), dann Sensorik (T3, T4). Derselbe Stoff wie in der Anleitung, nur
ausgeführt. Nachprüfen:

    ./lab grade --robot muster --task alle          # in Echtzeit, wie im Praktikum
    python3 tools/fastgrade.py --task alle --speed 4   # beschleunigt (Betreuer)

Konvention (Versuchsanleitung §2): x vorn, y LINKS, theta gegen den Uhrzeiger,
Radreihenfolge [VL, VR, HL, HR] in rad/s.
"""
import math
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import Gps, cfg_get, load_config, wrap_angle

CFG = load_config()
LX, LY, R = (cfg_get(CFG, "robot.lx"), cfg_get(CFG, "robot.ly"), cfg_get(CFG, "robot.r"))
A = LX + LY                                        # Hebelarm für Gier: a = lx + ly
RADIUS = cfg_get(CFG, "robot.footprint_r", 0.21)
SPIEL = 2 * RADIUS                                 # Reserve, die wir frei halten wollen
V_MAX, OM_MAX = 0.45, 1.1                          # m/s, rad/s — bewusst unter dem Maximum
K_POS, K_ROT = 1.1, 2.2                            # P-Verstärkungen [1/s]
TOL_XY, TOL_TH = 0.035, 0.06                        # Ziel-Toleranzen des P-Reglers
RICHTUNGEN = 24                                    # Sektoren des Freiheitsfelds (15 Grad)


# ------------------------------------------------------------- T1: inverse Kinematik

def inverse_kinematics(vx, vy, omega):
    """Körpergeschwindigkeit -> vier Radgeschwindigkeiten [VL, VR, HL, HR] in rad/s.

    Herleitung am Beispiel VL (vorn links, Rollerachse zeigt nach vorn-innen): Der Roller
    erlaubt freie Geschwindigkeit längs seiner Achse, angetrieben wird quer dazu. Adds man
    zur Körpergeschwindigkeit (vx, vy) den Gieranteil ω × Hebelarm, bleibt pro Rad genau
    eine Linearkombination übrig — geteilt durch den Radradius:

        [VL]   1 [ 1  -1  -a ] [  vx  ]
        [VR] = - [ 1  +1  +a ] [  vy  ]        Vorwärtskinematik = exakte Umkehrung (§2.4)
        [HL]   r [ 1  +1  -a ] [ omega ]
        [HR]     [ 1  -1  +a ]

    Vorzeichenproben gegen die GUI (kosten drei Zeilen, retten zwei Praktikumsstunden):
        vx > 0 -> alle vier positiv      (geradeaus)
        vy > 0 -> VL und HR negativ      (zur LINKEN Seite)
        ω > 0  -> VL und HL negativ      (linksherum)
    """
    return [(vx - vy - A * omega) / R,             # VL
            (vx + vy + A * omega) / R,             # VR
            (vx + vy - A * omega) / R,             # HL
            (vx - vy + A * omega) / R]             # HR


def klemm(wert, betrag):
    return max(-betrag, min(betrag, wert))


# ------------------------------------------------------------------ Sensor-Hilfsmittel

def freiheitsfeld(scan, richtungen=RICHTUNGEN):
    """Kürzester Treffer je Fahrtrichtung: aus 360 Strahlen werden 24 Sektoren.

    Pro Sektor das Minimum, nicht der Mittelwert — wer mittelt, übersieht die Tischkante.
    Diese eine Zahl pro Richtung ist alles, was ein Ausweichregler braucht.
    """
    if scan is None or not getattr(scan, "ranges", None):
        return [8.0] * richtungen
    n, rand = len(scan.ranges), scan.range_max
    breite = max(n // richtungen // 2, 1)
    feld = []
    for k in range(richtungen):
        mitte = k * n // richtungen
        feld.append(min(min(scan.ranges[(mitte + i) % n] for i in range(-breite, breite + 1)),
                        rand))
    return feld


def vy_quer(scan, totzone=0.04):
    """Kleiner Quer-Ausgleich aus den seitlichen Sektoren hält Abstand zu Wänden.

    In einem 1-m-Korridor würde man sonst unweigerlich an einer Seite langfahren; die
    Differenz der seitlichen linken und rechten Freiheit ist die Querabweichung direkt.
    Vorzeichen: vy positiv bedeutet nach links, also muss vy negativ werden, wenn links
    weniger frei ist.
    """
    if scan is None or not getattr(scan, "ranges", None):
        return 0.0
    n, rand = len(scan.ranges), scan.range_max
    seitlich = lambda indizes: min(min(scan.ranges[i] for i in indizes), rand)
    links = seitlich([n // 4, n // 5, 3 * n // 10])          # 90°, 72°, 108°
    rechts = seitlich([3 * n // 4, 4 * n // 5, 7 * n // 10])  # 270°, 288°, 252°
    fehl = (links - rechts) / 2.0
    if min(links, rechts) > 1.8 or abs(fehl) < totzone:
        return 0.0
    return klemm(1.1 * fehl, 0.24)


class Glättung:
    """Exponentieller Mittelwert über x und y — theta wird NIE gemittelt.

    Ein Winkel mittelbar über ±π hinweg ergibt Müll (der Roboter dreht sich im Kreis).
    Ohne Glättung jagt ein P-Regler bei 5 Hz GPS und σ = 6 cm jedem Rauschzipfel hinterher
    und pendelt um das Ziel — genau das ist T4s Lernpunkt.
    """

    def __init__(self, alpha=0.45):
        self.alpha, self.wert = alpha, None

    def __call__(self, mess):
        if mess is None:
            return None
        self.wert = (mess.x, mess.y) if self.wert is None else tuple(
            alt + self.alpha * (neu - alt) for alt, neu in zip(self.wert, (mess.x, mess.y)))
        return Gps(t=mess.t, x=self.wert[0], y=self.wert[1], theta=mess.theta)


# ------------------------------------------------------- Regler 1: Pose anfahren (T2)

def fahre_zur_pose(rob, ziel, ablesen, vmax=V_MAX, zeit_max=20.0, tol=TOL_XY, bremse=False):
    """P-Regler auf eine Ziel-Pose; Stellgröße ist die eigene inverse Kinematik.

    Der Positionsfehler wird in der Welt gemessen, gesteuert wird in Körpergeschwindigkeit,
    also den Fehler mit -theta in den Körperrahmen drehen:

        x_k =  cos θ · Δx + sin θ · Δy        y_k = -sin θ · Δx + cos θ · Δy

    Danach ist es reines P: vx = K·x_k, vy = K·y_k, ω = K_θ·Δθ. Kein Integralanteil nötig —
    die Odometrie ist selbst der Speicher, und dass sie driftet, ist ja der Auftrag.
    `bremse=True` lässt den LIDAR vorn mitentscheiden (für Fahrten durch eine volle Halle);
    T2 lässt es weg, weil dort ausdrücklich nur die Odometrie zählt.
    Die Frist läuft über die Simulationszeit in der Messung, nicht über die Wanduhr.
    """
    ende, vmax, geglättet = None, max(vmax, 0.05), Glättung() if ablesen is rob.gps else None
    while rob.running():
        mess = ablesen()
        if mess is None:
            rob.spin(0.05)
            continue
        if geglättet is not None:
            mess = geglättet(mess)
        if ende is None:
            ende = mess.t + zeit_max
        elif mess.t > ende:
            return False
        dx, dy = ziel[0] - mess.x, ziel[1] - mess.y
        längs = math.cos(mess.theta) * dx + math.sin(mess.theta) * dy
        quer = -math.sin(mess.theta) * dx + math.cos(mess.theta) * dy
        dtheta = wrap_angle(ziel[2] - mess.theta)
        if math.hypot(dx, dy) < tol and abs(dtheta) < TOL_TH:
            rob.publish_cmd_vel(0.0, 0.0, 0.0)
            return True
        vx, vy = klemm(K_POS * längs, vmax), klemm(K_POS * quer, vmax * 0.7)
        om = klemm(K_ROT * dtheta, OM_MAX)
        if bremse:
            feld = freiheitsfeld(rob.scan())
            if feld[0] < SPIEL:                       # Hindernis im Weg: nicht dagegen
                vx = min(vx, 0.05)                    # laufen, sondern vorbeidrehen
                om = klemm(om + (1.2 if feld[RICHTUNGEN // 4] > feld[3 * RICHTUNGEN // 4]
                                 else -1.2), OM_MAX)
        rob.publish_cmd_vel(vx, vy, om)
        rob.spin(0.02)
    return False


def fahre_quadrat(rob):
    """T2: vier Seiten à 1 m, 90° linksherum, zurück auf die Startpose — nur Odometrie."""
    start = rob.odom()
    if start is None:
        raise RuntimeError(f"keine Odometrie für '{rob.name}' — läuft der Simulator?")
    seite, ecke, pose = 1.0, math.pi / 2, [start.x, start.y, start.theta]
    for _ in range(4):
        ziel = [pose[0] + seite * math.cos(pose[2]), pose[1] + seite * math.sin(pose[2]),
                pose[2]]
        if not fahre_zur_pose(rob, ziel, rob.odom, zeit_max=22.0):
            raise RuntimeError("Seite nicht erreicht — Toleranz zu eng oder Zeit zu knapp")
        pose = [ziel[0], ziel[1], wrap_angle(pose[2] + ecke)]
    if not fahre_zur_pose(rob, [start.x, start.y, start.theta], rob.odom, zeit_max=22.0):
        raise RuntimeError("nicht zurück am Start angekommen")


# ------------------------------------------- Regler 2: durch die Halle, LIDAR-gestützt

def fahre_hin(rob, ziel, holung="odom", tol=0.20, zeit_max=100.0, vmax=V_MAX):
    """T3 und T4: mit dem Freiheitsfeld zu einem Punkt — nur die Quelle unterscheidet sie.

    Zwei Phasen, weil beide ihre eigene Fehlerquelle haben:

    1. Weite Strecke: von den 24 Richtungen des Feldes kommen nur die in die engere Wahl,
       die mindestens SPIEL frei haben; von denen nehmen wir die richtungsärmste zum Ziel,
       mit leichter Bevorzugung offener Richtungen (bricht das Umkreisen eines Tischbeins,
       den klassischen Fehler reiner Potentialfelder). Tempo wächst mit der Freiheit der
       gewählten Richtung — bremsen passiert von allein, bevor es knallt.
       Gegen Verrennen: 15 s ohne 30 cm Näherung heißt "Sackgasse" — dann drei Sekunden
       lang nur der freiesten Richtung folgen (die billige Variante von Umplanung).
    2. Letzte 1,3 m: dort bremst der Feldregler zu stark aus und versickert; also mit dem
       Pose-Regler und LIDAR-Bremse genau einfahren.

    `holung` ist der Lernunterschied: T3 misst sich mit Odometrie, T4 mit geglättetem GPS.
    """
    quelle = rob.odom if holung == "odom" else rob.gps
    geglättet = Glättung() if quelle is rob.gps else None
    letzte_entfernung, stillstand, ende, umweg_bis, letzte_messt = None, 0.0, None, 0.0, None
    schritt = 2 * math.pi / RICHTUNGEN
    while rob.running():
        mess, scan = quelle(), rob.scan()
        if mess is None or scan is None:
            rob.spin(0.05)
            continue
        if geglättet is not None:
            mess = geglättet(mess)
        if ende is None:
            ende = mess.t + zeit_max
        entfernung = math.hypot(ziel[0] - mess.x, ziel[1] - mess.y)
        if entfernung < tol:
            rob.publish_cmd_vel(0.0, 0.0, 0.0)
            rob.spin(0.4)
            return True
        if entfernung < 1.3:
            return fahre_zur_pose(rob, [ziel[0], ziel[1], mess.theta], quelle, vmax=0.24,
                                  zeit_max=max(1.0, ende - mess.t), tol=min(tol, 0.20),
                                  bremse=True)
        verstrichen = 0.0 if letzte_messt is None else max(0.0, mess.t - letzte_messt)
        letzte_messt = mess.t
        if letzte_entfernung is None:
            letzte_entfernung = entfernung
        elif entfernung < letzte_entfernung - 0.30:
            letzte_entfernung, stillstand = entfernung, 0.0
        else:                                  # Fortschritt in MESSEzeit messen, nicht pro
            stillstand += verstrichen          # Schleifendurchlauf (beschleunigte Laeufe!)
        umweg = mess.t < umweg_bis
        if not umweg and stillstand > 15.0:
            umweg_bis, stillstand = mess.t + 3.0, 0.0
        feld = freiheitsfeld(scan)
        fahrbar = [k for k in range(RICHTUNGEN) if feld[k] >= SPIEL]
        if not fahrbar:                               # nirgends Platz: freiste Richtung
            k = max(range(RICHTUNGEN), key=lambda k: feld[k])
            vx, om = 0.0, klemm(1.4 * wrap_angle(k * schritt), 1.4)
        else:
            zielwinkel = wrap_angle(math.atan2(ziel[1] - mess.y, ziel[0] - mess.x)
                                    - mess.theta)

            def kosten(k):
                richtung = wrap_angle(k * schritt)
                if umweg:                             # nur Freiheit, Ziel ignorieren
                    return -feld[k]
                return abs(wrap_angle(zielwinkel - richtung)) + 0.12 * max(0.0, 1.6 - feld[k])

            k = min(fahrbar, key=kosten)
            richtung = wrap_angle(k * schritt)
            vx = vmax * min(1.0, max(0.0, (feld[k] - SPIEL) / 0.55))
            om = klemm(K_ROT * wrap_angle(richtung if umweg
                                          else 0.35 * zielwinkel + 0.65 * richtung), OM_MAX)
        rob.publish_cmd_vel(vx, vy_quer(scan), om)
        rob.spin(0.02)
        if mess.t > ende:
            raise RuntimeError(f"Ziel in {zeit_max:.0f} s nicht erreicht, noch "
                               f"{entfernung:.2f} m entfernt")
    return False


def fahre_korridor(rob):
    """T3: vom Start zum Tor der Welt. Odometrie kennt die Richtung, LIDAR die Wände."""
    ziel = (rob.world() or {}).get("goal")
    if not ziel:
        raise RuntimeError("Welt hat kein Ziel — G in worlds/<name>.txt gesetzt?")
    return fahre_hin(rob, ziel, "odom", tol=0.20, zeit_max=100.0)


def fahre_zu_gps(rob):
    """T4 (Bonus): Ladeplatz = letzter Spawn-Punkt der Welt, angefahren mit GPS.

    Neu gegenüber T3 ist nur die Quelle: GPS ist verrauscht und langsamer (5 Hz) ->
    Glättung und eine Totzone von 15 cm, damit der Regler nicht um das Ziel pendelt. Und
    das Ziel ist absichtlich NICHT die eigene Startpose, sonst wäre T4 geschenkt.
    """
    starts = (rob.world() or {}).get("spawns") or []
    if not starts:
        raise RuntimeError("Welt hat keine Startpunkte (S/2/3/4)")
    return fahre_hin(rob, starts[-1], "gps", tol=0.15, zeit_max=80.0, vmax=0.30)


# ------------------------------------------------------------------------- Anschluss

def mission(rob, task):
    """Vom Runner für T2..T4 genau einmal aufgerufen, solange rob.running().

    Fertig = normal zurückkehren (der Runner meldet "done"). Aufgegeben = Exception
    werfen: der Runner meldet "failed:<Grund>", was ehrlicher ist als ein "done" ohne
    Zielankunft.
    """
    for name, funktion in (("quadrat", fahre_quadrat), ("korridor", fahre_korridor),
                           ("gps_anfahrt", fahre_zu_gps)):
        if task.startswith(name):
            return funktion(rob)
    raise RuntimeError(f"unbekannter Auftrag '{task}'")


if __name__ == "__main__":
    # serve() reicht bei T1 jede cmd_vel durch inverse_kinematics durch und ruft für
    # T2..T4 mission() auf; publish_cmd_vel ist dabei der einzige Stellpfad.
    robot_io.serve(sys.modules[__name__])
