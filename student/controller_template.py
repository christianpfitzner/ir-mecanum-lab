#!/usr/bin/env python3
"""Versuch 1 — Mecanum-Räder ansteuern. Das hier ist deine Abgabedatei.

Konvention (Versuchsanleitung §2, sie steht sonst nirgends so genau da):
    x  zeigt in Fahrtrichtung        [m/s]
    y  zeigt nach LINKS              [m/s]   <- links ist positiv, nicht rechts!
    theta dreht gegen den Uhrzeiger  [rad/s] <- linksherum ist positiv
    Räder in der Reihenfolge [VL, VR, HL, HR], Angaben in rad/s (nicht U/min!)

    Draufsicht, Rollerachsen in X-Anordnung:        inverse Kinematik = deine Aufgabe:
        VL ╱····╲ VR                                  a = lx + ly
        (antriebsmotor je Rad, Rolle diagonal)        VL = (vx - vy - a·ω) / r
        HL ╲····╱ RR                                  ...drei weitere folgen

Starten (ein Kommando, kein ROS nötig):
    ./lab run --robot alice --controller student/controller_template.py
Selbst kontrollieren:
    ./lab grade --robot alice --task kinematik     # prüft nur deine Vorzeichen
Fahren mit der Tastatur (nur wenn du gar nichts tust, Simulator-Fernsteuerung):
    ./lab sim --world maze            # dann in einem zweiten Terminal ./lab teleop --robot alice

Du änderst nur zwei Funktionen: `inverse_kinematics` (Teilaufgabe 1) und die drei
`fahre_*`-Funktionen (Teilaufgaben 2-4). Der Rest ist Anschlusscode.
"""
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import cfg_get, load_config

# Fahrwerksmaße aus config/default.json — auf dem echten Roboter sind es die
# gemessenen Werte, hier vorgegeben. Nicht ändern, sonst stimmt deine Rechnung
# nicht mit der Simulation überein.
CFG = load_config()
LX = cfg_get(CFG, "robot.lx")            # halber Radstand in x [m]
LY = cfg_get(CFG, "robot.ly")            # halbe Breite in y    [m]
R = cfg_get(CFG, "robot.r")              # Radradius            [m]
A = LX + LY                              # Hebelarm für die Gierbewegung


# ------------------------------------------------------------------- Teilaufgabe 1 (T1)

def inverse_kinematics(vx, vy, omega):
    """Körpergeschwindigkeit -> vier Radgeschwindigkeiten [VL, VR, HL, HR] in rad/s.

    Herleitung siehe Versuchsanleitung: Die Roller erlauben Geschwindigkeit längs
    ihrer Achse, das Antriebsrad liefert die Komponente quer dazu. Addiert man für
    ein Rad die Körpergeschwindigkeit (vx, vy) und den Gierterm ω×hebelarm, bleibt
    genau die Rollrichtung übrig — deshalb hebt sich in jedem Term eine Hälfte weg.

    Merkhilfe statt Abschreiben:  VL und RR sind gleichsinnig, VR und HL sind deren
    Spiegelbilder an der x-Achse; für eine Drehung nach links (ω > 0) müssen die
    rechten Räder schneller laufen als die linken.
    """
    vl = 0.0      # TODO 1: Term für VL, vorn links
    vr = 0.0      # TODO 2: Term für VR, vorn rechts  (Spiegelung von VL an y)
    hl = 0.0      # TODO 3: Term für HL, hinten links (Spiegelung von VR an x)
    hr = 0.0      # TODO 4: Term für HR, hinten rechts
    return [vl, vr, hl, hr]


# ------------------------------------------------------------------- Teilaufgaben 2 bis 4

def fahre_quadrat(rob):
    """T2: 1 m pro Seite, 90°-Ecken, Odometrie als Rückführung, closed loop.

    Erfolg (./lab grade --task quadrat): nach vier Seiten wieder unter 0,20 m und
    15° Abstand zur Startpose, Gesamtzeit unter 90 s, keine Wandberührung.
    Idee: fahre gerade mit Soll-vx, korrigiere die Querabweichung aus der Odometrie;
    in der Ecke drehe bis die Odometrie-Theta 90° weiter ist.
    """
    start = rob.odom()
    if start is None:
        raise RuntimeError(f"keine Odometrie fuer '{rob.name}'")
    # TODO 5: vier Seiten lang fahre_strecke(rob, 1.0) und drehe(rob, pi/2) aufrufen
    raise RuntimeError("T2 noch nicht implementiert")


def fahre_korridor(rob):
    """T3: aus dem Labyrinth mit LIDAR zum Ziel, ohne eine Wand zu streifen.

    Erfolg (./lab grade --task korridor): Ziel erreicht (Abstand < 0,30 m),
    contacts == 0, und seitlicher Wandabstand nie unter 0,25 m gefallen.
    Idee: Median der vorderen linken und rechten Strahlen hält dich mittig,
    der vorderste Strahl macht dich langsam, wenn es eng wird.
    """
    # TODO 6: rob.scan() benutzen — ranges[0] ist vorn, Winkel gegen den Uhrzeiger
    raise RuntimeError("T3 noch nicht implementiert")


def fahre_zu_gps(rob):
    """T4 (Bonus): Zielanfahrt mit der globalen Position statt mit Odometrie.

    Erfolg (./lab grade --task gps_anfahrt): Ziel aus rob.world()["goal"] mit
    Abstand < 0,25 m erreicht; der GPS-Rauschteppich darf dich nicht schütteln
    (Totzone und gedämpfte Drehrate sind erlaubt).
    """
    goal = (rob.world() or {}).get("goal")
    if not goal:
        raise RuntimeError("kein Ziel in rob.world() — Welt ohne G?")
    # TODO 7: Fehlervektor in den Körperframe drehen, dann inverse_kinematics
    raise RuntimeError("T4 noch nicht implementiert")


# ------------------------------------------------------------------- Anschlusscode (unten)

def mission(rob, task):
    """Wird vom Runner für T2-T4 genau einmal aufgerufen, solange rob.running().

    Fertig = normal zurückkehren (der Runner meldet dann "done"). Aufgegeben =
    Exception werfen: der Runner meldet "failed:<Grund>", was für die Bewertung
    ehrlicher ist als ein "done" ohne Zielankunft.
    """
    for name, funktion in (("quadrat", fahre_quadrat), ("korridor", fahre_korridor),
                           ("gps_anfahrt", fahre_zu_gps)):
        if task.startswith(name):
            return funktion(rob)
    raise RuntimeError(f"unbekannter Auftrag '{task}'")


if __name__ == "__main__":
    # Der Runner übernimmt: Knoten verbinden, T1 automatisch durchreichen
    # (cmd_vel -> inverse_kinematics -> wheel_speeds) oder für T2-T4 mission() aufrufen.
    robot_io.serve(sys.modules[__name__])
