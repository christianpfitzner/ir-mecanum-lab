"""Tests fuer mecanum_lab/physics.py — Kinematik, Motor, Kollision (Agent A).

Die Vorzeichen-Tests sind der Kern von Versuch 1: sie pruefen exakt die Konvention
aus CONTRACT §5 (y nach links, theta CCW), weil die Studierenden genau diese
Zuordnung in ihrer inverse_kinematics selbst schreiben mussen.
"""
import math
import random

import pytest

from mecanum_lab.physics import Chassis, Geometry, forward_kinematics, make_geometry
from mecanum_lab.physics import inverse_kinematics
from mecanum_lab.types import Pose, Rect, load_config, cfg_get

DT = 1.0 / 50.0


def geom(**kw):
    return Geometry(**kw)


def fahrt(chassis, sekunden, walls=()):
    for _ in range(round(sekunden / DT)):
        chassis.step(DT, walls)
    return chassis


# ----------------------------------------------------------------------- Kinematik


def test_fk_ik_umkehrung_a():
    """fk(ik(v)) ist exakt die Identitaet — sonst ist die Uebungsaufgabe bose."""
    g, rnd = geom(), random.Random(4)
    for _ in range(200):
        vx, vy, om = (rnd.uniform(-2, 2) for _ in range(3))
        got = forward_kinematics(g, inverse_kinematics(g, vx, vy, om))
        assert got == (pytest.approx(vx), pytest.approx(vy), pytest.approx(om))


def test_ik_vorzeichen_einzelachsen_a():
    g = geom()
    vor, seit, dreh = (inverse_kinematics(g, 1, 0, 0), inverse_kinematics(g, 0, 1, 0),
                       inverse_kinematics(g, 0, 0, 1))
    assert vor == [pytest.approx(v) for v in vor] and len(set([round(v, 9) for v in vor])) == 1
    assert vor[0] > 0                                        # vorwaerts: alle Rader positiv
    assert seit[1] > 0 > seit[0]                             # links: VL rueckwaerts, VR vor
    assert dreh[0] < 0 < dreh[1] and dreh[2] < 0 < dreh[3]    # CCW: linke Seite rueckwaerts
    assert inverse_kinematics(g, 0, 0, 0) == [0.0] * 4


def test_fk_einzelrad_a():
    """Nur VL laeuft: vorwaerts, nach rechts (-y) und im Uhrzeigersinn."""
    vx, vy, om = forward_kinematics(geom(), [1.0, 0.0, 0.0, 0.0])
    assert vx > 0 and vy < 0 and om < 0
    assert vx == pytest.approx(0.05 / 4) and vy == pytest.approx(-0.05 / 4)


def test_vorzeichen_im_fahrbetrieb_a():
    """vy > 0 bewegt in +y, omega > 0 erhoht theta — uber die komplette Kette."""
    g = geom()
    gerade = fahrt(Chassis(g, Pose(3, 3, 0.6), seed=1), 2.0)
    gerade.set_wheels(inverse_kinematics(g, 0, 0.3, 0))
    fahrt(gerade, 2.0)
    assert gerade.pose.y > 3.3 and abs(gerade.pose.theta - 0.6) < 0.02
    dreher = Chassis(g, Pose(5, 5, 0.0), seed=1)
    dreher.set_wheels(inverse_kinematics(g, 0, 0, 0.8))
    fahrt(dreher, 1.5)
    assert dreher.pose.theta > 1.0                           # CCW positiv
    assert abs(dreher.pose.x - 5) < 0.02 and abs(dreher.pose.y - 5) < 0.02
    umgekehrt = Chassis(g, Pose(5, 5, 0.0), seed=1)
    umgekehrt.set_wheels([4, -4, 4, -4])                     # [VL,VR,HL,HR] -> Uhrzeigersinn
    fahrt(umgekehrt, 1.0)
    assert umgekehrt.pose.theta < -0.5


def test_geradeaus_ohne_abdrift_a():
    ch = Chassis(geom(), Pose(1, 1, 0.0), seed=0)
    ch.set_wheels([6, 6, 6, 6])
    fahrt(ch, 2.0)
    assert ch.pose.x > 1.5 and ch.pose.y == pytest.approx(1.0, abs=1e-12)
    assert ch.pose.theta == pytest.approx(0.0, abs=1e-12)


# ---------------------------------------------------------------------------- Motor


def test_anfahrrampe_ueber_tau_a():
    g = geom(tau=0.06, max_accel=40.0, max_speed=12.0)
    ch = Chassis(g, Pose(), seed=0)
    ch.set_wheels([12] * 4)
    ch.step(DT, [])
    assert 0 < ch.wheels[0] < 1.2                            # ein Schritt = max_accel*dt
    for _ in range(2):                                       # eine tau-Zeit spaeter
        ch.step(DT, [])
    assert ch.wheels[0] < 0.4 * 12
    fahrt(ch, 1.0)
    assert ch.wheels == [pytest.approx(12.0, abs=1e-3)] * 4   # ausgesteuert (1. Ordnung naehert asymptotisch)


def test_saettigung_max_speed_a():
    g = geom(max_speed=12.0)
    ch = Chassis(g, Pose(), seed=0)
    ch.set_wheels([-1e6, 1e6, -1e6, 1e6])                    # das Muster fuer omega > 0
    assert ch.cmd == [-12.0, 12.0, -12.0, 12.0]
    fahrt(ch, 2.0)
    assert max(abs(w) for w in ch.wheels) <= 12.0 + 1e-9
    assert ch.twist.omega == pytest.approx(12.0 * g.r / g.arm, abs=1e-9)


# ------------------------------------------------------------------------ Kollision


def test_kollision_zaehlt_und_stoppt_a():
    wand = [Rect(1.0, -1.0, 3.0, 1.0)]
    ch = Chassis(geom(footprint_r=0.21), Pose(0.2, 0.0, 0.0), seed=0)
    ch.set_wheels([8, 8, 8, 8])
    fahrt(ch, 3.0, wand)
    assert ch.contacts == 1                                  # nur ein Ereignis, nicht pro Schritt
    assert ch.pose.x + 0.21 <= 1.0 + 1e-6                     # bleibt auessen vor der Wand
    assert ch.wheels == [0.0] * 4
    assert ch.twist.vx == pytest.approx(0.0, abs=1e-9)


def test_kollision_seitlich_und_ecke_a():
    wand = [Rect(0.0, 0.0, 1.0, 1.0)]
    ch = Chassis(geom(footprint_r=0.21), Pose(0.5, 0.5, 0.0), seed=0)   # Mittelpunkt im Recht-
    ch.step(DT, wand)                                        # eck: kleinste Eindringtiefe
    assert ch.contacts == 1 and ch.pose.x == pytest.approx(-0.21) and ch.pose.y == 0.5
    ch2 = Chassis(geom(footprint_r=0.21), Pose(1.1, 1.1, 0.0), seed=0)  # Ecke
    ch2.set_wheels([5, 5, 5, 5])
    fahrt(ch2, 0.5, wand)
    assert ch2.contacts >= 1
    assert ch2.pose.x >= 1.0 and ch2.pose.y >= 1.0


def test_determinismus_a():
    g = geom()
    ergebnis = []
    for _ in range(2):
        ch = Chassis(g, Pose(1, 1, 0.3), seed=7)
        for i in range(200):
            ch.set_wheels(inverse_kinematics(g, 0.4, 0.1 * math.sin(i / 20), 0.3))
            ch.step(DT, [Rect(3, 3, 4, 4)])
        ergebnis.append((ch.pose.x, ch.pose.y, ch.pose.theta, ch.contacts))
    assert ergebnis[0] == ergebnis[1]


# ------------------------------------------------------------------- Geometrie-Konfig


def test_make_geometry_mit_varianten_a():
    cfg = cfg_get(load_config(), "robot")
    assert make_geometry(cfg, "stock").max_speed == cfg["max_speed"]
    assert make_geometry(cfg, "").name == "stock"
    assert make_geometry(cfg, "slow").max_speed == 8.0
    assert make_geometry(cfg, "slow").tau == 0.12
    assert make_geometry(cfg, "fast").max_speed == 16.0
    assert make_geometry(cfg, "agile").max_accel == 80.0
    assert make_geometry(cfg, "unbekannt").max_speed == cfg["max_speed"]
    assert make_geometry(cfg, "fast").arm == pytest.approx(cfg["lx"] + cfg["ly"])
    assert make_geometry({}, "stock").r == pytest.approx(0.05)      # leere Konfiguration
