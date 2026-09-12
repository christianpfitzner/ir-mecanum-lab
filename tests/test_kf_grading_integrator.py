"""Tests der KF-Bewertung in mecanum_lab/grade.py (Versuch 2, Integrator).

Die Physik wird hier nicht gebraucht: Der Bewerter liest ausschließlich Themen, also
spielen wir ihm truth, gps und kf/pose als-functions-of-time zu und prüfen sein Urteil.
Genau dadurch ist die Bewertung im Stub-Lauf identisch zum ROS-Betrieb.
"""
import json
import math

import pytest

from mecanum_lab.grade import Grader, fahrt_dauer, kommando_fahrt
from mecanum_lab.stub import StubBus
from mecanum_lab.types import Gps, Kf, Pose, Twist

AUFTRAG = {
    "id": "kf_test", "titel": "K9 — Prüfauftrag", "punkte": 30, "versuch": 2, "art": "kf",
    "timeout": 14.0, "einlauf": 1.0, "sensor": "gps", "schaetzung": "kf",
    "fahrt": [{"vx": 0.5, "dauer": 8.0}], "wiederhole": 1,
    "rmse_max": 0.40, "verbesserung_min": 1.5, "rate_min": 5.0, "kontakte_max": 0,
}


def fahrt_als_zeit(t: float) -> float:
    """x- Weg des Bewerter-Kommandos (vx = 0,5 m/s), für truth/gps/kf in derselben Bahn."""
    return 0.5 * t


def durchlaufe(auftrag, schatz_fehler=0.10, gps_fehler=0.70, sx=0.20, sy=0.20,
               luecke_ab=None, sekunden=14.0, dt=0.02, profil=None):
    """Bewertung gegen synthetische Themen: Wahrheit, GPS mit Fehler, Schätzung mit Fehler."""
    bus = StubBus("test")
    bew = Grader("alice", auftrag["id"], bus, {"tasks": [auftrag], "reihenfolge": [auftrag["id"]]},
                 world_info={"name": "arena", "size": [24, 16], "goal": None,
                             "spawns": [[1, 1, 0]], "walls": 4}).start()
    if profil is not None:
        bus.publish("/sim/config", json.dumps(profil))
    t = 0.0
    while not bew.tick(dt) and t < sekunden:
        t += dt
        wahr = fahrt_als_zeit(t)
        bus.publish("/alice/truth", Pose(wahr, 0.0, 0.0, t))
        bus.publish("/sim/robots", json.dumps([{"name": "alice", "index": 0, "contacts": 0,
                                                "distance": wahr, "mission": "running",
                                                "mode": "pass-through", "task": auftrag["id"]}]))
        gps_frisch = luecke_ab is None or t < luecke_ab
        if gps_frisch:
            mess = Gps(t=t, x=wahr + gps_fehler, y=0.5 * gps_fehler, theta=0.0)
            bus.publish("/alice/gps", mess)
        bus.publish("/alice/kf/pose", Kf(t=t, x=wahr + schatz_fehler, y=0.1 * schatz_fehler,
                                         theta=0.0, sx=sx, sy=sy))
    return bew.report()["tasks"][0]


# ------------------------------------------------------------------- Kommandosequenz


def test_kommandofahrt_zaehlt_und_wiederholt():
    auftrag = {"fahrt": [{"vx": 0.4, "dauer": 3.0}, {"vx": 0.0, "omega": 0.5, "dauer": 2.0}],
               "wiederhole": 2, "timeout": 40}
    assert fahrt_dauer(auftrag) == pytest.approx(10.0)
    assert kommando_fahrt(auftrag, 1.0) == pytest.approx(Twist(vx=0.4))
    assert kommando_fahrt(auftrag, 4.0).omega == pytest.approx(0.5)
    assert kommando_fahrt(auftrag, 6.0) == pytest.approx(Twist(vx=0.4))     # zweite Runde


def test_kommandofahrt_ohne_segmente_faehrt_einfach_draus_loos():
    auftrag = {"vx": 0.3, "timeout": 12.0}
    assert fahrt_dauer(auftrag) == pytest.approx(11.0)
    assert kommando_fahrt(auftrag, 3.0) == pytest.approx(Twist(vx=0.3))


def test_cov_diag_liest_auch_mehrwertige_felder_wie_unter_ros():
    """ROS liefert die Kovarianz als numpy-artiges Feld — `cov or []` ware dort ein Fehler."""
    from mecanum_lab.ros_bridge import cov_diag

    class Feld(list):
        def __bool__(self):                          # numpy verhaelt sich genauso
            raise ValueError("The truth value of an array is ambiguous")

    vier_x_vier = Feld([1, 0, 0, 0, 0, 2, 0, 0, 0, 0, 3, 0, 0, 0, 0, 4])
    assert cov_diag(vier_x_vier, n=4) == [1.0, 2.0, 3.0, 4.0]
    assert cov_diag(None) == [0.0] * 6


def test_sinuesfoermige_gierrate_ueberlagert_grundwert():
    auftrag = {"fahrt": [{"vx": 0.5, "omega": 0.2, "sinus": 0.1, "frequenz": 0.5,
                          "dauer": 6.0}]}
    bei_null = kommando_fahrt(auftrag, 0.0).omega
    bei_halber_periode = kommando_fahrt(auftrag, 1.0).omega
    assert bei_null == pytest.approx(0.2, abs=1e-9)
    assert bei_halber_periode == pytest.approx(0.2, abs=1e-6)     # sin(pi) = 0
    assert kommando_fahrt(auftrag, 0.5).omega == pytest.approx(0.3, abs=1e-6)


# ------------------------------------------------------------------------ Bewertung


def test_gute_schaetzung_bestundet_und_meldet_alle_zahlen():
    ergebnis = durchlaufe(dict(AUFTRAG), sx=0.10, sy=0.10)
    mess = ergebnis["messwerte"]
    assert ergebnis["bestanden"], ergebnis["begruendung"]
    assert mess["rmse"] == pytest.approx(0.10, abs=0.02)
    assert mess["rmse_gps"] == pytest.approx(0.78, abs=0.05)
    assert mess["verbesserung"] > 1.5 and mess["rate_hz"] >= 5.0
    assert 0.4 < mess["nees"] < 1.2, mess["nees"]      # sx=sy=0,1 bei 0,1 m Fehler -> ~0,5
    assert ergebnis["punkte"] == 30


def test_filter_der_nur_die_messung_wiederholt_fliegt_durch_verbesserung():
    ergebnis = durchlaufe(dict(AUFTRAG), schatz_fehler=0.70, sy=0.2)
    assert not ergebnis["bestanden"]
    assert "Verbesserung" in ergebnis["begruendung"]
    assert ergebnis["punkte"] == 0.0


def test_unehrliche_kovarianz_wird_an_dem_nees_erkent():
    scharf = durchlaufe(dict(AUFTRAG, nees=[0.5, 3.0]), schatz_fehler=0.10, sx=0.01, sy=0.01)
    assert not scharf["bestanden"] and "NEES" in scharf["begruendung"]
    assert scharf["messwerte"]["nees"] > 3.0


def test_gps_funkloch_ohne_fusion_fliegt():
    """Ab t = 6 s kommen keine Fixes mehr: ohne Gegenmaßnahmen wächst der Fehler."""
    auftrag = dict(AUFTRAG, timeout=14.0, luecke={"dauer_min": 2.0, "fehler_max": 0.5})
    gut = durchlaufe(auftrag, schatz_fehler=0.10, luecke_ab=6.0)
    assert gut["messwerte"]["luecke_dauer"] > 2.0, "Funkloch nicht erkannt"
    assert gut["bestanden"], gut["begruendung"]
    schlampig = durchlaufe(auftrag, schatz_fehler=0.90, luecke_ab=6.0)
    assert not schlampig["bestanden"] and "Funkloch" in schlampig["begruendung"]


def test_keine_schaetzung_ist_ein_klarer_begrundungstext():
    bus = StubBus("test")
    bew = Grader("alice", AUFTRAG["id"], bus,
                 {"tasks": [AUFTRAG], "reihenfolge": [AUFTRAG["id"]]}).start()
    t = 0.0
    while not bew.tick(0.02) and t < 14.0:
        t += 0.02
        bus.publish("/alice/truth", Pose(0.5 * t, 0.0, 0.0))
    ergebnis = bew.report()["tasks"][0]
    assert not ergebnis["bestanden"]
    assert "kf/pose" in ergebnis["begruendung"]


def test_pruefprofil_abweichung_wird_gemeldet():
    profil = {"gps": {"rate": 5.0, "sigma_xy": 0.5}}
    soll = dict(AUFTRAG, sim={"gps": {"rate": 5.0, "sigma_xy": 0.5}})
    mit_profil = durchlaufe(soll, profil=profil)
    assert mit_profil["bestanden"], mit_profil["begruendung"]
    anders = durchlaufe(soll, profil={"gps": {"rate": 20.0, "sigma_xy": 5.0}})
    assert not anders["bestanden"] and "Prüfprofil" in anders["begruendung"]


def test_wahrheit_fehlt_ist_ein_betreuerfehler_und_kein_studentenfehler():
    bus = StubBus("test")
    bew = Grader("alice", AUFTRAG["id"], bus,
                 {"tasks": [AUFTRAG], "reihenfolge": [AUFTRAG["id"]]}).start()
    for _ in range(900):
        if bew.tick(0.02):
            break
    ergebnis = bew.report()["tasks"][0]
    assert not ergebnis["bestanden"] and "stichproben" in ergebnis["messwerte"]


# ------------------------------------------------------------------ Welt zu den Aufträgen

def _welt(task, world=None):
    """Welche Halle nimmt `./lab`, wenn die Aufgabe eine Welt empfiehlt?"""
    import types
    from mecanum_lab import node
    args = types.SimpleNamespace(world=world, task=task)
    return node.cfg_get_welt({"world": "maze"}, args, {})


def_test = _welt        # Pytest sammelt nur test_*-Namen; hier ist es ein Helfer


def test_versuch_1_bekommt_production_und_versuch_2_arena():
    # Die Aufgaben sagen es in config/tasks.json; wer das ignoriert, bewertet eine
    # Quadratfahrt in der Irrgartenhalle und hält Wandkontakte für Studienleistungen.
    assert _welt("v1") == "production"
    assert _welt("kf_alle") == "arena"
    assert _welt("alle") == "production"
    assert _welt(None) == "maze"                      # ohne Auftrag gilt der Config-Standard
    assert _welt("v1", world="track") == "track"      # Angesagtes gewinnt


def test_welt_der_aufgabe_gilt_auch_ohne_auto():
    assert _welt("kf_gps", world="auto") == "arena"
    assert _welt("quadrat") == "production"


def test_simlauf_haelt_die_Roboterliste_aktuell():
    """/sim/robots ist die einzige Quelle des Bewerters für mission_state und distance.

    Die Engine stösst die Liste nur bei spawn und reset. Wer sie im Lauf nicht nachlegt,
    bewertet eine Fahrt, deren Weg für die ganze Aufgabe 0 bleibt — und merkt nicht einmal,
    dass die Mission der Studierenden längst vorbei ist.
    """
    from mecanum_lab import node, worlds
    from mecanum_lab.engine import SimEngine
    from mecanum_lab.robot_io import robot_info
    from mecanum_lab.types import DEFAULT_CONFIG

    bus = StubBus()
    eng = SimEngine(worlds.load_world("arena"), dict(DEFAULT_CONFIG), seed=5)
    eng.spawn("karl")
    node.abo(bus, eng, "karl")
    bus.pub("twist", "karl")(Twist(vx=0.5, vy=0.0, omega=0.0))
    node.simlauf(eng, bus, None, [], 0.6)
    info = robot_info(bus, "karl")
    assert info.get("distance", 0) > 0.05, info
    assert "mission" in info and "contacts" in info, info
