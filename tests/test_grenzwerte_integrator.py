"""Limits that a finished solution can miss are not requirements — this pins the measured ones.

Every number here comes from repeated `./lab grade` runs of the reference solution on this
machine (paced, ROS-stub). The point of the test is the relation "threshold vs measured spread":
if someone tightens a limit below what a correct solution reproducibly achieves, grading becomes
a lottery, and the lab cannot have that.
"""
from mecanum_lab import tasks

AUFGABEN = tasks.load_tasks()


def grenzwert(aufgabe: str, schluessel: str):
    return [a for a in AUFGABEN["tasks"] if a["id"] == aufgabe][0][schluessel]


def test_t4_ziel_schwelle_deckt_die_gemessene_streuung():
    """GPS approach: measured 0.07 / 0.17 / 0.18 / 0.25 / 0.33 m over paced runs.

    The drive plan ends while the robot is still closing on the spot, so the stopping point
    depends on when the last command is cut off — the spread is timing, not GPS noise (default
    σ_xy for this task is 0.06 m). 0.30 m was inside that spread; the limit is 0.45 m now and
    must stay above the worst measured value plus its margin.
    """
    schlechteste_gemessene = 0.33
    assert grenzwert("gps_anfahrt", "ziel_max") >= schlechteste_gemessene + 0.1


def test_t3_ziel_schwelle_darf_scharf_bleiben():
    """The corridor task measures 0.17/0.18 m repeatedly — LIDAR is smooth, so 0.3 m is fair."""
    assert grenzwert("korridor", "ziel_max") == 0.3


def test_t4_wird_nicht_durch_blindes_drucken_bestanden():
    """Loosening the goal radius must not make the task free: contacts and 'not beforehand' stay."""
    aufgabe = [a for a in AUFGABEN["tasks"] if a["id"] == "gps_anfahrt"][0]
    assert aufgabe["kontakte_max"] == 0
    assert aufgabe["ziel"] == "spawn" and aufgabe["ziel_index"] == -1
