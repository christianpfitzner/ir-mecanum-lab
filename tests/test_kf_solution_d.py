"""Tests der Musterlösung Versuch 2 — student/kf_solution.py (Agent D, CONTRACT-KF §6/§8).

Die Filtergleichungen werden direkt geprüft: Modul importieren, Helfer aufrufen, einen
synthetischen Messstrom einspeisen und schauen, was dabei herauskommt. Kein Simulator,
kein ROS, kein Fenster — deterministisch über einen festen Zufallsstrom.

    SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q

Was drin ist und warum: F·x (die Prädiktion muss rechnen, nicht raten), Q positiv
semidefinit (sonst wird P irgendwann negativ), P symmetrisch (Kovarianz ist es per Definition),
NEES in der richtigen Größenordnung (die gemeldete 1σ muss zum Fehler passen — sonst ist K3
wertlos) und kein Explodieren ohne Messungen (der Funkloch-Fall aus K2).
"""
import ast
import importlib.util
import math
import os
import random

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEG = os.path.join(ROOT, "student", "kf_solution.py")


def _loesung():
    """Die Musterlösung so importieren, wie robot_io.serve() sie lädt — als Datei."""
    spec = importlib.util.spec_from_file_location("kf_solution_unter_test", WEG)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    return modul


kfs = _loesung()


# ---------------------------------------------------------------------- Matrixrechnung


def test_cv_praedktion_schiebt_nur_die_position():
    """F·x: Position wächst um v·dt, Geschwindigkeit bleibt — sonst ist kein CV-Modell drin."""
    assert kfs.cv_matrix(0.5) == [[1, 0, 0.5, 0], [0, 1, 0, 0.5], [0, 0, 1, 0], [0, 0, 0, 1]]
    assert kfs.mv(kfs.cv_matrix(0.5), [0.0, 0.0, 2.0, 4.0]) == [1.0, 2.0, 2.0, 4.0]
    assert kfs.mv(kfs.cv_matrix(0.0), [1.0, 2.0, 3.0, 4.0]) == [1.0, 2.0, 3.0, 4.0]
    # Zweimal hintereinander ist dasselbe wie ein Schritt über die doppelte Zeit
    zweimal = kfs.mv(kfs.cv_matrix(0.2), kfs.mv(kfs.cv_matrix(0.2), [0, 0, 1, -2]))
    assert zweimal == pytest.approx(kfs.mv(kfs.cv_matrix(0.4), [0, 0, 1, -2]))


def test_q_ist_positiv_semidefinit_und_fuer_dt_null_verschwunden():
    """Q aus dt: darf keine negative Varianz erzeugen (quadratische Form ≥ 0) und muss für
    dt = 0 nichts zumischen — sonst addiert der Filter Rausch, wo keine Zeit verging."""
    rnd = random.Random(11)
    for dt in (0.0, 0.001, 0.02, 0.2, 1.0):
        Q = kfs.q_matrix(2.5, dt)
        for _ in range(50):
            v = [rnd.uniform(-1, 1) for _ in range(4)]
            form = sum(v[i] * Q[i][j] * v[j] for i in range(4) for j in range(4))
            assert form >= -1e-12, (dt, form)
        if dt == 0.0:
            assert all(v == 0.0 for reihe in Q for v in reihe)   # dt = 0 -> Q = 0
    # Achsen unabhängig, Blöcke gleich: q_matrix ist symmetrisch
    Q = kfs.q_matrix(1.0, 0.1)
    assert Q == kfs.transpose(Q)
    assert Q[0][0] == Q[1][1] and Q[2][2] == Q[3][3] and Q[0][1] == 0.0
    assert Q[2][2] > 0 and Q[0][0] * Q[2][2] - Q[0][2] ** 2 > 0   # 2×2-Block: PSD nach Sylvester


def test_inv2_ist_wirkliche_inverse_und_bei_verletzung_demuetig():
    M = [[2.0, 0.5], [0.5, 4.0]]
    E = kfs.mul(M, kfs.inv2(M))
    assert [w for reihe in E for w in reihe] == pytest.approx([1, 0, 0, 1], abs=1e-12)
    assert kfs.inv2([[0.0, 0.0], [0.0, 0.0]]) == [[1.0, 0.0], [0.0, 1.0]]   # darf nicht craschen


# ------------------------------------------------------------------------ Kovarianzpflege


def test_P_bleibt_symmetrisch_und_positiv_ueber_viele_updates():
    """Prädiktion und Updates dürfen P nicht verzerren: symmetrisch, Diagonale > 0, spurarm."""
    rnd = random.Random(3)
    f = kfs.KF(1.0, 2.0, 0.4, -0.1, 0.3, 0.0)
    for k in range(600):
        f.schritt(0.02, rnd.uniform(-0.4, 0.4), 0.4, -0.1, kfs.SIGMA_V ** 2)
        if k % 10 == 0:
            f.update((0, 1), [1.0 + rnd.gauss(0, 0.5), 2.0 + rnd.gauss(0, 0.5)], 0.25)
        assert all(abs(f.P[i][j] - f.P[j][i]) < 1e-9 for i in range(4) for j in range(4)), k
        assert all(f.P[i][i] > 0.0 for i in range(4)), k
        sx, sy, sth = f.sigmas()
        assert math.isfinite(sx) and sx > 0.0 and math.isfinite(sy) and sy > 0.0 and sth >= 0.0


def test_ohne_einzige_messung_eksplodiert_nichts():
    """K2-Funkloch: 40 s nur prädizieren. Der Zustand muss folgen, P darf wachsen — aber
    endlich bleiben, sonst rechnet der Filter beim nächsten Fix mit Unendlich."""
    f = kfs.KF(0.0, 0.0, 0.6, 0.0, 0.0, 0.0)
    for _ in range(2000):                                     # 40 s bei 50 Hz
        f.schritt(0.02, 0.0, 0.6, 0.0, kfs.SIGMA_V ** 2)
    assert f.t == pytest.approx(40.0)
    assert f.x[0] == pytest.approx(0.6 * 40.0, abs=0.05)       # folgt der gemessenen Fahrt
    assert all(math.isfinite(v) for v in f.x)
    sx, sy, sth = f.sigmas()
    assert 0.0 < sx < 10.0 and 0.0 < sy < 10.0 and sth < 1.0   # wächst, aber bleibt brauchbar
    assert all(math.isfinite(w) for reihe in f.P for w in reihe)


# ------------------------------------------------------------------- Konsistenz (NEES)


def messstrom(sigma_gps=0.5, rate_gps=5.0, sekunden=40.0, rate_odom=50.0, seed=5):
    """Synthetische Fahrt (Gerade mit Sanftanlauf, dann Bogen) durch die Musterlösung.

    Die "Wahrheit" ist bekannt, das GPS kommt mit `sigma_gps` Rauschen, Odometrie und Gyro
    melden beinahe saubere Werte — genau die Konstellation von K1/K3. Rückgabe: RMSE der
    Schätzung, RMSE des rohen GPS und mittleres NEES (nur nach 4 s Einlauf, wie der Bewerter).
    """
    rnd = random.Random(seed)
    f = kfs.KF(0.3, -0.2, 0.0, 0.0, 0.05, 0.0)         # Start mit realem Anfangsfehler
    x = y = 0.0
    serie, roh, letzte = [], [], [sigma_gps, sigma_gps]   # letzte = Fehler des letzten Fixes
    zeit, naechster_fix, dt = 1.0 / rate_gps, 1.0 / rate_gps, 1.0 / rate_odom
    for k in range(int(sekunden * rate_odom)):
        t = (k + 1) * dt
        v = 0.45 * (1.0 - math.exp(-t / 0.4))           # Motor-Trägheit, dann konstant
        x = x + v * dt                                 # geradeaus, ab 20 s mit Drehrate
        f.schritt(dt, 0.35 if t > 20.0 else 0.0, v + rnd.gauss(0, 0.002), 0.0,
                  kfs.SIGMA_V ** 2)
        if t >= naechster_fix - 1e-9:
            naechster_fix += zeit
            gx, gy = x + rnd.gauss(0, sigma_gps), y + rnd.gauss(0, sigma_gps)
            letzte = [gx - x, gy - y]
            f.update((0, 1), [gx, gy], sigma_gps ** 2)
        if t > 4.0:                                    # erst nach dem Einlauf, wie der Bewerter
            ex, ey = f.x[0] - x, f.x[1] - y
            sx, sy, _ = f.sigmas()
            serie.append((ex * ex + ey * ey, (ex * ex / sx ** 2 + ey * ey / sy ** 2) / 2.0))
            roh.append(letzte[0] ** 2 + letzte[1] ** 2)
    rmse = math.sqrt(sum(w[0] for w in serie) / len(serie))
    rmse_roh = math.sqrt(sum(roh) / len(roh))          # = sigma_xy·√2, der Bewerter misst so
    return rmse, rmse_roh, sum(w[1] for w in serie) / len(serie)


def test_schaetzung_ist_viel_besser_als_der_rohe_sensor():
    rmse, rmse_roh, _ = messstrom()
    assert rmse < 0.42, "K1-Schwelle: RMSE gegen truth"          # config/tasks.json: rmse_max
    assert rmse_roh / rmse >= 1.6, "K1-Schwelle: Verbesserung"   # und die ist deutlich größer


def test_nees_ist_in_der_groessenordnung_eins():
    """Die gemeldete 1σ muss zum tatsächlichen Fehler passen (K3: Band um 1, nicht 0,01)."""
    _, _, nees = messstrom()
    assert 0.2 < nees < 5.0, f"NEES {nees:.2f} — angegebene Streuung beschreibt den Fehler nicht"


def test_nees_reagiert_nicht_auf_pures_skalieren_der_messung():
    """Mehr GPS-Rauschen muss den Fehler vergrößern, aber die NEES in der Bandbreite halten —
    sonst ist die Kovarianz nur ein Dekorationswert, der mit dem Fehler nichts zu tun hat."""
    _, _, nees_klein = messstrom(sigma_gps=0.5)
    _, _, nees_gross = messstrom(sigma_gps=1.0)
    assert 0.2 < nees_gross < 5.0
    assert nees_gross == pytest.approx(nees_klein, rel=1.2)


# ------------------------------------------------------------- verspaete Messungen


def test_fix_aus_der_vergangenheit_wird_auf_seinen_stempel_zurueckgerechnet():
    """Ein Fix trägt fix.t — der liegt hinter dem Zustand. Zurückspulen, updaten, nachfahren."""
    f = kfs.KF(0.0, 0.0, 0.8, 0.0, 0.0, 0.0)
    for _ in range(20):
        f.schritt(0.02, 0.0, 0.8, 0.0, kfs.SIGMA_V ** 2)
    t_now, x_ohne = f.t, f.x[0]
    p_ohne = [reihe[:] for reihe in f.P]
    n_vorher = f.n
    t_fix = t_now - 0.10                                     # der Fix ist 5 Schritte alt
    nachfahren = f.spule(t_fix)
    assert len(nachfahren) == 5 and f.t == pytest.approx(t_fix, abs=1e-9)
    f.update((0, 1), [f.x[0] - 1.0, f.x[1] + 0.5], 0.04)      # Messung 1 m links neben ihr
    for dt, omega, vx_w, vy_w, rv in reversed(nachfahren):
        f.schritt(dt, omega, vx_w, vy_w, rv)
    assert f.t == pytest.approx(t_now)                        # keine Zeitreise, kein Verlust
    assert f.x[0] < x_ohne and f.x[1] > 0.0                  # der Zug geht zur Messung hin
    assert f.n == n_vorher + 1 + len(nachfahren)     # ein Positions- plus die nachgefahrenen
    assert all(abs(f.P[i][j] - f.P[j][i]) < 1e-9 for i in range(4) for j in range(4))
    assert f.P[0][0] < p_ohne[0][0]                           # und P wurde kleiner, nicht größer


# ------------------------------------------------------------------ Vertragliches


def test_filter_pfad_beruehrt_keinen_stellpfad_und_druckt_nicht():
    """CONTRACT-KF §8.1: der Schätzknoten sendet keine Räder, kein cmd_vel, schaut nicht in
    die Wahrheit und schreibt nichts auf die Konsole. Ein Texttest, der das überwacht."""
    namen = set()
    for knoten in ast.walk(ast.parse(open(WEG, encoding="utf-8").read())):
        name = (getattr(knoten, "attr", None) or getattr(knoten, "id", None)
                or getattr(knoten, "name", None))
        if name:
            namen.add(name)
        if isinstance(knoten, ast.ImportFrom):
            namen.add(knoten.module or "")
    for verboten in ("inverse_kinematics", "send_wheels", "publish_cmd_vel", "truth", "print",
                     "numpy", "scipy", "yaml"):
        assert verboten not in namen, verboten
    assert "send_kf" in namen and "mission" in namen


def test_musterloesung_bleibt_im_zeilenbudget():
    """CONTRACT-KF §6: student/kf_solution.py 295 Zeilen — die Datei ist Lesevorbild.

    Der Rahmen ist gegenueber dem ersten Entwurf (260) gewachsen, weil die Loesung die
    Konsistenzregel fuer K3 und das groessere Q im Funkloch zeigt — beides Punkte, die in
    K2 und K3 sonst verloren gehen.
    """
    with open(WEG, encoding="utf-8") as fh:
        assert len(fh.read().splitlines()) <= 295


def test_template_und_loesung_sind_gleich_angeschlossen():
    """Das Template darf nichts anderes erwarten als die Lösung: gleiche Helfer, gleiche Klassen."""
    spec = importlib.util.spec_from_file_location(
        "kf_template_unter_test", os.path.join(ROOT, "student", "kf_template.py"))
    tpl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tpl)
    for name in ("mul", "mv", "transpose", "inv2", "cv_matrix", "q_matrix", "KF", "mission"):
        assert hasattr(tpl, name) and hasattr(kfs, name), name
    assert callable(tpl.mission) and hasattr(tpl.KF(0, 0, 0, 0, 0, 0), "sigmas")
    # Mit den TODOs muss der Knoten durchlaufen: Update ohne K verändert den Zustand nicht …
    f = tpl.KF(1.0, 2.0, 0.5, 0.0, 0.0, 0.0)
    y = f.update((0, 1), [9.0, 9.0], 0.25)
    assert f.x == [1.0, 2.0, 0.5, 0.0] and y == [8.0, 7.0]
    assert f.sigmas() == (0.0, 0.0, 0.0)
    # … und die Platzhalter für F und Q machen nichts kaputt: Zustand bleibt
    for _ in range(200):
        f.schritt(0.02, 0.1, 0.5, 0.0, tpl.SIGMA_V ** 2)
    assert all(math.isfinite(v) for v in f.x) and f.t == pytest.approx(4.0)
