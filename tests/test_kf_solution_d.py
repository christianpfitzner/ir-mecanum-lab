"""Tests for the experiment 2 reference solution — student/kf_solution.py (CONTRACT-KF §6/§8).

The filter equations are checked directly: import the module, call the helpers, feed a
synthetic measurement stream and see what comes out. No simulator, no ROS, no window —
deterministic over a fixed random stream.

    SDL_VIDEODRIVER=dummy MECANUM_ROS=stub python3 -m pytest tests -q

What is inside and why: F·x (prediction must compute, not guess), Q positive semidefinite
(otherwise P turns negative at some point), P symmetric (a covariance is, by definition),
NEES in the right order of magnitude (the reported 1σ must match the error — otherwise K3
is worthless) and no explosion without measurements (the GPS outage case of K2).
"""
import ast
import importlib.util
import math
import os
import random
from types import SimpleNamespace

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEG = os.path.join(ROOT, "student", "kf_solution.py")


def _loesung():
    """Import the reference solution the way robot_io.serve() loads it — as a file."""
    spec = importlib.util.spec_from_file_location("kf_solution_unter_test", WEG)
    modul = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(modul)
    return modul


kfs = _loesung()


# ---------------------------------------------------------------------- matrix math


def test_cv_praedktion_schiebt_nur_die_position():
    """F·x: position grows by v·dt, velocity stays — otherwise there is no CV model inside."""
    assert kfs.cv_matrix(0.5) == [[1, 0, 0.5, 0], [0, 1, 0, 0.5], [0, 0, 1, 0], [0, 0, 0, 1]]
    assert kfs.mv(kfs.cv_matrix(0.5), [0.0, 0.0, 2.0, 4.0]) == [1.0, 2.0, 2.0, 4.0]
    assert kfs.mv(kfs.cv_matrix(0.0), [1.0, 2.0, 3.0, 4.0]) == [1.0, 2.0, 3.0, 4.0]
    # Twice in a row is the same as one step over double the time
    zweimal = kfs.mv(kfs.cv_matrix(0.2), kfs.mv(kfs.cv_matrix(0.2), [0, 0, 1, -2]))
    assert zweimal == pytest.approx(kfs.mv(kfs.cv_matrix(0.4), [0, 0, 1, -2]))


def test_q_ist_positiv_semidefinit_und_fuer_dt_null_verschwunden():
    """Q from dt: may not create a negative variance (quadratic form ≥ 0) and must not mix
    anything in for dt = 0 — otherwise the filter adds noise where no time passed."""
    rnd = random.Random(11)
    for dt in (0.0, 0.001, 0.02, 0.2, 1.0):
        Q = kfs.q_matrix(2.5, dt)
        for _ in range(50):
            v = [rnd.uniform(-1, 1) for _ in range(4)]
            form = sum(v[i] * Q[i][j] * v[j] for i in range(4) for j in range(4))
            assert form >= -1e-12, (dt, form)
        if dt == 0.0:
            assert all(v == 0.0 for reihe in Q for v in reihe)   # dt = 0 -> Q = 0
    # Axes independent, blocks equal: q_matrix is symmetric
    Q = kfs.q_matrix(1.0, 0.1)
    assert Q == kfs.transpose(Q)
    assert Q[0][0] == Q[1][1] and Q[2][2] == Q[3][3] and Q[0][1] == 0.0
    assert Q[2][2] > 0 and Q[0][0] * Q[2][2] - Q[0][2] ** 2 > 0   # 2×2 block: PSD by Sylvester


def test_inv2_ist_wirkliche_inverse_und_bei_verletzung_demuetig():
    M = [[2.0, 0.5], [0.5, 4.0]]
    E = kfs.mul(M, kfs.inv2(M))
    assert [w for reihe in E for w in reihe] == pytest.approx([1, 0, 0, 1], abs=1e-12)
    assert kfs.inv2([[0.0, 0.0], [0.0, 0.0]]) == [[1.0, 0.0], [0.0, 1.0]]   # must not crash


# -------------------------------------------------------------------- covariance upkeep


def test_P_bleibt_symmetrisch_und_positiv_ueber_viele_updates():
    """Prediction and updates must not skew P: symmetric, diagonal > 0, small trace."""
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
    """K2 GPS outage: predict only for 40 s. The state must follow, P may grow — but must
    stay finite, otherwise the filter computes with infinity at the next fix."""
    f = kfs.KF(0.0, 0.0, 0.6, 0.0, 0.0, 0.0)
    for _ in range(2000):                                     # 40 s at 50 Hz
        f.schritt(0.02, 0.0, 0.6, 0.0, kfs.SIGMA_V ** 2)
    assert f.t == pytest.approx(40.0)
    assert f.x[0] == pytest.approx(0.6 * 40.0, abs=0.05)       # follows the measured drive
    assert all(math.isfinite(v) for v in f.x)
    sx, sy, sth = f.sigmas()
    assert 0.0 < sx < 10.0 and 0.0 < sy < 10.0 and sth < 1.0   # grows, but stays usable
    assert all(math.isfinite(w) for reihe in f.P for w in reihe)


# ------------------------------------------------------------------- consistency (NEES)


def messstrom(sigma_gps=0.5, rate_gps=5.0, sekunden=40.0, rate_odom=50.0, seed=5):
    """Synthetic drive (straight with a soft start, then an arc) through the reference solution.

    The "truth" is known, the GPS carries `sigma_gps` noise, odometry and gyro report almost
    clean values — exactly the K1/K3 setup. Returns: RMSE of the estimate, RMSE of the raw
    GPS and the mean NEES (only after 4 s of settling time, like the grader).
    """
    rnd = random.Random(seed)
    f = kfs.KF(0.3, -0.2, 0.0, 0.0, 0.05, 0.0)         # Start mit realem Anfangsfehler
    x = y = 0.0
    serie, roh, letzte = [], [], [sigma_gps, sigma_gps]   # letzte = error of the last fix
    zeit, naechster_fix, dt = 1.0 / rate_gps, 1.0 / rate_gps, 1.0 / rate_odom
    for k in range(int(sekunden * rate_odom)):
        t = (k + 1) * dt
        v = 0.45 * (1.0 - math.exp(-t / 0.4))           # motor inertia, then constant
        x = x + v * dt                                 # straight ahead, with a turn rate after 20 s
        f.schritt(dt, 0.35 if t > 20.0 else 0.0, v + rnd.gauss(0, 0.002), 0.0,
                  kfs.SIGMA_V ** 2)
        if t >= naechster_fix - 1e-9:
            naechster_fix += zeit
            gx, gy = x + rnd.gauss(0, sigma_gps), y + rnd.gauss(0, sigma_gps)
            letzte = [gx - x, gy - y]
            f.update((0, 1), [gx, gy], sigma_gps ** 2)
        if t > 4.0:                                    # only after the settling time, like the grader
            ex, ey = f.x[0] - x, f.x[1] - y
            sx, sy, _ = f.sigmas()
            serie.append((ex * ex + ey * ey, (ex * ex / sx ** 2 + ey * ey / sy ** 2) / 2.0))
            roh.append(letzte[0] ** 2 + letzte[1] ** 2)
    rmse = math.sqrt(sum(w[0] for w in serie) / len(serie))
    rmse_roh = math.sqrt(sum(roh) / len(roh))          # = sigma_xy·√2, the grader measures this way
    return rmse, rmse_roh, sum(w[1] for w in serie) / len(serie)


def test_schaetzung_ist_viel_besser_als_der_rohe_sensor():
    rmse, rmse_roh, _ = messstrom()
    assert rmse < 0.42, "K1 threshold: RMSE against truth"        # config/tasks.json: rmse_max
    assert rmse_roh / rmse >= 1.6, "K1 threshold: improvement"    # and it is clearly larger


def test_nees_ist_in_der_groessenordnung_eins():
    """The reported 1σ must match the actual error (K3: a band around 1, not around 0.01)."""
    _, _, nees = messstrom()
    assert 0.2 < nees < 5.0, f"NEES {nees:.2f} — the stated spread does not describe the error"


def test_nees_reagiert_nicht_auf_pures_skalieren_der_messung():
    """More GPS noise must grow the error but keep the NEES inside its band — otherwise the
    covariance is just a decorative number with no link to the actual error."""
    _, _, nees_klein = messstrom(sigma_gps=0.5)
    _, _, nees_gross = messstrom(sigma_gps=1.0)
    assert 0.2 < nees_gross < 5.0
    assert nees_gross == pytest.approx(nees_klein, rel=1.2)


# ------------------------------------------------------------ delayed measurements


def test_fix_aus_der_vergangenheit_wird_auf_seinen_stempel_zurueckgerechnet():
    """A fix carries fix.t — behind the state. Rewind, update, then replay forward."""
    f = kfs.KF(0.0, 0.0, 0.8, 0.0, 0.0, 0.0)
    for _ in range(20):
        f.schritt(0.02, 0.0, 0.8, 0.0, kfs.SIGMA_V ** 2)
    t_now, x_ohne = f.t, f.x[0]
    p_ohne = [reihe[:] for reihe in f.P]
    n_vorher = f.n
    t_fix = t_now - 0.10                                     # the fix is 5 steps old
    nachfahren = f.spule(t_fix)
    assert len(nachfahren) == 5 and f.t == pytest.approx(t_fix, abs=1e-9)
    f.update((0, 1), [f.x[0] - 1.0, f.x[1] + 0.5], 0.04)      # measurement 1 m to the left of it
    for dt, omega, vx_w, vy_w, rv in reversed(nachfahren):
        f.schritt(dt, omega, vx_w, vy_w, rv)
    assert f.t == pytest.approx(t_now)                        # no time travel, no loss
    assert f.x[0] < x_ohne and f.x[1] > 0.0                  # the pull goes towards the measurement
    assert f.n == n_vorher + 1 + len(nachfahren)     # one position update plus the replayed ones
    assert all(abs(f.P[i][j] - f.P[j][i]) < 1e-9 for i in range(4) for j in range(4))
    assert f.P[0][0] < p_ohne[0][0]                           # and P became smaller, not larger


# ------------------------------------------------------------- task boundary, stale bus


class _Nachbau:
    """Minimal RobotIO: hands out exactly the messages it is given, one per spin().

    Only what student/kf_solution.mission() actually touches: the three reads, task(),
    running(), spin(), config() and send_kf(). The caches deliberately behave like the real
    bus: the last message stays there until a new one replaces it.
    """

    def __init__(self, star, strom):
        self.odom_c, self.imu_c, self.gps_c = star
        self.strom, self.i, self.meldungen = strom, 0, []

    def running(self):
        return self.i < len(self.strom)

    def task(self):
        return "kf_regression"

    def spin(self, dt):
        if self.i < len(self.strom):
            setattr(self, self.strom[self.i][0] + "_c", self.strom[self.i][1])
            self.i += 1

    def odom(self):
        return self.odom_c

    def imu(self):
        return self.imu_c

    def gps(self):
        return self.gps_c

    def config(self, pfad, standard=None):
        return standard

    def send_kf(self, x, y, th, sx, sy, sth, info=None):
        self.meldungen.append((x, y, th, sx, sy, sth, info or {}))


def _botschaften(step, ab, x0, v=0.4, dt=0.02, gps=True):
    """Live odometry at 50 Hz from `ab` on, pose x = x0 + v·(t-ab), GPS fix every fifth step."""
    out = []
    for k in range(step):
        t = ab + k * dt
        x = x0 + v * (t - ab)
        out.append(("odom", SimpleNamespace(t=t, x=x, y=0.0, theta=0.0, vx=v, vy=0.0,
                                            omega=0.0, ax=0.0, ay=0.0)))
        if gps and k % 5 == 0:
            out.append(("gps", SimpleNamespace(t=t, x=x, y=0.0, theta=0.0)))
    return out


def test_neuer_auftrag_startt_nicht_mit_der_letzten_messung_des_vorigen():
    """The bus still holds the last odom of the previous drive when the new task begins.

    That message is from before the respawn: its pose is metres away and its stamp is old. The
    grader measured this as one KF task collapsing (rmse 3.5 m, NEES 332) in a run that was
    clean the time before — the node thread simply read the cache before the new data landed.
    The guard compares stamps with stamps, never with the wall clock.
    """
    alt = SimpleNamespace(t=44.9, x=3.5, y=1.0, theta=0.8, vx=0.4, vy=0.0, omega=0.0,
                          ax=0.0, ay=0.0)                       # last odom of the old drive
    rob = _Nachbau((alt, None, None), _botschaften(150, 45.0, 0.0))
    kfs.mission(rob, "kf_regression")
    assert rob.meldungen, "the node has to report something"
    x, y, _, sx, sy, _, info = rob.meldungen[0]
    assert info["t"] >= 45.0 - 1e-9, f"filter started on a stale stamp: {info}"
    assert abs(x) < 0.5 and abs(y) < 0.5, f"filter started at the old pose: {x=}, {y=}"
    assert 0.0 < sx < 2.0 and 0.0 < sy < 2.0


def test_geschlafener_knoten_rechnet_nicht_funf_sekunden_cv_modell():
    """A five second gap in the odometry is a sleeping node, not a drive to extrapolate.

    One CV step over 5 s throws the state five seconds of velocity ahead of the robot and blows
    up P; the reference solution starts a fresh filter instead. Without this the same task
    grades differently depending on how busy the machine was.
    """
    start = SimpleNamespace(t=10.0, x=4.0, y=0.0, theta=0.0, vx=0.4, vy=0.0, omega=0.0,
                            ax=0.0, ay=0.0)
    strom = _botschaften(20, 10.0, 4.0)                      # driving, then the node sleeps
    strom += _botschaften(20, 15.4, 4.16, v=0.0, gps=False)  # robot stood still, no GPS in between
    rob = _Nachbau((start, None, None), strom)
    kfs.mission(rob, "kf_regression")
    x, _, _, sx, _, _, info = rob.meldungen[-1]
    assert abs(x - 4.16) < 0.4, f"state was extrapolated across the gap: {x}"
    assert 0.0 < sx < 1.5, f"covariance did not recover from the gap: {sx} ({info})"


# ------------------------------------------------------------------------ contract


def test_filter_pfad_beruehrt_keinen_stellpfad_und_druckt_nicht():
    """CONTRACT-KF §8.1: the estimate node sends no wheels, no cmd_vel, does not look at
    truth and writes nothing to the console. A text test that watches over this."""
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
    """CONTRACT-KF §6: student/kf_solution.py 310 lines — the file is a reading model.

    The budget grew over the first draft (260) because the solution shows the consistency
    rule for K3 and the larger Q during a GPS outage — points that would be lost in K2 and
    K3 otherwise. The second increase (295 -> 310) is the task boundary: not starting on a
    stamp from the previous drive and not extrapolating a sleeping node. Both are worth more
    than the 15 lines they cost, because without them a run fails at random.
    """
    with open(WEG, encoding="utf-8") as fh:
        assert len(fh.read().splitlines()) <= 310


def test_template_und_loesung_sind_gleich_angeschlossen():
    """The template must expect nothing other than the solution: same helpers, same classes."""
    spec = importlib.util.spec_from_file_location(
        "kf_template_unter_test", os.path.join(ROOT, "student", "kf_template.py"))
    tpl = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tpl)
    for name in ("mul", "mv", "transpose", "inv2", "cv_matrix", "q_matrix", "KF", "mission"):
        assert hasattr(tpl, name) and hasattr(kfs, name), name
    assert callable(tpl.mission) and hasattr(tpl.KF(0, 0, 0, 0, 0, 0), "sigmas")
    # With the TODOs the node must still run: an update without K leaves the state unchanged …
    f = tpl.KF(1.0, 2.0, 0.5, 0.0, 0.0, 0.0)
    y = f.update((0, 1), [9.0, 9.0], 0.25)
    assert f.x == [1.0, 2.0, 0.5, 0.0] and y == [8.0, 7.0]
    assert f.sigmas() == (0.0, 0.0, 0.0)
    # … and the placeholders for F and Q break nothing: the state stays
    for _ in range(200):
        f.schritt(0.02, 0.1, 0.5, 0.0, tpl.SIGMA_V ** 2)
    assert all(math.isfinite(v) for v in f.x) and f.t == pytest.approx(4.0)
