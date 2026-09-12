"""Tests für die IMU und die GPS-Eingriffe in mecanum_lab/sensors.py (Versuch 2, Agent A).

Die Prüfgrößen sind absichtlich physikalisch formuliert (Rauschdichte, Bias, Doppel­integration),
weil die Studierenden genau diese Zahlen in den Vorfragen rechnen: stimmt das Modell nicht mit
der Doku überein, ist der Versuch nicht lösbar.
"""
import math

import pytest

from mecanum_lab.sensors import GpsSensor, ImuSensor, Noise
from mecanum_lab.types import Pose, Twist, load_config, cfg_get

DT = 1.0 / 50.0


def imu_cfg(**änderungen):
    cfg = dict(cfg_get(load_config(), "imu"))
    cfg.update(änderungen)
    return cfg


def stiche(probe, pose=None, twist=None, sekunden=1.0, dt=DT):
    """Samples für `sekunden` Sekunden abholen, als flache Liste."""
    pose, twist = pose or Pose(0, 0, 0), twist or Twist()
    raus = []
    for _ in range(round(sekunden / dt)):
        raus.extend(probe.sample(pose, twist, dt))
    return raus


# ------------------------------------------------------------------------- IMU-Grundlagen


def test_imu_liefert_gravitation_als_specific_force():
    """Im Stand zeigt az nach oben mit +g — kein Mesfehler, sondern Physik des Chips."""
    mess = stiche(ImuSensor(Noise(3), imu_cfg(vibration=0.0, tilt_sigma=0.0)), sekunden=0.5)
    assert mess, "keine IMU-Stichproben"
    az = [m.az for m in mess]
    assert 9.6 < sum(az) / len(az) < 10.0


def test_imu_stichproben_rate_entspricht_konfiguration():
    """100 Hz IMU an 50-Hz-Physik: pro Physikschritt kommen zwei Stichproben."""
    probe = ImuSensor(Noise(1), imu_cfg(rate=100.0))
    assert len(stiche(probe, sekunden=2.0)) == pytest.approx(200, abs=2)


def test_imu_weisses_rauschen_entspricht_der_dichte():
    """σ pro Stichprobe = Dichte · √(rate/2) — die Formel aus der Versuchsanleitung."""
    dichte = 0.02
    mess = stiche(ImuSensor(Noise(7), imu_cfg(rate=100.0, accel_noise=dichte,
                                             accel_bias=0.0, accel_bias_walk=0.0,
                                             accel_scale=0.0, tilt_sigma=0.0,
                                             vibration=0.0, startup=0.001,
                                             startup_bias=0.0)), sekunden=20.0)
    achse = [m.ax for m in mess]
    mittel = sum(achse) / len(achse)
    streuung = math.sqrt(sum((a - mittel) ** 2 for a in achse) / len(achse))
    assert streuung == pytest.approx(dichte * math.sqrt(50.0), rel=0.25)


def test_imu_bias_bleibt_und_ist_je_roboter_verschieden():
    """Der Start-Bias ist pro Roboter anders und verschwindet nicht über Nacht."""
    ohne_alle_anderen_quellen = imu_cfg(accel_noise=0.0, accel_bias_walk=0.0, tilt_sigma=0.0,
                                       vibration=0.0, startup=0.001, startup_bias=0.0)
    erste = stiche(ImuSensor(Noise(11), ohne_alle_anderen_quellen), sekunden=0.3)
    zweite = stiche(ImuSensor(Noise(12), ohne_alle_anderen_quellen), sekunden=0.3)
    dritte = stiche(ImuSensor(Noise(11), ohne_alle_anderen_quellen), sekunden=0.3)
    assert mitte(erste) == mitte(dritte), "gleiches Seed, anderer Bias — nicht deterministisch"
    assert mitte(erste) != mitte(zweite), "zwei Roboter, derselbe Bias — individuell anders?"
    assert abs(mitte(erste)) > 1e-4, "Bias wurde nie gezogen"
    assert len({round(m.ax, 6) for m in erste}) == 1, "Bias driftet ohne Random-Walk"


def mitte(mess):
    return sum(m.ax for m in mess) / len(mess)


def test_imu_bias_random_walk_verschiebt_den_bias():
    """Bias-Random-Walk: nach einigen Sekunden ist der gemittelte Bias ein anderer als am Anfang."""
    probe = ImuSensor(Noise(5), imu_cfg(accel_noise=0.0, accel_bias=0.0, tilt_sigma=0.0,
                                       vibration=0.0, startup=0.001, startup_bias=0.0,
                                       accel_bias_walk=0.01))
    mess = stiche(probe, sekunden=10.0)
    anfang = sum(m.ax for m in mess[:100]) / 100
    ende = sum(m.ax for m in mess[-100:]) / 100
    assert abs(ende - anfang) > 0.01, "Bias-Random-Walk war nicht zu sehen"


def test_imu_neigung_laesst_schwerkraft_in_die_ebene_sickern():
    """0,6° Neigung sind 0,1 m/s² auf ax bzw. ay — deshalb ist 'nur ax/ay messen' nicht harmlos."""
    probe = ImuSensor(Noise(9), imu_cfg(accel_noise=0.0, accel_bias=0.0, accel_bias_walk=0.0,
                                       vibration=0.0, startup=0.001, startup_bias=0.0,
                                       tilt_sigma=0.02, tilt_tau=0.5))
    mess = stiche(probe, sekunden=6.0)
    betraege = [abs(m.ax) + abs(m.ay) for m in mess]
    assert sum(betraege) / len(betraege) > 0.02, "Neigung hat die Horizontalachsen nicht erreicht"


def test_imu_drehrate_folgt_dem_Gyro_mit_bias_und_skala():
    probe = ImuSensor(Noise(4), imu_cfg(rate=100.0, gyro_noise=0.0, gyro_bias=0.0,
                                       gyro_bias_walk=0.0, gyro_scale=0.0,
                                       accel_noise=0.0, tilt_sigma=0.0, vibration=0.0,
                                       startup=0.001, startup_bias=0.0))
    mess = stiche(probe, twist=Twist(0.0, 0.0, 0.5), sekunden=1.0)
    assert [m.gz for m in mess] == [pytest.approx(0.5, abs=1e-9)] * len(mess)


def test_imu_skalenfehler_skaliert_die_drehrate():
    """2 % Skalenfehler: der Gyro meldet 0,51 rad/s statt 0,5 — Mitteln hilft nicht."""
    probe = ImuSensor(Noise(8), imu_cfg(rate=100.0, gyro_noise=0.0, gyro_bias=0.0,
                                       gyro_bias_walk=0.0, gyro_scale=0.02,
                                       accel_noise=0.0, tilt_sigma=0.0, vibration=0.0,
                                       startup=0.001, startup_bias=0.0))
    mess = stiche(probe, twist=Twist(0.0, 0.0, 0.5), sekunden=0.5)
    faktoren = {round(m.gz / 0.5, 9) for m in mess}
    assert len(faktoren) == 1, "Skalenfehler muss konstant sein, nicht rauschen"
    faktor = faktoren.pop()
    assert 0.98 <= faktor <= 1.02 and abs(faktor - 1.0) > 1e-6


def test_imu_deterministisch_bei_gleichem_seed():
    def lauf():
        return [(m.ax, m.ay, m.gz) for m in
                stiche(ImuSensor(Noise(2), imu_cfg(rate=200.0)), twist=Twist(0.4, 0.1, 0.2),
                       sekunden=1.0)]
    assert lauf() == lauf()


def test_imu_doppelintegration_eines_bias_waechst_quadratisch():
    """Kern des Versuches: 0,05 m/s² Bias -> nach 4 s gut 0,4 m, nach 8 s gut 1,6 m Wegfehler."""
    cfg = imu_cfg(accel_noise=0.0, accel_bias_walk=0.0, accel_scale=0.0, tilt_sigma=0.0,
                  vibration=0.0, startup=0.001, startup_bias=0.0, accel_bias=0.0,
                  gyro_noise=0.0, gyro_bias=0.0, gyro_bias_walk=0.0, gyro_scale=0.0)
    probe = ImuSensor(Noise(6), cfg)
    probe.b_a[0] = 0.05                                    # Bias von Hand, damit nachrechenbar
    zeit, geschwindigkeit, weg, dt = 0.0, 0.0, 0.0, 1.0 / 100.0
    for _ in range(400):
        mess = probe.sample(Pose(), Twist(), dt)[0]
        weg += geschwindigkeit * dt
        geschwindigkeit += mess.ax * dt
        zeit += dt
    assert weg == pytest.approx(0.5 * 0.05 * zeit ** 2, rel=0.05)


# ------------------------------------------------------------------- GPS: Funkloch und Bias


def gps_cfg(**änderungen):
    cfg = dict(cfg_get(load_config(), "gps"))
    cfg.update(änderungen)
    return cfg


def test_gps_luecke_unterdrueckt_fixe_relativ_zum_auftrag():
    sensor = GpsSensor(Noise(1), gps_cfg(rate=5.0, sigma_xy=0.0, gap=[10, 4]))
    sensor.t0 = 30.0                                      # Auftrag begann bei t = 30 s
    assert sensor.fix(Pose(1, 2, 0), 29.0) is not None
    assert sensor.fix(Pose(1, 2, 0), 36.0) is not None     # 6 s nach Auftragsbeginn
    for t in (40.5, 42.0, 43.5):
        assert sensor.fix(Pose(1, 2, 0), t) is None, "Funkloch verfehlt"
    assert sensor.fix(Pose(1, 2, 0), 45.5) is not None


def test_gps_bias_step_verschibt_fuer_einen_Zeitraum():
    sensor = GpsSensor(Noise(1), gps_cfg(rate=5.0, sigma_xy=0.0, bias_step=[5, 3, 2.0, -1.0]))
    vorher = sensor.fix(Pose(4, 4, 0), 4.0)
    waehrend = sensor.fix(Pose(4, 4, 0), 6.0)
    nachher = sensor.fix(Pose(4, 4, 0), 9.0)
    assert (vorher.x, vorher.y) == pytest.approx((4.0, 4.0))
    assert (waehrend.x, waehrend.y) == pytest.approx((6.0, 3.0))
    assert (nachher.x, nachher.y) == pytest.approx((4.0, 4.0))


def test_gps_unsinnige_eingriffe_schalten_ab_statt_abzustuerzen():
    assert GpsSensor(Noise(1), gps_cfg(gap=[7])).gap is None
    assert GpsSensor(Noise(1), gps_cfg(gap=[7, 0])).gap is None
    assert GpsSensor(Noise(1), gps_cfg(bias_step=[1, 2, 3])).step is None


def test_gps_fixt_wahrheit_plus_bias_und_streuung():
    sensor = GpsSensor(Noise(2), gps_cfg(sigma_xy=0.05, sigma_theta=0.0, bias_xy=[0.2, 0.0]))
    mess = [sensor.fix(Pose(3, 1, 0.5), 0.1) for _ in range(300)]
    mittig_x = sum(m.x for m in mess) / len(mess)
    assert mittig_x == pytest.approx(3.2, abs=0.02)
    assert abs(sum(m.y for m in mess) / len(mess) - 1.0) < 0.02
