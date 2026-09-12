"""Tests for the IMU and the GPS injections in mecanum_lab/sensors.py (experiment 2, agent A).

The checks are deliberately phrased in physical terms (noise density, bias, double integration),
because students compute exactly these numbers in the prelab questions: if the model disagrees
with the docs, the experiment cannot be solved.
"""
import math

import pytest

from mecanum_lab.sensors import GpsSensor, ImuSensor, Noise
from mecanum_lab.types import Pose, Twist, load_config, cfg_get

DT = 1.0 / 50.0


def imu_cfg(**changes):
    cfg = dict(cfg_get(load_config(), "imu"))
    cfg.update(changes)
    return cfg


def sampled(sample, pose=None, twist=None, seconds=1.0, dt=DT):
    """Collect the samples for `seconds` seconds as a flat list."""
    pose, twist = pose or Pose(0, 0, 0), twist or Twist()
    out = []
    for _ in range(round(seconds / dt)):
        out.extend(sample.sample(pose, twist, dt))
    return out


# --------------------------------------------------------------------------- IMU basics


def test_imu_reports_gravity_as_specific_force():
    """At rest az points up with +g - not a measurement error, but the physics of the chip."""
    samples = sampled(ImuSensor(Noise(3), imu_cfg(vibration=0.0, tilt_sigma=0.0)), seconds=0.5)
    assert samples, "no IMU samples"
    az = [m.az for m in samples]
    assert 9.6 < sum(az) / len(az) < 10.0


def test_imu_sample_rate_matches_config():
    """A 100 Hz IMU on 50 Hz physics: every physics step yields two samples."""
    sample = ImuSensor(Noise(1), imu_cfg(rate=100.0))
    assert len(sampled(sample, seconds=2.0)) == pytest.approx(200, abs=2)


def test_imu_white_noise_matches_the_density():
    """σ per sample = density · √(rate/2) — the formula from the lab handout."""
    dichte = 0.02
    samples = sampled(ImuSensor(Noise(7), imu_cfg(rate=100.0, accel_noise=dichte,
                                             accel_bias=0.0, accel_bias_walk=0.0,
                                             accel_scale=0.0, tilt_sigma=0.0,
                                             vibration=0.0, startup=0.001,
                                             startup_bias=0.0)), seconds=20.0)
    mount_key = [m.ax for m in samples]
    mittel = sum(mount_key) / len(mount_key)
    scatter = math.sqrt(sum((a - mittel) ** 2 for a in mount_key) / len(mount_key))
    assert scatter == pytest.approx(dichte * math.sqrt(50.0), rel=0.25)


def test_imu_bias_persists_and_differs_per_robot():
    """The startup bias differs per robot and does not vanish overnight."""
    quiet_cfg = imu_cfg(accel_noise=0.0, accel_bias_walk=0.0, tilt_sigma=0.0,
                                       vibration=0.0, startup=0.001, startup_bias=0.0)
    first = sampled(ImuSensor(Noise(11), quiet_cfg), seconds=0.3)
    second = sampled(ImuSensor(Noise(12), quiet_cfg), seconds=0.3)
    third = sampled(ImuSensor(Noise(11), quiet_cfg), seconds=0.3)
    assert mean_of(first) == mean_of(third), "same seed, different bias — not deterministic"
    assert mean_of(first) != mean_of(second), "two robots, the same bias — should be individual"
    assert abs(mean_of(first)) > 1e-4, "bias was never drawn"
    assert len({round(m.ax, 6) for m in first}) == 1, "bias drifts without a random walk"


def mean_of(samples):
    return sum(m.ax for m in samples) / len(samples)


def test_imu_bias_random_walk_moves_the_bias():
    """Bias random walk: after a few seconds the mean bias differs from the one at the start."""
    sample = ImuSensor(Noise(5), imu_cfg(accel_noise=0.0, accel_bias=0.0, tilt_sigma=0.0,
                                       vibration=0.0, startup=0.001, startup_bias=0.0,
                                       accel_bias_walk=0.01))
    samples = sampled(sample, seconds=10.0)
    started = sum(m.ax for m in samples[:100]) / 100
    end = sum(m.ax for m in samples[-100:]) / 100
    assert abs(end - started) > 0.01, "bias random walk not visible"


def test_imu_tilt_leaks_gravity_into_the_plane():
    """0.6 degrees of tilt is 0.1 m/s² on ax or ay — so 'only measuring ax/ay' is not harmless."""
    sample = ImuSensor(Noise(9), imu_cfg(accel_noise=0.0, accel_bias=0.0, accel_bias_walk=0.0,
                                       vibration=0.0, startup=0.001, startup_bias=0.0,
                                       tilt_sigma=0.02, tilt_tau=0.5))
    samples = sampled(sample, seconds=6.0)
    betraege = [abs(m.ax) + abs(m.ay) for m in samples]
    assert sum(betraege) / len(betraege) > 0.02, "tilt never reached the horizontal axes"


def test_imu_yaw_rate_follows_the_gyro_with_bias_and_scale():
    sample = ImuSensor(Noise(4), imu_cfg(rate=100.0, gyro_noise=0.0, gyro_bias=0.0,
                                       gyro_bias_walk=0.0, gyro_scale=0.0,
                                       accel_noise=0.0, tilt_sigma=0.0, vibration=0.0,
                                       startup=0.001, startup_bias=0.0))
    samples = sampled(sample, twist=Twist(0.0, 0.0, 0.5), seconds=1.0)
    assert [m.gz for m in samples] == [pytest.approx(0.5, abs=1e-9)] * len(samples)


def test_imu_scale_error_scales_the_yaw_rate():
    """A 2 % scale error: the gyro reports 0.51 rad/s instead of 0.5 — averaging does not help."""
    sample = ImuSensor(Noise(8), imu_cfg(rate=100.0, gyro_noise=0.0, gyro_bias=0.0,
                                       gyro_bias_walk=0.0, gyro_scale=0.02,
                                       accel_noise=0.0, tilt_sigma=0.0, vibration=0.0,
                                       startup=0.001, startup_bias=0.0))
    samples = sampled(sample, twist=Twist(0.0, 0.0, 0.5), seconds=0.5)
    faktoren = {round(m.gz / 0.5, 9) for m in samples}
    assert len(faktoren) == 1, "a scale error must stay constant, not act as noise"
    faktor = faktoren.pop()
    assert 0.98 <= faktor <= 1.02 and abs(faktor - 1.0) > 1e-6


def test_imu_is_deterministic_with_the_same_seed():
    def run():
        return [(m.ax, m.ay, m.gz) for m in
                sampled(ImuSensor(Noise(2), imu_cfg(rate=200.0)), twist=Twist(0.4, 0.1, 0.2),
                       seconds=1.0)]
    assert run() == run()


def test_double_integrating_a_bias_grows_quadratically():
    """Core of the experiment: a 0.05 m/s² bias gives about 0.4 m of path error in 4 s, 1.6 m in 8 s."""
    cfg = imu_cfg(accel_noise=0.0, accel_bias_walk=0.0, accel_scale=0.0, tilt_sigma=0.0,
                  vibration=0.0, startup=0.001, startup_bias=0.0, accel_bias=0.0,
                  gyro_noise=0.0, gyro_bias=0.0, gyro_bias_walk=0.0, gyro_scale=0.0)
    sample = ImuSensor(Noise(6), cfg)
    sample.b_a[0] = 0.05                                    # bias set by hand, so it stays recomputable
    time, velocity, path, dt = 0.0, 0.0, 0.0, 1.0 / 100.0
    for _ in range(400):
        samples = sample.sample(Pose(), Twist(), dt)[0]
        path += velocity * dt
        velocity += samples.ax * dt
        time += dt
    assert path == pytest.approx(0.5 * 0.05 * time ** 2, rel=0.05)


# ----------------------------------------------------------------------- GPS: outage and bias


def gps_cfg(**changes):
    cfg = dict(cfg_get(load_config(), "gps"))
    cfg.update(changes)
    return cfg


def test_gps_outage_suppresses_fixes_relative_to_the_task():
    sensor = GpsSensor(Noise(1), gps_cfg(rate=5.0, sigma_xy=0.0, gap=[10, 4]))
    sensor.t0 = 30.0                                      # the task started at t = 30 s
    assert sensor.fix(Pose(1, 2, 0), 29.0) is not None
    assert sensor.fix(Pose(1, 2, 0), 36.0) is not None     # 6 s after the task started
    for t in (40.5, 42.0, 43.5):
        assert sensor.fix(Pose(1, 2, 0), t) is None, "GPS outage missed"
    assert sensor.fix(Pose(1, 2, 0), 45.5) is not None


def test_gps_bias_step_shifts_for_a_window():
    sensor = GpsSensor(Noise(1), gps_cfg(rate=5.0, sigma_xy=0.0, bias_step=[5, 3, 2.0, -1.0]))
    vorher = sensor.fix(Pose(4, 4, 0), 4.0)
    waehrend = sensor.fix(Pose(4, 4, 0), 6.0)
    nachher = sensor.fix(Pose(4, 4, 0), 9.0)
    assert (vorher.x, vorher.y) == pytest.approx((4.0, 4.0))
    assert (waehrend.x, waehrend.y) == pytest.approx((6.0, 3.0))
    assert (nachher.x, nachher.y) == pytest.approx((4.0, 4.0))


def test_nonsense_gps_edits_switch_off_instead_of_crashing():
    assert GpsSensor(Noise(1), gps_cfg(gap=[7])).gap is None
    assert GpsSensor(Noise(1), gps_cfg(gap=[7, 0])).gap is None
    assert GpsSensor(Noise(1), gps_cfg(bias_step=[1, 2, 3])).step is None


def test_gps_fixes_truth_plus_bias_and_scatter():
    sensor = GpsSensor(Noise(2), gps_cfg(sigma_xy=0.05, sigma_theta=0.0, bias_xy=[0.2, 0.0]))
    samples = [sensor.fix(Pose(3, 1, 0.5), 0.1) for _ in range(300)]
    center_x = sum(m.x for m in samples) / len(samples)
    assert center_x == pytest.approx(3.2, abs=0.02)
    assert abs(sum(m.y for m in samples) / len(samples) - 1.0) < 0.02
