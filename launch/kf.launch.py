"""Experiment 2 (state estimation) — configure everything important in the launch file.

    ros2 launch launch/kf.launch.py                                  # K1 with the reference solution
    ros2 launch launch/kf.launch.py aufgabe:=kf_fusion \
        controller:=student/kf_template.py gps_luecke:=[14,8] gps_sigma:=1.2
    ros2 launch launch/kf.launch.py bewerten:=kf_alle headless:=true \
        controller:=student/kf_solution.py protokoll:=messung.csv
    ros2 launch launch/kf.launch.py aufgabe:=kf_dynamik imu_rate:=400 \
        imu_accel_bias:=0.12 rviz:=true

Every sensor argument is the same setting you would give without ROS:
`./lab sim --set gps.sigma_xy=1.2 --set imu.rate=400` (table below in EINSTELLUNGEN, also in
the experiment handout). An argument that stays empty is not set — then the test profile of
the task from config/tasks.json wins; an argument that is set always wins. `wahrheit:=true`
(default) publishes the exact pose on /<robot>/truth: the grader needs it, without it there
is no grading.

Two processes are started: the simulator (with optional grader) and your node; on top of that
optionally `ros2 bag record` for the evaluation at home and `rviz2`.
"""
import os
import sys

from launch import LaunchDescription
import launch.actions as L
from launch.substitutions import LaunchConfiguration as Halt

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAHR = ("true", "1", "yes", "on")

# launch argument -> path in the simulator config (types.DEFAULT_CONFIG); empty = do not set
EINSTELLUNGEN = [
    ("gps_rate", "gps.rate", "GPS measurements per second"),
    ("gps_sigma", "gps.sigma_xy", "GPS scatter per axis in m"),
    ("gps_sigma_theta", "gps.sigma_theta", "GPS scatter of the heading in rad"),
    ("gps_bias", "gps.bias_xy", "fixed GPS offset as [dx,dy] in m"),
    ("gps_luecke", "gps.gap", "GPS outage as [start, duration] in s after the task starts"),
    ("gps_bias_step", "gps.bias_step", "jumping bias [start, duration, dx, dy]"),
    ("imu_rate", "imu.rate", "IMU samples per second"),
    ("imu_gyro_noise", "imu.gyro_noise", "yaw-rate noise density rad/s/√Hz"),
    ("imu_gyro_bias", "imu.gyro_bias", "yaw-rate bias 1σ in rad/s"),
    ("imu_gyro_walk", "imu.gyro_bias_walk", "bias random walk rad/s/√s"),
    ("imu_accel_noise", "imu.accel_noise", "acceleration noise density m/s²/√Hz"),
    ("imu_accel_bias", "imu.accel_bias", "acceleration bias 1σ in m/s²"),
    ("imu_accel_walk", "imu.accel_bias_walk", "bias random walk m/s²/√s"),
    ("imu_skala", "imu.accel_scale", "relative scale error of the acceleration"),
    ("imu_neigung", "imu.tilt_sigma", "IMU tilt 1σ in rad (gravity leaks into it)"),
    ("imu_neigung_tau", "imu.tilt_tau", "correlation time of the tilt in s"),
    ("imu_vibration", "imu.vibration", "amplitude of the chassis oscillation in m/s²"),
    ("imu_vibration_hz", "imu.vibration_hz", "frequency of that oscillation in Hz"),
    ("imu_einschwingen", "imu.startup", "settling time after switch-on in s"),
    ("imubias_start", "imu.startup_bias", "extra bias while settling, in m/s²"),
    ("odom_sigma_rad", "odom.sigma_wheel", "noise per measured wheel speed"),
    ("odom_bias_omega", "odom.bias_omega", "systematic yaw-rate error rad/s"),
    ("odom_sigma_xy", "odom.sigma_xy", "measurement noise of the published odometry pose in m"),
    ("truth_rate", "truth.rate", "rate of the exact pose in Hz (used for grading)"),
    ("physik_rate", "rate", "physics steps per second"),
    ("gui_rate", "gui_rate", "GUI frames per second"),
    ("kf_rate", "kf.rate", "starting recommendation for your filter rate (hint only)"),
    ("kf_q", "kf.q_acc", "starting recommendation for your Q in m²/s³ (hint only)"),
    ("kf_q_gier", "kf.q_turn", "starting recommendation for your yaw Q (hint only)"),
    ("tf_karten_odom", "tf.map_to_odom",
     "odom (default: map->base_link is your drifting odometry) | truth (supervisor view)"),
]

GRUNDLEGENDES = [
    ("welt", "arena", "environment — arena is the world meant for the KF tasks"),
    ("robot", "alice", "your robot name (one person, one robot)"),
    ("robots", "", "robots to spawn at start (empty = only your robot)"),
    ("controller", "student/kf_template.py", "your filter node (file relative to the source tree)"),
    ("aufgabe", "kf_gps", "kf_gps | kf_fusion | kf_kovarianz | kf_dynamik (empty = free driving)"),
    ("bewerten", "", "task or group to grade, e.g. kf_alle (empty = do not grade)"),
    ("sekunden", "0", "end after N seconds of simulation time (0 = until q/ctrl-C)"),
    ("headless", "false", "without Pygame window (sets SDL_VIDEODRIVER=dummy)"),
    ("wahrheit", "true", "exact pose on /<robot>/truth — needed for grading"),
    ("seed", "1", "noise seed: same seed, same measurement series (reproducibility)"),
    ("config", "", "extra JSON config (overrides config/default.json)"),
    ("protokoll", "", "CSV measurement log, e.g. measurement.csv — tools/kfplot.py works on it"),
    ("protokoll_intervall", "0.05", "distance between log lines in s of simulation time"),
    ("aufzeichnung", "", "ros2 bag name, e.g. kf_experiment (empty = no recording)"),
    ("rviz", "false", "also start rviz2 with rviz/kf.rviz, if the file exists"),
    ("log_stufe", "info", "info | debug | warning"),
    ("use_sim_time", "true", "use simulation time (/clock) for timestamps"),
]


def _pfad(angabe: str) -> str:
    """Relative to the source tree, absolute stays absolute — the file may come from anywhere."""
    return angabe if os.path.isabs(angabe) else os.path.join(WURZEL, angabe)


def aufbau(context, *args, **kwargs):
    """Build two (up to four) processes out of the arguments."""
    hol = lambda name: Halt(name).perform(context)                        # noqa: E731
    kopflos = hol("headless").lower() in WAHR
    umgebung = {"PYTHONPATH": os.pathsep.join([WURZEL, os.environ.get("PYTHONPATH", "")]),
                "MECANUM_LOG": hol("log_stufe"),
                "MECANUM_USE_SIM_TIME": "1" if hol("use_sim_time").lower() in WAHR else "0"}
    if kopflos:
        umgebung["SDL_VIDEODRIVER"] = "dummy"

    sim = [sys.executable, "-m", "mecanum_lab.node", "sim",
           "--world", hol("welt"), "--robots", hol("robots") or hol("robot"),
           "--seconds", hol("sekunden"), "--seed", hol("seed")]
    if hol("aufgabe"):
        sim += ["--task", hol("aufgabe")]
    if hol("config"):
        sim += ["--config", _pfad(hol("config"))]
    for name, pfad, _beschreibung in EINSTELLUNGEN:
        if hol(name):
            sim += ["--set", f"{pfad}={hol(name)}"]
    if hol("wahrheit").lower() in WAHR:
        sim.append("--truth")
    if hol("protokoll"):
        sim += ["--log", _pfad(hol("protokoll")), "--log-intervall",
                hol("protokoll_intervall")]
    if hol("bewerten"):
        sim += ["--grade", hol("bewerten"), "--robot", hol("robot")]
    if kopflos:
        sim.append("--headless")

    knoten = [sys.executable, "-m", "mecanum_lab.node", "controller",
              "--robot", hol("robot"), "--controller", _pfad(hol("controller"))]

    teile = [L.ExecuteProcess(cmd=sim, additional_env=umgebung, output="screen",
                              name="mecanum_sim", emulate_tty=True),
             L.ExecuteProcess(cmd=knoten, additional_env=umgebung, output="screen",
                              name=f"knoten_{hol('robot')}", emulate_tty=True)]
    if hol("aufzeichnung"):
        themen = [f"/{hol('robot')}/{ende}" for ende in
                  ("odom", "gps", "imu", "kf/pose", "truth")] + ["/sim/robots", "/sim/config"]
        teile.append(L.ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", _pfad(hol("aufzeichnung")), *themen],
            additional_env=umgebung, output="screen", name="aufzeichnung"))
    rviz_config = os.path.join(WURZEL, "rviz", "kf.rviz")
    if hol("rviz").lower() in WAHR and os.path.exists(rviz_config):
        # Without use_sim_time, rviz compares its wall clock with the TF stamps of the sim
        # (seconds since start) and shows an empty map — the tree is there, just "in the past".
        befehl = ["rviz2", "-d", rviz_config]
        if hol("use_sim_time").lower() in WAHR:
            befehl += ["--ros-args", "-p", "use_sim_time:=true"]
        teile.append(L.ExecuteProcess(cmd=befehl, additional_env=umgebung, output="screen",
                                      name="rviz"))
    sensorik = "  ".join(f"{name}={hol(name)}" for name, _, _ in EINSTELLUNGEN if hol(name))
    teile.insert(0, L.LogInfo(msg=f"[kf] task: {hol('aufgabe') or '—'} · world: {hol('welt')} · "
                                  f"robot: {hol('robots') or hol('robot')} · node: "
                                  f"{hol('controller')} · sensors: {sensorik or 'test profile'}"))
    return teile


def generate_launch_description():
    argumente = [L.DeclareLaunchArgument(name, default_value=default, description=text)
                 for name, default, text in GRUNDLEGENDES]
    argumente += [L.DeclareLaunchArgument(name, default_value="", description=text)
                  for name, _, text in EINSTELLUNGEN]
    return LaunchDescription(argumente + [L.OpaqueFunction(function=aufbau)])
