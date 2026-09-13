"""Experiment 2 (state estimation) — configure everything important in the launch file.

    ros2 launch launch/kf.launch.py                                  # K1 with the reference solution
    ros2 launch launch/kf.launch.py task:=kf_fusion \
        controller:=student/kf_template.py gps_gap:=[14,8] gps_sigma:=1.2
    ros2 launch launch/kf.launch.py grade:=kf_alle headless:=true \
        controller:=student/kf_solution.py log:=messung.csv
    ros2 launch launch/kf.launch.py task:=kf_dynamik imu_rate:=400 \
        imu_accel_bias:=0.12 rviz:=true

Every sensor argument is the same setting you would give without ROS:
`./lab sim --set gps.sigma_xy=1.2 --set imu.rate=400` (table below in SETTINGS, also in
the experiment handout). An argument that stays empty is not set — then the test profile of
the task from config/tasks.json wins; an argument that is set always wins. `truth:=true`
(default) publishes the exact pose on /<robot>/truth: the grader needs it, without it there
is no grading.

Two processes are started: the simulator (with optional grader) and your node; on top of that
optionally `ros2 bag record` for the evaluation at home and `rviz2`.
"""
import os
import sys

from launch import LaunchDescription
import launch.actions as L
from launch.substitutions import LaunchConfiguration

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.append(REPO)         # *behind* the ROS packages, never in front: see lab.launch.py
from mecanum_lab import rviz_view                                   # noqa: E402
TRUE = ("true", "1", "yes", "on")

# launch argument -> path in the simulator config (types.DEFAULT_CONFIG); empty = do not set
SETTINGS = [
    ("gps_rate", "gps.rate", "GPS measurements per second"),
    ("gps_sigma", "gps.sigma_xy", "GPS scatter per axis in m"),
    ("gps_sigma_theta", "gps.sigma_theta", "GPS scatter of the heading in rad"),
    ("gps_bias", "gps.bias_xy", "fixed GPS offset as [dx,dy] in m"),
    ("gps_gap", "gps.gap", "GPS outage as [start, duration] in s after the task starts"),
    ("gps_bias_step", "gps.bias_step", "jumping bias [start, duration, dx, dy]"),
    ("imu_rate", "imu.rate", "IMU samples per second"),
    ("imu_gyro_noise", "imu.gyro_noise", "yaw-rate noise density rad/s/√Hz"),
    ("imu_gyro_bias", "imu.gyro_bias", "yaw-rate bias 1σ in rad/s"),
    ("imu_gyro_walk", "imu.gyro_bias_walk", "bias random walk rad/s/√s"),
    ("imu_accel_noise", "imu.accel_noise", "acceleration noise density m/s²/√Hz"),
    ("imu_accel_bias", "imu.accel_bias", "acceleration bias 1σ in m/s²"),
    ("imu_accel_walk", "imu.accel_bias_walk", "bias random walk m/s²/√s"),
    ("imu_scale", "imu.accel_scale", "relative scale error of the acceleration"),
    ("imu_tilt", "imu.tilt_sigma", "IMU tilt 1σ in rad (gravity leaks into it)"),
    ("imu_tilt_tau", "imu.tilt_tau", "correlation time of the tilt in s"),
    ("imu_vibration", "imu.vibration", "amplitude of the chassis oscillation in m/s²"),
    ("imu_vibration_hz", "imu.vibration_hz", "frequency of that oscillation in Hz"),
    ("imu_startup", "imu.startup", "settling time after switch-on in s"),
    ("imu_startup_bias", "imu.startup_bias", "extra bias while settling, in m/s²"),
    ("odom_sigma_rad", "odom.sigma_wheel", "noise per measured wheel speed"),
    ("odom_bias_omega", "odom.bias_omega", "systematic yaw-rate error rad/s"),
    ("odom_sigma_xy", "odom.sigma_xy", "measurement noise of the published odometry pose in m"),
    ("truth_rate", "truth.rate", "rate of the exact pose in Hz (used for grading)"),
    ("sim_rate", "rate", "physics steps per second"),
    ("gui_rate", "gui_rate", "GUI frames per second"),
    ("kf_rate", "kf.rate", "starting recommendation for your filter rate (hint only)"),
    ("kf_q", "kf.q_acc", "starting recommendation for your Q in m²/s³ (hint only)"),
    ("kf_q_turn", "kf.q_turn", "starting recommendation for your yaw Q (hint only)"),
    ("tf_map_to_odom", "tf.map_to_odom",
     "odom (default: map->base_link is your drifting odometry) | truth (supervisor view)"),
]

BASICS = [
    ("world", "arena", "environment — arena is the world meant for the KF tasks"),
    ("robot", "alice", "your robot name (one person, one robot)"),
    ("robots", "", "robots to spawn at start (empty = only your robot)"),
    ("controller", "student/kf_template.py", "your filter node (file relative to the source tree)"),
    ("task", "kf_gps", "kf_gps | kf_fusion | kf_kovarianz | kf_dynamik (empty = free driving)"),
    ("grade", "", "task or group to grade, e.g. kf_alle (empty = do not grade)"),
    ("seconds", "0", "end after N seconds of simulation time (0 = until q/ctrl-C)"),
    ("headless", "false", "without Pygame window (sets SDL_VIDEODRIVER=dummy)"),
    ("truth", "true", "exact pose on /<robot>/truth — needed for grading"),
    ("seed", "1", "noise seed: same seed, same measurement series (reproducibility)"),
    ("config", "", "extra JSON config (overrides config/default.json)"),
    ("log", "", "CSV measurement log, e.g. measurement.csv — tools/kfplot.py works on it"),
    ("log_interval", "0.05", "distance between log lines in s of simulation time"),
    ("recording", "", "ros2 bag name, e.g. kf_experiment (empty = no recording)"),
    ("rviz", "false", "start rviz2 on this robot's topics: auto | true | false "
     "(auto = only when rviz2 is installed; true without it says so and continues)"),
    ("log_level", "info", "info | debug | warning"),
    ("use_sim_time", "true", "use simulation time (/clock) for timestamps"),
]

# Today's name -> the German name this argument had until the English migration. The old name
# stays usable (the lab handouts of past semesters print it), but says what to type from now on.
DEPRECATED = {
    "world": "welt", "task": "aufgabe", "grade": "bewerten", "seconds": "sekunden", "truth": "wahrheit",
    "log": "protokoll", "log_interval": "protokoll_intervall",
    "recording": "aufzeichnung",
    "log_level": "log_stufe", "gps_gap": "gps_luecke", "imu_scale": "imu_skala",
    "imu_tilt": "imu_neigung", "imu_tilt_tau": "imu_neigung_tau",
    "imu_startup": "imu_einschwingen", "imu_startup_bias": "imubias_start",
    "sim_rate": "physik_rate", "kf_q_turn": "kf_q_gier",
    "tf_map_to_odom": "tf_karten_odom",
}


# defaults of the arguments declared below — needed to tell "not given" from "given"
DEFAULTS = {name: default for name, default, _text in BASICS}


def _path(given: str) -> str:
    """Relative to the source tree, absolute stays absolute — the file may come from anywhere."""
    return given if os.path.isabs(given) else os.path.join(REPO, given)


def setup(context, *args, **kwargs):
    """Build two (up to four) processes out of the arguments."""
    def read_arg(name):
        """Value of a launch argument; the German name still works, with a one-line notice.

        Both names are declared, so both are always in `launch_configurations` — what decides is
        whether the old one carries a value and the new one still holds its default. Then the old
        name wins; give both and the new one wins, which is the rule of CONTRACT §6.11.
        """
        old = DEPRECATED.get(name, "")
        if old and context.launch_configurations.get(old, ""):
            if context.launch_configurations.get(name, "") == DEFAULTS.get(name, ""):
                print(f"deprecated launch argument '{old}', use '{name}'")
                return context.launch_configurations[old]
        return context.launch_configurations.get(name, "")

    headless = read_arg("headless").lower() in TRUE
    env = {"PYTHONPATH": os.pathsep.join([REPO, os.environ.get("PYTHONPATH", "")]),
           "MECANUM_LOG": read_arg("log_level"),
           "MECANUM_USE_SIM_TIME": "1" if read_arg("use_sim_time").lower() in TRUE else "0"}
    if headless:
        env["SDL_VIDEODRIVER"] = "dummy"

    robot = read_arg("robot")
    sim = [sys.executable, "-m", "mecanum_lab.node", "sim",
           "--world", read_arg("world"), "--robots", read_arg("robots") or robot,
           "--seconds", read_arg("seconds"), "--seed", read_arg("seed")]
    if read_arg("task"):
        sim += ["--task", read_arg("task")]
    if read_arg("config"):
        sim += ["--config", _path(read_arg("config"))]
    for name, config_path, _comment in SETTINGS:
        if read_arg(name):
            sim += ["--set", f"{config_path}={read_arg(name)}"]
    if read_arg("truth").lower() in TRUE:
        sim.append("--truth")
    if read_arg("log"):
        sim += ["--log", _path(read_arg("log")), "--interval", read_arg("log_interval")]
    if read_arg("grade"):
        sim += ["--grade", read_arg("grade"), "--robot", robot]
    if headless:
        sim.append("--headless")

    controller_cmd = [sys.executable, "-m", "mecanum_lab.node", "controller",
                      "--robot", robot, "--controller", _path(read_arg("controller"))]

    parts = [L.ExecuteProcess(cmd=sim, additional_env=env, output="screen",
                              name="mecanum_sim", emulate_tty=True),
             L.ExecuteProcess(cmd=controller_cmd, additional_env=env, output="screen",
                              name=f"node_{robot}", emulate_tty=True)]
    if read_arg("recording"):
        topics = [f"/{robot}/{topic}" for topic in
                  ("odom", "gps", "imu", "kf/pose", "truth")] + ["/sim/robots", "/sim/config"]
        parts.append(L.ExecuteProcess(
            cmd=["ros2", "bag", "record", "-o", _path(read_arg("recording")), *topics],
            additional_env=env, output="screen", name="bag_record"))
    start_rviz, note = rviz_view.plan(read_arg("rviz"))
    if start_rviz:
        # one config for every launch file, written for this robot: a hand-kept .rviz with one robot's
        # name in its topics is an empty window for everyone else in the room
        config = rviz_view.render_config(REPO, robot)
        viewer = rviz_view.command(config, sim_time=read_arg("use_sim_time").lower() in TRUE)
        parts.append(L.ExecuteProcess(cmd=viewer, additional_env=env, output="screen", name="rviz"))
    if note:
        parts.append(L.LogInfo(msg=note))
    sensors = "  ".join(f"{name}={read_arg(name)}" for name, _, _ in SETTINGS if read_arg(name))
    started = f"task: {read_arg('task') or '—'} · world: {read_arg('world')}"
    parts.insert(0, L.LogInfo(msg=f"[kf] {started} · robot: {read_arg('robots') or robot}"
                                  f" · node: {read_arg('controller')}"
                                  f" · sensors: {sensors or 'test profile'}"))
    return parts


def generate_launch_description():
    arguments = [L.DeclareLaunchArgument(name, default_value=default, description=text)
                 for name, default, text in BASICS]
    arguments += [L.DeclareLaunchArgument(name, default_value="", description=text)
                  for name, _, text in SETTINGS]
    # The deprecated German names are declared too (empty = not given), so `--show-args` lists every
    # name this file accepts instead of only half of them — a student with an old handout in front of
    # them finds the line that says which spelling to use from now on.
    arguments += [L.DeclareLaunchArgument(german, default_value="",
                                         description=f"deprecated, use '{english}'")
                  for english, german in sorted(DEPRECATED.items())]
    return LaunchDescription(arguments + [L.OpaqueFunction(function=setup)])
