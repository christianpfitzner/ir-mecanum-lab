"""The radio link as its own launch file: one AP per hall, and what a bad link does to a command.

    ros2 launch launch/wifi.launch.py                                # production, the example driver
    ros2 launch launch/wifi.launch.py wifi:=false                    # the same run without a radio
    ros2 launch launch/wifi.launch.py ap:=[10,6]                     # hang the AP over the hall
    ros2 launch launch/wifi.launch.py wall_db:=20 autonomy:=dead_reckoning
    ros2 launch launch/wifi.launch.py headless:=true seconds:=15 controller:=   # the sim alone

Everything here is the same setting you would give without ROS:

    ./lab sim --world production --set wifi.enabled=true --set wifi.ap=[10,6] \
              --set wifi.autonomy=dead_reckoning

so the launch file is a table of `--set` calls and nothing else. An argument that stays empty is not
passed on, and then `wifi.ap_by_world` of the world or `mecanum_lab/types.py` decides — the layering
is the same in both modes, which is the point of running the exercise twice.

`wifi` is the one option of the feature and it is on by default **here** (in the simulator it is off,
because the graded runs are calibrated on a link that delivers everything). Two processes start: the
simulator and — unless `controller:=` is given empty — the node in `controller`, which by default is
`student/link_autonomy_example.py`: it drives away from the access point until the link gives out.
That is the whole demonstration, and `ros2 topic echo /<robot>/link` is how to watch it from a third
terminal.
"""
import os
import sys

from launch import LaunchDescription
import launch.actions as L

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    sys.path.append(REPO)         # *behind* the ROS packages, never in front: see lab.launch.py
from mecanum_lab import rviz_view                                   # noqa: E402
TRUE = ("true", "1", "yes", "on")

# launch argument -> path in the simulator config (types.DEFAULT_CONFIG["wifi"]); empty = do not set.
# The names are the config names, not new ones: what you read in README section "Radio link" is what
# you type here, and both are what `./lab --set` takes.
SETTINGS = [
    ("ap", "wifi.ap", "access point as [x, y] in world metres (empty = the hall's own spot)"),
    ("tx_dbm", "wifi.tx_dbm", "level at the reference distance d0, in dBm"),
    ("d0", "wifi.d0", "reference distance in m (the metre tx_dbm is measured at)"),
    ("n", "wifi.n", "path-loss exponent: 2 free space, ~2.4 furnished hall, 4 concrete"),
    ("wall_db", "wifi.wall_db", "dB per wall crossing on the line AP -> robot"),
    ("floor_dbm", "wifi.floor_dbm", "q = 0 at this level (receiver sensitivity)"),
    ("good_dbm", "wifi.good_dbm", "q = 1 at this level (flat part of the rate curve)"),
    ("shadow_db", "wifi.shadow_db", "amplitude of the slow fade in dB (0 = no fade, no random draw)"),
    ("shadow_period", "wifi.shadow_period", "s between two fade targets"),
    ("latency_ms", "wifi.latency_ms", "wire delay at q = 1 in ms (three times that at q = 0)"),
    ("link_rate", "wifi.rate", "/link messages per second"),
    ("link_up_q", "wifi.link_up_q", "below this quality the countdown to autonomy runs"),
    ("link_timeout", "wifi.link_timeout", "s of low quality before the link counts as down"),
    ("autonomy", "wifi.autonomy", "stop (hold the place) | dead_reckoning (keep the last command)"),
]

BASICS = [
    ("world", "production", "hall to drive — it is the furnished one, walls are what kill the link"),
    ("robot", "alice", "your robot name (one person, one robot)"),
    ("robots", "", "robots to spawn at start (empty = only your robot)"),
    ("wifi", "true", "the one option of the feature: false = a link that delivers everything"),
    ("controller", "student/link_autonomy_example.py",
     "node to drive (empty = no node, drive it yourself with the keyboard)"),
    ("seconds", "0", "end after N seconds of simulation time (0 = until q/ctrl-C)"),
    ("headless", "false", "without the Pygame window (sets SDL_VIDEODRIVER=dummy)"),
    ("seed", "1", "noise and fade seed: same seed, same drop pattern (reproducibility)"),
    ("config", "", "extra JSON config, e.g. config/demo_wifi.json (this file's arguments win)"),
    ("rviz", "false", "start rviz2 on this robot's topics: auto | true | false "
     "(auto = only when rviz2 is installed; true without it says so and continues)"),
    ("log_level", "info", "info | debug | warning"),
    ("use_sim_time", "true", "use simulation time (/clock) for timestamps"),
]


def _path(given: str) -> str:
    """Relative to the source tree, absolute stays absolute — the file may come from anywhere."""
    return given if os.path.isabs(given) else os.path.join(REPO, given)


def setup(context, *args, **kwargs):
    """Simulator, plus the driving node unless one was asked not to start."""
    def read_arg(name):
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
    if read_arg("config"):
        sim += ["--config", _path(read_arg("config"))]
    sim += ["--set", f"wifi.enabled={'true' if read_arg('wifi').lower() in TRUE else 'false'}"]
    for name, config_path, _text in SETTINGS:
        if read_arg(name):
            sim += ["--set", f"{config_path}={read_arg(name)}"]
    if headless:
        sim.append("--headless")

    parts = [L.ExecuteProcess(cmd=sim, additional_env=env, output="screen",
                              name="mecanum_sim", emulate_tty=True)]
    if read_arg("controller"):
        node_cmd = [sys.executable, "-m", "mecanum_lab.node", "controller", "--robot", robot,
                    "--controller", _path(read_arg("controller"))]
        parts.append(L.ExecuteProcess(cmd=node_cmd, additional_env=env, output="screen",
                                      name=f"node_{robot}", emulate_tty=True))
    start_rviz, note = rviz_view.plan(read_arg("rviz"))
    if start_rviz:
        config = rviz_view.render_config(REPO, robot)
        viewer = rviz_view.command(config, sim_time=read_arg("use_sim_time").lower() in TRUE)
        parts.append(L.ExecuteProcess(cmd=viewer, additional_env=env, output="screen", name="rviz"))
    if note:
        parts.append(L.LogInfo(msg=note))
    knobs = "  ".join(f"{name}={read_arg(name)}" for name, _, _ in SETTINGS if read_arg(name))
    parts.insert(0, L.LogInfo(msg=f"[wifi] world: {read_arg('world')}  robots: "
                                  f"{read_arg('robots') or robot}  enabled: {read_arg('wifi')}  "
                                  f"driver: {read_arg('controller') or 'the keyboard'}  "
                                  f"{knobs or 'the hall defaults'}"))
    return parts


def generate_launch_description():
    arguments = [L.DeclareLaunchArgument(name, default_value=default, description=text)
                 for name, default, text in BASICS]
    arguments += [L.DeclareLaunchArgument(name, default_value="", description=text)
                  for name, _, text in SETTINGS]
    return LaunchDescription(arguments + [L.OpaqueFunction(function=setup)])
