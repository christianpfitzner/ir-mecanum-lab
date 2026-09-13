"""Everything at once: simulator + your node + (optional) grader + (optional) RViz.

    ros2 launch launch/lab.launch.py robot:=alice controller:=student/solution.py \
        world:=production headless:=true seconds:=60
    ros2 launch launch/lab.launch.py config:=config/demo_wifi.json rviz:=true

The grader runs inside the sim process (--grade); its report comes at the end. `config:=` is any file
under config/ — the same one `./lab sim --config …` takes — and `rviz:=auto` (the default) opens the
ROS-side view when rviz2 is installed and says one line when it is not; `demo.launch.py` is this file
with a demo config picked out and RViz meant to be on.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                           OpaqueFunction)
from launch.substitutions import LaunchConfiguration

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    # Appended, never inserted at the front: the repository has a directory named `launch`, and a path
    # entry in front of the ROS packages would make `import launch` answer with *that* one — for this
    # file and for every launch file the same process loads afterwards. Appending finds `mecanum_lab`
    # and leaves the framework's own names alone.
    sys.path.append(REPO)
from mecanum_lab import rviz_view                           # noqa: E402
TRUE = ("true", "1", "yes", "on")
# Every argument this file forwards, with its default and one line of help: `--show-args` prints one
# row per entry here, and `tools/launchargs.py` fails the build when a row would be empty.
BASICS = [
    ("world", "production", "hall to drive: production | track | maze | arena | open"),
    ("robot", "muster", "your robot name (one person, one robot)"),
    ("robots", "muster", "robots to spawn at start (comma-separated)"),
    ("controller", "student/solution.py", "your node, relative to the source tree"),
    ("task", "", "task or group announced to the students (empty = the grader decides)"),
    ("grade", "", "grade this task or group inside the simulator (empty = do not grade)"),
    ("seconds", "0", "end after N s of simulation time (0 = until q/Ctrl-C, as in ./lab)"),
    ("headless", "false", "without the Pygame window (sets SDL_VIDEODRIVER=dummy)"),
    ("use_sim_time", "true", "use simulation time (/clock) for the timestamps"),
    ("config", "", "config file under config/ (a demo: config/demo_wifi.json), empty = the defaults"),
    ("view", "clean", "what the window shows at start: clean | sensors"),
    ("layers", "", "single layers on/off over that view, e.g. scan,ghost or -hud (empty = nothing)"),
    ("rviz", "auto", "RViz 2 beside the window: auto starts it when installed, true insists, false not"),
]


def start(context, *args, **kwargs):
    read_arg = lambda name: LaunchConfiguration(name).perform(context)          # noqa: E731
    headless = read_arg("headless").lower() in TRUE
    env = {"PYTHONPATH": os.pathsep.join([REPO, os.environ.get("PYTHONPATH", "")]),
           "MECANUM_USE_SIM_TIME": "1" if read_arg("use_sim_time").lower() in TRUE else "0"}
    child = lambda command: [sys.executable, "-m", "mecanum_lab.node", command]  # noqa: E731
    robot = read_arg("robot")
    controller = os.path.join(REPO, read_arg("controller"))   # absolute paths win the join
    sim_cmd = child("sim") + ["--world", read_arg("world"),
                              "--robots", read_arg("robots") or robot,   # else: only your robot
                              "--seconds", read_arg("seconds"),
                              "--view", read_arg("view")]
    if read_arg("config"):
        sim_cmd += ["--config", read_arg("config")]
    if read_arg("layers"):
        sim_cmd += ["--layers", read_arg("layers")]
    if read_arg("task"):
        sim_cmd += ["--task", read_arg("task")]
    if read_arg("grade"):
        sim_cmd += ["--grade", read_arg("grade"), "--robot", robot]
    controller_cmd = child("controller") + ["--robot", robot, "--controller", controller]
    if headless:
        env["SDL_VIDEODRIVER"] = "dummy"                 # no X on the CI machines
        sim_cmd.append("--headless")
    run = {"additional_env": env, "output": "screen", "emulate_tty": True}
    actions = [ExecuteProcess(cmd=sim_cmd, name="mecanum_sim", **run),
               ExecuteProcess(cmd=controller_cmd, name=f"node_{robot}", **run)]
    start_rviz, note = rviz_view.plan(read_arg("rviz"))
    if start_rviz:
        config = rviz_view.render_config(REPO, robot)
        viewer = rviz_view.command(config, sim_time=read_arg("use_sim_time").lower() in TRUE)
        actions.append(ExecuteProcess(cmd=viewer, name="rviz2", **run))
    if note:
        actions.append(LogInfo(msg=note))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=text)
                              for name, default, text in BASICS]
                             + [OpaqueFunction(function=start)])
