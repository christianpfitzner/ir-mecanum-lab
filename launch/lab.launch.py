"""Everything at once: simulator + your node + (optional) grader.

    ros2 launch launch/lab.launch.py robot:=alice controller:=student/solution.py \
        world:=production headless:=true seconds:=60

The grader runs inside the sim process (--grade); its report comes at the end.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
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
                              "--seconds", read_arg("seconds")]
    if read_arg("task"):
        sim_cmd += ["--task", read_arg("task")]
    if read_arg("grade"):
        sim_cmd += ["--grade", read_arg("grade"), "--robot", robot]
    controller_cmd = child("controller") + ["--robot", robot, "--controller", controller]
    if headless:
        env["SDL_VIDEODRIVER"] = "dummy"                 # no X on the CI machines
        sim_cmd.append("--headless")
    run = {"additional_env": env, "output": "screen", "emulate_tty": True}
    return [ExecuteProcess(cmd=sim_cmd, name="mecanum_sim", **run),
            ExecuteProcess(cmd=controller_cmd, name=f"node_{robot}", **run)]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=text)
                              for name, default, text in BASICS]
                             + [OpaqueFunction(function=start)])
