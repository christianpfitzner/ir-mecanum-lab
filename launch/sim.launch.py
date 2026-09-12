"""Start the simulator on its own:  ros2 launch launch/sim.launch.py [world:=track robots:=a,b]

No colcon build: PYTHONPATH points at the source tree and the process started is
`python3 -m mecanum_lab.node sim`. headless:=true sets SDL_VIDEODRIVER=dummy.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TRUE = ("true", "1", "yes", "on")


def start(context, *args, **kwargs):
    headless = LaunchConfiguration("headless").perform(context).lower() in TRUE
    seconds = LaunchConfiguration("seconds").perform(context)
    alt = LaunchConfiguration("sekunden", default="").perform(context)   # germanids: legacy keys
    if alt and seconds == "0":               # the German name from the printed contract
        print("deprecated launch argument 'sekunden', use 'seconds'")
        seconds = alt
    cmd = [sys.executable, "-m", "mecanum_lab.node", "sim",
           "--world", LaunchConfiguration("world").perform(context),
           "--robots", LaunchConfiguration("robots").perform(context),
           "--seconds", seconds]
    task = LaunchConfiguration("task").perform(context)
    config = LaunchConfiguration("config").perform(context)
    if task:
        cmd += ["--task", task]
    if config:
        cmd += ["--config", config]
    env = {"PYTHONPATH": os.pathsep.join([REPO, os.environ.get("PYTHONPATH", "")]),
           "MECANUM_USE_SIM_TIME": "1" if LaunchConfiguration(
               "use_sim_time").perform(context).lower() in TRUE else "0"}
    if headless:
        env["SDL_VIDEODRIVER"] = "dummy"                # no X on the CI machines
        cmd.append("--headless")
    return [ExecuteProcess(cmd=cmd, additional_env=env, output="screen",
                           name="mecanum_sim", emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="maze",
                              description="maze | track | production"),
        DeclareLaunchArgument("robots", default_value="alice",
                              description="Comma-separated robot names to spawn at start"),
        DeclareLaunchArgument("task", default_value="", description="Task, e.g. kinematik"),
        DeclareLaunchArgument("seconds", default_value="0",
                              description="Exit after N seconds (0 = until q/Ctrl-C)"),
        DeclareLaunchArgument("sekunden", default_value="", description="Deprecated: seconds"),
        DeclareLaunchArgument("headless", default_value="false", description="No Pygame window"),
        DeclareLaunchArgument("config", default_value="", description="Additional JSON config"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Timestamps in simulation time (/clock)"),
        OpaqueFunction(function=start),
    ])
