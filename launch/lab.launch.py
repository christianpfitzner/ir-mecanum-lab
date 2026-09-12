"""Everything at once: simulator + your node + (optional) grader.

    ros2 launch launch/lab.launch.py robot:=alice controller:=student/solution.py \
        world:=production headless:=true seconds:=60

The grader runs inside the sim process (/--grade); its report comes at the end.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAHR = ("true", "1", "yes", "on")


def start(context, *args, **kwargs):
    hol = lambda name: LaunchConfiguration(name).perform(context)          # noqa: E731
    kopflos = hol("headless").lower() in WAHR
    pfad = {"PYTHONPATH": os.pathsep.join([WURZEL, os.environ.get("PYTHONPATH", "")]),
            "MECANUM_USE_SIM_TIME": "1" if hol("use_sim_time").lower() in WAHR else "0"}
    if kopflos:
        pfad["SDL_VIDEODRIVER"] = "dummy"
    node = lambda befehl: [sys.executable, "-m", "mecanum_lab.node", befehl]   # noqa: E731
    roboten = hol("robots") or hol("robot")        # if omitted: exactly your robot
    sim = node("sim") + ["--world", hol("world"), "--robots", roboten,
                         "--seconds", hol("seconds")]
    if kopflos:
        sim.append("--headless")
    if hol("task"):
        sim += ["--task", hol("task")]
    if hol("grade"):
        sim += ["--grade", hol("grade"), "--robot", hol("robot")]
    knoten = node("controller") + ["--robot", hol("robot"), "--controller",
                                   hol("controller") if os.path.isabs(hol("controller"))
                                   else os.path.join(WURZEL, hol("controller"))]
    return [ExecuteProcess(cmd=sim, additional_env=pfad, output="screen", name="mecanum_sim",
                           emulate_tty=True),
            ExecuteProcess(cmd=knoten, additional_env=pfad, output="screen",
                           name=f"knoten_{hol('robot')}", emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="production"),
        DeclareLaunchArgument("robot", default_value="muster"),
        DeclareLaunchArgument("robots", default_value="muster"),
        DeclareLaunchArgument("controller", default_value="student/solution.py"),
        DeclareLaunchArgument("task", default_value="", description="Empty = grader decides"),
        DeclareLaunchArgument("grade", default_value="", description="e.g. alle or kinematik"),
        DeclareLaunchArgument("seconds", default_value="0"),
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=start),
    ])
