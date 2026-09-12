"""Simulator solo starten:  ros2 launch launch/sim.launch.py [world:=track robots:=a,b]

Ohne colcon-Build: PYTHONPATH zeigt auf den Quellbaum, gestartet wird
`python3 -m mecanum_lab.node sim`. headless:=true setzt SDL_VIDEODRIVER=dummy.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAHR = ("true", "1", "yes", "on")


def start(context, *args, **kwargs):
    cmd = [sys.executable, "-m", "mecanum_lab.node", "sim",
           "--world", LaunchConfiguration("world").perform(context),
           "--robots", LaunchConfiguration("robots").perform(context),
           "--seconds", LaunchConfiguration("seconds").perform(context)]
    task = LaunchConfiguration("task").perform(context)
    config = LaunchConfiguration("config").perform(context)
    kopflos = LaunchConfiguration("headless").perform(context).lower() in WAHR
    if kopflos:
        cmd.append("--headless")
    if task:
        cmd += ["--task", task]
    if config:
        cmd += ["--config", config]
    umgebung = {"PYTHONPATH": os.pathsep.join([WURZEL, os.environ.get("PYTHONPATH", "")]),
                "MECANUM_USE_SIM_TIME": "1" if LaunchConfiguration(
                    "use_sim_time").perform(context).lower() in WAHR else "0"}
    if kopflos:
        umgebung["SDL_VIDEODRIVER"] = "dummy"                # kein X auf den CI-Rechnern
    return [ExecuteProcess(cmd=cmd, additional_env=umgebung, output="screen",
                           name="mecanum_sim", emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="maze",
                              description="maze | track | production"),
        DeclareLaunchArgument("robots", default_value="alice",
                              description="Komma-getrennte Roboternamen beim Start"),
        DeclareLaunchArgument("task", default_value="", description="Auftrag, z. B. kinematik"),
        DeclareLaunchArgument("seconds", default_value="0",
                              description="nach N Sekunden enden (0 = bis per q/Strg-C)"),
        DeclareLaunchArgument("headless", default_value="false", description="ohne Pygame-Fenster"),
        DeclareLaunchArgument("config", default_value="", description="zusaetzliche JSON-Config"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Zeitstempel in Simulationszeit (/clock)"),
        OpaqueFunction(function=start),
    ])
