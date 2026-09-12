"""Einen Studierendenknoten auf einen laufenden Simulator setzen:

    ros2 launch launch/student.launch.py robot:=alice controller:=student/solution.py

Der Simulator muss schon laufen (launch/sim.launch.py oder ./lab sim). Die Knoten-
Datei wird relativ zum Quellbaum geloest, absolute Pfade gehen auch.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

WURZEL = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WAHR = ("true", "1", "yes", "on")


def start(context, *args, **kwargs):
    datei = LaunchConfiguration("controller").perform(context)
    if not os.path.isabs(datei):
        datei = os.path.join(WURZEL, datei)
    robot = LaunchConfiguration("robot").perform(context)
    cmd = [sys.executable, "-m", "mecanum_lab.node", "controller",
           "--robot", robot, "--controller", datei]
    config = LaunchConfiguration("config").perform(context)
    if config:
        cmd += ["--config", config]
    umgebung = {"PYTHONPATH": os.pathsep.join([WURZEL, os.environ.get("PYTHONPATH", "")]),
                "MECANUM_USE_SIM_TIME": "1" if LaunchConfiguration(
                    "use_sim_time").perform(context).lower() in WAHR else "0"}
    return [ExecuteProcess(cmd=cmd, additional_env=umgebung, output="screen",
                           name=f"knoten_{robot}", emulate_tty=True)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot", default_value="alice",
                              description="Name deines Roboters (eine Person, ein Roboter)"),
        DeclareLaunchArgument("controller", default_value="student/controller_template.py",
                              description="Datei mit inverse_kinematics() und mission()"),
        DeclareLaunchArgument("config", default_value="", description="zusaetzliche JSON-Config"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Simulationszeit fuer /clock verwenden"),
        OpaqueFunction(function=start),
    ])
