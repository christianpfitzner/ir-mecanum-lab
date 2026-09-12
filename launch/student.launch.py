"""Put a student node onto a simulator that is already running:

    ros2 launch launch/student.launch.py robot:=alice controller:=student/solution.py

The simulator must already run (launch/sim.launch.py or ./lab sim). The node file
resolves relative to the source tree; absolute paths work too.
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
                              description="Your robot name (one person, one robot)"),
        DeclareLaunchArgument("controller", default_value="student/controller_template.py",
                              description="File with inverse_kinematics() and mission()"),
        DeclareLaunchArgument("config", default_value="", description="Additional JSON config"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation time from /clock"),
        OpaqueFunction(function=start),
    ])
