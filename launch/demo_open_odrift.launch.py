"""Demo: Dead reckoning in an empty hall — the settings of `config/demo_open_odrift.json`.

    ros2 launch mecanum_lab demo_open_odrift.launch.py
    ros2 launch launch/demo_open_odrift.launch.py robot:=alice          # straight from the source tree

What the window should show: no GPS for the whole run on top of the too-large radius, so the trail (`t`) shows where odometry thinks
it has been and the truth is nowhere on screen — the drift of Experiment 1 with nothing to correct it.

Arguments and their defaults: `--show-args`. Every `demo_*.launch.py` in this folder is this file with a
different name — `mecanum_lab/demo_launch.py` holds what a demo start means, and this file's own name is
what picks the config, so a launcher cannot point at a demo other than the one in its docstring.
"""
import os
import sys

# Appended, never in front of the ROS packages: this repository has a directory named `launch`, and putting
# it first would make `import launch` answer with that directory for every launch file the process reads
# afterwards. The note is in lab.launch.py; after `colcon build` the package is importable anyway.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from launch import LaunchDescription                                   # noqa: E402
from mecanum_lab.demo_launch import demo_of, description                # noqa: E402


def generate_launch_description() -> LaunchDescription:
    """`demo_of(__file__)` reads this file's name: launcher, config and docstring are one fact."""
    return description(demo_of(os.path.abspath(__file__)))
