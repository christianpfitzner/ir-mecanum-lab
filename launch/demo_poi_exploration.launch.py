"""Demo: An empty hall and a source to find — the settings of `config/demo_poi_exploration.json`.

    ros2 launch mecanum_lab demo_poi_exploration.launch.py
    ros2 launch launch/demo_poi_exploration.launch.py robot:=alice          # straight from the source tree

What the window should show: the `poi` panel (`p`) and a robot that hunts one number: it climbs while the reading rises, arcs when it
stops rising, and drives back to where it was loudest when the field goes silent.

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
