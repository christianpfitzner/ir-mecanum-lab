"""Demo: What a sensor says besides its number — the settings of `config/demo_sensor_reality.json`.

    ros2 launch mecanum_lab demo_sensor_reality.launch.py
    ros2 launch launch/demo_sensor_reality.launch.py robot:=alice          # straight from the source tree

What the window should show: `sensor_state` in the readout: fixes the transport dropped (`lost_gps`), how good the last one was
(`q_gps`), the IMU chip temperature its bias walks with (`temp_imu`), and the delay on each message.

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


def generate_launch_description():
    """`demo_of(__file__)` reads this file's name: launcher, config and docstring are one fact."""
    return description(demo_of(os.path.abspath(__file__)))
