"""A demo, chosen by argument: `ros2 launch launch/demo.launch.py demo:=wifi`

    ros2 launch launch/demo.launch.py --show-args              # the six names and what each one is
    ros2 launch launch/demo.launch.py demo:=gps_shadow robot:=alice rviz:=true

`demo_<name>.launch.py` is the same thing with the name already filled in, and both are a few lines over
`mecanum_lab/demo_launch.py`: the config comes from `config/`, the decision whether RViz starts comes from
`rviz_view`, and the processes a lab run consists of are started by exactly one file — `lab.launch.py`,
which this includes rather than copies.
"""
import os
import sys

# Appended, never in front of the ROS packages — see lab.launch.py; the repository owns a `launch/` folder.
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from mecanum_lab.demo_launch import description                        # noqa: E402


def generate_launch_description():
    """`demo:=` is an argument here, so `demos()` is what its help line lists."""
    return description()
