"""The demos, launchable: one config, the lab window, and RViz 2 when the machine has it.

    ros2 launch launch/demo.launch.py                                  # the first one, RViz if installed
    ros2 launch launch/demo.launch.py demo:=wifi rviz:=true
    ros2 launch launch/demo.launch.py demo:=gps_shadow view:=sensors   # with the raw layers visible
    ros2 launch launch/demo.launch.py --show-args

`demo:=` names a file `config/demo_<name>.json` — the same file `./lab sim --config …` takes. The names
and what each demo shows are written once, in the description of that argument, so `--show-args` is the
table and the launch file cannot describe a demo that is not in `config/` without a test noticing.

Everything that is not the choice of a demo belongs to `lab.launch.py`, and this file *includes* it
rather than repeating its process list: two copies of a list of `ExecuteProcess` lines drift apart within
a month, and it is the students who find out. `rviz:=` is forwarded, so the RViz rule — start it when it
is installed, insist when asked, say one honest line when it is not — is written once, in
`mecanum_lab/rviz_view.py`.
"""
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
# No `sys.path` manipulation here: this file imports nothing from the project — REPO is only used to name
# the file below — and the repository owns a directory called `launch`, so putting it in front of the ROS
# packages would break `import launch` for every launch file this process reads afterwards. The note (and
# the append) lives in lab.launch.py, which is the file that does import from the project.
from mecanum_lab import rviz_view                                   # noqa: E402

LAB = os.path.join(HERE, "lab.launch.py")
CONFIGS = os.path.join(REPO, "config")

# The demo list is *one* literal sentence, in the row of the argument it documents: `--show-args` prints
# that row, `tools/launchargs.py` reads this table as a literal (an expression built at import time is
# invisible to it), and `tests/test_launch_docs.py` compares the names in the sentence against
# `config/demo_*.json` in both directions. A list here and a second list there is how a documented
# argument starts lying about what a student can type.
BASICS = [
    ("demo", "gps_shadow",
     "one of: gps_shadow (GPS fixes that drift, fall silent behind the shelves and come back biased) | "
     "wifi (one access point, walls between it and the robot) | odom_error (odometry that integrates "
     "wrongly on purpose: the gap between odom and truth) | open_odrift (an empty hall, where the drift "
     "cannot be blamed on a wall) | sensor_reality (lidar noise, dropout, a biased IMU) | "
     "poi_exploration (a radiation source to find by field strength, not by distance). "
     "Launches config/demo_<name>.json — the same file ./lab sim --config takes."),
    ("world", "production", "hall to drive; the demos are authored for production"),
    ("robot", "muster", "your robot name — RViz draws the topics of this one"),
    ("robots", "", "extra robots to spawn at start (comma-separated, empty = only yours)"),
    ("controller", "student/solution.py", "your node, started beside the demo; the sim alone is "
                                         "launch/sim.launch.py"),
    ("view", "", "what the window shows: clean (default) or sensors, i.e. everything"),
    ("layers", "", "single layers over that view, e.g. scan,ghost or -hud"),
    ("rviz", "auto", "RViz 2 beside the lab window: auto = when installed, true = insist, false = no"),
    ("seconds", "0", "end after N s of simulation time (0 = until q/Ctrl-C, as in ./lab)"),
    ("headless", "false", "without the Pygame window (sets SDL_VIDEODRIVER=dummy)"),
    ("use_sim_time", "true", "use simulation time (/clock) for the timestamps"),
]


def documented_demos() -> list:
    """The names in the `demo` row's sentence — the list a student reads, as the machine sees it."""
    sentence = dict((name, (default, text)) for name, default, text in BASICS)["demo"][1]
    return [part.split(" (")[0].strip() for part in
            sentence.split("one of: ")[1].split(". Launches")[0].split(" | ")]
def available_demos() -> list:
    """The demo names that have a config file — from the directory, so this cannot go stale."""
    return sorted(name[len("demo_"):-len(".json")]
                  for name in os.listdir(CONFIGS)
                  if name.startswith("demo_") and name.endswith(".json"))


def start(context, *args, **kwargs):
    read_arg = lambda name: LaunchConfiguration(name).perform(context)          # noqa: E731
    demo = read_arg("demo")
    known = available_demos()
    if demo not in known:
        raise ValueError(f"demo:={demo} has no config/demo_{demo}.json. The demos of this repository "
                         f"are: " + ", ".join(known))
    # one line about the choice; whether RViz starts, and what stops it, is said by lab.launch.py,
    # which is the file that starts it — twice from two files is noise on the screen
    print(f"demo {demo}: config/demo_{demo}.json")
    start_rviz, _note = rviz_view.plan(read_arg("rviz"))
    forwarded = [("config", f"config/demo_{demo}.json")]
    for name, _default, _text in BASICS:
        if name == "demo":
            continue
        value = read_arg(name)
        if value:
            forwarded.append((name, value))
    return [IncludeLaunchDescription(PythonLaunchDescriptionSource(LAB), launch_arguments=forwarded)]


def generate_launch_description():
    """The arguments, plus a warning when the sentence above and config/ stop agreeing."""
    known, described = set(available_demos()), set(documented_demos())
    notes = []
    if described - known:
        notes.append(LogInfo(msg="demo.launch.py: named in the argument's text, no config file: "
                                  + ", ".join(sorted(described - known))))
    if known - described:
        notes.append(LogInfo(msg="demo.launch.py: config files that --show-args does not describe: "
                                  + ", ".join(sorted(known - described))))
    return LaunchDescription(
        notes
        + [DeclareLaunchArgument(name, default_value=default, description=text)
           for name, default, text in BASICS]
        + [OpaqueFunction(function=start)])
