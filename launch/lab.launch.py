"""Everything at once: simulator + your node + (optional) grader + (optional) RViz.

    ros2 launch mecanum_lab lab.launch.py                                     # the window, keys drive
    ros2 launch launch/lab.launch.py controller:=student/solution.py          # a node drives
    ros2 launch launch/lab.launch.py robot:=alice config:=config/demo_wifi.json

`config:=` is any file under config/, the same one `./lab sim --config …` takes; `rviz:=auto` (the
default) opens the ROS-side view when rviz2 is installed and says one line when it is not;
`controller:=<file>` hands the wheel from the keyboard to a node. Grading is asked for with `grade:=`,
but the number that goes on a sheet comes from `./lab grade` — see the note under "Two things a grade
is not" in `docs/CONTRACT.md` §9.2, and the same section for why a graded run announces its task.

**An argument that is empty was not typed, and is not passed on.** That is the whole argument table
below: defaults belong to the simulator's config layers (`docs/CONTRACT.md` §2), not to a launch file.
A file that hands over its own `world:=production` for a hall nobody asked for outbids the
`"world": "open"` of a demo config, and the reader of `ros2 launch mecanum_lab demo_poi.launch.py`
stands in a hall full of tables while the page beside it promises an empty one. Say nothing and the
config decides; say something and it wins — the rule of `launch/kf.launch.py`, applied here too.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if REPO not in sys.path:
    # Appended, never inserted at the front: the repository has a directory named `launch`, and a path
    # entry in front of the ROS packages would make `import launch` answer with *that* one — for this
    # file and for every launch file the same process loads afterwards. Appending finds `mecanum_lab`
    # and leaves the framework's own names alone.
    sys.path.append(REPO)
from mecanum_lab import rviz_view                           # noqa: E402
TRUE = ("true", "1", "yes", "on")
# Every argument, its default and one line of help: `--show-args` prints a row per entry, and
# `tools/launchargs.py` fails the build when a row would be empty. Empty default = "not typed", which
# is a value of its own here — see the module docstring.
BASICS = [
    ("world", "", "hall to drive: production | track | maze | open (empty = the config file's hall)"),
    ("robot", "muster", "your robot name (one person, one robot)"),
    ("robots", "", "robots to spawn at start, comma-separated (empty = only yours)"),
    ("controller", "", "your node (empty = the keyboard alone drives)"),   # empty is the point
    ("task", "", "task or group announced to the students (empty = the grader decides, see grade:=)"),
    ("grade", "", "grade inside the sim process (empty = do not grade) — not the ./lab grade "
                  "measurement, see the note below and CONTRACT §9"),
    ("seconds", "", "end after N s of simulation time (empty = until q or Ctrl-C, as in ./lab)"),
    ("headless", "false", "without the Pygame window (sets SDL_VIDEODRIVER=dummy)"),
    ("use_sim_time", "true", "use simulation time (/clock) for the timestamps"),
    ("config", "", "config file under config/ (a demo: config/demo_wifi.json), empty = the defaults"),
    ("view", "", "what the window shows at start: clean | sensors (empty = what the config says)"),
    ("layers", "", "single layers on/off over that view, e.g. scan,ghost or -hud (empty = nothing)"),
    ("rviz", "auto", "RViz 2 beside the window: auto starts it when installed, true insists, false not"),
    ("truth", "false", "publish the exact pose on /<robot>/truth (the ghost to compare an estimate with)"),
    ("log", "", "measurement log as CSV, e.g. messung.csv — tools/kfplot.py reads it (empty = no log)"),
    ("json", "", "grading report as JSON, e.g. bericht.json (empty = the report on screen only)"),
    ("seed", "", "noise seed: the same seed, the same measurement series (empty = the simulator's 1)"),
    ("tf_tree", "", "tf tree to publish: sim (whole tree, map is the hall) | slam (a mapper publishes "
                    "map -> odom, hall coordinates are frame `hall`) — empty = sim"),
    ("lidar_no_echo", "", "a missing echo as: range_max (default, what the laboratories measure) | inf "
                         "(what a mapper reads as \"open beyond\") — empty = range_max"),
]

# launch argument -> the option of `mecanum_lab.node` it stands for. Everything here is passed only when
# it carries a value, so one table replaces nine `if` blocks and the empty-means-not-typed rule holds for
# every one of them instead of being decided per argument.
PASSTHROUGH = (("world", "--world"), ("config", "--config"), ("seconds", "--seconds"),
               ("view", "--view"), ("layers", "--layers"), ("task", "--task"),
               ("log", "--log"), ("json", "--json"), ("seed", "--seed"))


def path_of(given: str) -> str:
    """Relative to the source tree, absolute stays absolute — `ros2 launch` is typed from anywhere.

    After `colcon build` there is no working directory that holds `config/`, which is why the demo
    launcher passes an absolute path (see `mecanum_lab/demo_launch.py`); a path typed by a student in
    their own clone is relative and has to mean the same thing.
    """
    return given if os.path.isabs(given) else os.path.join(REPO, given)


def start(context, *args, **kwargs):
    read_arg = lambda name: LaunchConfiguration(name).perform(context)          # noqa: E731
    headless = read_arg("headless").lower() in TRUE
    env = {"PYTHONPATH": os.pathsep.join([REPO, os.environ.get("PYTHONPATH", "")]),
           "MECANUM_USE_SIM_TIME": "1" if read_arg("use_sim_time").lower() in TRUE else "0"}
    child = lambda command: [sys.executable, "-m", "mecanum_lab.node", command]  # noqa: E731
    robot = read_arg("robot")
    controller = path_of(read_arg("controller")) if read_arg("controller") else ""
    sim_cmd = child("sim") + ["--robots", read_arg("robots") or robot]   # empty: only your robot
    for argument, option in PASSTHROUGH:
        given = read_arg(argument)
        if given:
            sim_cmd += [option, path_of(given) if argument == "config" else given]
    if read_arg("truth").lower() in TRUE:
        sim_cmd.append("--truth")           # a flag of its own, so it is not in the table
    if read_arg("tf_tree"):
        sim_cmd += ["--set", f"tf.tree={read_arg('tf_tree')}"]      # a config path, likewise
    if read_arg("lidar_no_echo"):
        sim_cmd += ["--set", f"sensor.lidar.no_echo={read_arg('lidar_no_echo')}"]
    if read_arg("grade"):
        sim_cmd += ["--grade", read_arg("grade"), "--robot", robot]
    controller_cmd = child("controller") + ["--robot", robot, "--controller", controller]
    if headless:
        env["SDL_VIDEODRIVER"] = "dummy"                 # no X on the CI machines
        sim_cmd.append("--headless")
    run = {"additional_env": env, "output": "screen", "emulate_tty": True}
    actions = [ExecuteProcess(cmd=sim_cmd, name="mecanum_sim", **run)]
    if controller:
        # Two publishers: the node every tick, the keys only while held. Said before the first confusion.
        actions.append(LogInfo(msg=f"[lab] '{read_arg('controller')}' drives '{robot}': its cmd_vel "
                                   f"comes every tick, the keys only in between — stop it to drive by hand"))
        actions.append(ExecuteProcess(cmd=controller_cmd, name=f"node_{robot}", **run))
    else:
        # The alternative is a window that ignores the keyboard: with no node there is exactly one
        # driver, and it is the person in front of it.
        actions.append(LogInfo(msg=f"[lab] no node started — the keyboard drives '{robot}' "
                                   f"(w/s drive, a/d strafe, q/e turn, shift x2); controller:=<file> hands the wheel to a node"))
    start_rviz, note = rviz_view.plan(read_arg("rviz"))
    if start_rviz:
        viewer = rviz_view.command(rviz_view.render_config(REPO, robot),
                                   sim_time=read_arg("use_sim_time").lower() in TRUE)
        actions.append(ExecuteProcess(cmd=viewer, name="rviz2", **run))
    if note:
        actions.append(LogInfo(msg=note))
    return actions


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument(name, default_value=default, description=text)
                              for name, default, text in BASICS]
                             + [OpaqueFunction(function=start)])
