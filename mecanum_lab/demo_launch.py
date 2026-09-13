"""The one implementation behind the six `launch/demo_*.launch.py` files.

A demo should be startable by naming it — `ros2 launch mecanum_lab demo_gps_shadow.launch.py` — instead of
by remembering which argument carries which config file. Six launcher files do that, and every one of them
is a few lines that call :func:`description` here. What "starting a demo" means (which config file, whether
RViz, which arguments are worth typing at all, what is said on the screen) is one decision; six copies of it
would be right for a month and then disagree, and the students are the ones who find out.

**The demos are read from `config/`, never listed by hand.** A file `config/demo_<name>.json` *is* a demo:
its launcher is `demo_<name>.launch.py`, the number in `--show-args` and the completion of
`ros2 launch mecanum_lab <TAB>` all follow from the folder. That is also why the launcher passes an
*absolute* config path — a launch file is started from whatever directory the shell happens to be in, and
after `colcon build` there is no "here" that contains the configs (see `types.data_root`).
"""
import glob
import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from .types import ROOT

#: launch/lab.launch.py, next to this file in the source tree and in `share/mecanum_lab/launch`
LAB = os.path.join(ROOT, "launch", "lab.launch.py")

#: The arguments a demo run is typed with. Everything else — `task`, `grade`, the sensor knobs — belongs to
#: a whole experiment and is listed by `launch/lab.launch.py --show-args`, which is also where it is applied.
ARGS = (
    ("robot", "muster", "your robot name — RViz and the topics of this one"),
    ("robots", "", "extra robots to spawn (comma-separated, empty = only yours)"),
    ("controller", "", "your node (empty = the keyboard alone drives)"),
    ("seconds", "0", "end after N s of simulation time (0 = until q or Ctrl-C)"),
    ("view", "", "what the window shows: clean (default) or sensors"),
    ("layers", "", "single layers over that view, e.g. scan,ghost or -hud"),
    ("rviz", "auto", "RViz 2 beside the window: auto = when installed, true = insist, false = no"),
    ("headless", "false", "no pygame window (CI, or a machine without a screen)"),
)


def demos() -> list:
    """Every demo of this installation, in the order a reader should meet them."""
    return sorted(os.path.basename(path)[len("demo_"):-len(".json")] for path in
                  glob.glob(os.path.join(ROOT, "config", "demo_*.json")))


def config_of(demo: str) -> str:
    """Absolute path of a demo's config — see the module docstring about the working directory."""
    return os.path.join(ROOT, "config", f"demo_{demo}.json")


def demo_of(launch_file: str) -> str:
    """`demo_gps_shadow.launch.py` -> `gps_shadow`; anything else names the demos there are.

    The file name is not parsed for convenience: `test_every_demo_has_its_own_launcher` pairs the launchers
    and the configs both ways, so a launcher that points at a config nobody wrote is a failing test and not
    a launch that opens the wrong hall.
    """
    name = os.path.basename(launch_file)
    demo = name[len("demo_"):-len(".launch.py")] if name.startswith("demo_") else name
    if os.path.exists(config_of(demo)):
        return demo
    raise ValueError(f"{name} is not the launcher of a demo: there is no {config_of(demo)}. "
                     f"Demos of this installation: {', '.join(demos()) or 'none'}")


def description(demo: str | None = None) -> LaunchDescription:
    """The description of one named demo, or of `demo.launch.py` when `demo` is asked for by argument.

    Nothing here starts a process: `lab.launch.py` is included, and it is the one place that knows how many
    processes a lab run is (sim, optional node, optional RViz). Duplicating that list is how a launch file
    ends up starting three of the four things the documentation promises.
    """
    known = demos()
    declared = [] if demo is not None else [DeclareLaunchArgument(
        "demo", default_value=known[0] if known else "",
        description="which demo to drive: " + ", ".join(known))]
    declared += [DeclareLaunchArgument(name, default_value=default, description=text)
                 for name, default, text in ARGS]
    return LaunchDescription(declared + [OpaqueFunction(function=lambda context: _start(context, demo))])


def _start(context, demo: str | None) -> list:
    read = lambda name: LaunchConfiguration(name).perform(context)              # noqa: E731
    chosen = demo or read("demo")
    known = demos()
    if chosen not in known:
        raise ValueError(f"demo:={chosen} has no {os.path.join(ROOT, 'config', f'demo_{chosen}.json')}. "
                         f"The demos of this installation are: " + ", ".join(known))
    forwarded = [("config", config_of(chosen))]
    forwarded += [(name, read(name)) for name, _default, _text in ARGS if read(name)]
    return [LogInfo(msg=f"[demo] {chosen}: {config_of(chosen)} — the keyboard in the window drives "
                        f"'{read('robot')}', `rviz:={read('rviz')}`"),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(LAB), launch_arguments=forwarded)]
