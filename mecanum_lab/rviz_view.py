"""RViz as the second view of the same topics — and what to do on the machine that has no RViz.

The lab's own window is the picture the students work with: it draws the world of the world file, the
shadow zones, the radio link, everything the simulator knows. RViz is offered beside it for a different
question — *what does my robot look like to the rest of the ROS graph?* — because a topic that only the
lab window can read is a topic nobody can debug with the tools of the framework. Both views read the
same messages: `tf_bcast.frames()` names the frames (`map` → `<robot>/odom` → `<robot>/base_link`) and
the displays below are exactly the kinds in `types.MSG_SPECS`.

The config is a template with one placeholder, the robot name: topic names are per robot, and an RViz
config that names `alice` shows nothing for a class that drives `muster`. RViz 2 itself has no way to
substitute a name into a display's topic, so `render_config()` writes the file at launch time.

`available()` is the part that matters on the machines of the lab room and of CI: rviz2 is not part of
the ROS base install (`ros-$ROS_DISTRO-ros-base` has no GUI), and a launch file that starts with

    [rviz2-3] process has died ...

or, worse, one that silently never starts the view, teaches the wrong lesson twice. So the launch files
ask here first, and the answer students get is a line that says what happened and what to type.
"""
import os
import shutil

TEMPLATE = os.path.join("config", "rviz", "template", "lab.rviz")
INSTALL_HINT = "sudo apt install ros-$ROS_DISTRO-rviz2"
DEFAULT_OUT = "/tmp/mecanum_rviz_{robot}.rviz"


def available() -> bool:
    """Is there an `rviz2` to start? PATH only — this never imports ROS and never runs anything."""
    return shutil.which("rviz2") is not None


def render_config(repo: str, robot: str, out_path: str | None = None) -> str:
    """Fill the template for one robot and write it; returns the path RViz should be given.

    The file keeps the `.rviz` suffix on purpose: `tools/langcheck.py` reads `.rviz` files as
    markup that students read, and a template is the same kind of text. The generated file in /tmp is a
    build product of a launch and is never edited.

    The placeholder is `{robot}`, and only in the topic names. Writing to /tmp by default keeps the
    repository clean: the file is a build product of a launch, regenerated every time, and a student
    editing it will find their change gone on the next start — which is why the template, not the
    output, is the file under version control.
    """
    source = os.path.join(repo, TEMPLATE)
    if not os.path.exists(source):
        raise FileNotFoundError(f"the RViz template is missing: {source} — it is part of the "
                                f"repository (config/rviz/lab.rviz.template), do not delete it")
    with open(source, encoding="utf-8") as handle:
        text = handle.read()
    if "{robot}" not in text:
        raise ValueError(f"{TEMPLATE} has no {{robot}} placeholder; every topic of a robot is its own")
    target = out_path or DEFAULT_OUT.format(robot=robot)
    directory = os.path.dirname(os.path.abspath(target))
    os.makedirs(directory, exist_ok=True)
    with open(target, "w", encoding="utf-8") as handle:
        handle.write(text.replace("{robot}", robot))
    return target


def plan(answer: str) -> tuple:
    """(start it?, one line for the user or None) — the whole RViz decision in one place.

    A launch file that only knows "started" and "not started" leaves the reader of the screen with a
    window that is one fewer than the documentation promised and no word about why. So `auto` on a
    machine without rviz2 still says something, and so does the `true` that cannot be honoured.
    """
    value = (answer or "auto").strip().lower()
    if value in ("false", "0", "no", "off"):
        return False, None
    if available():
        return True, None
    if value in ("auto", ""):
        return False, ("rviz2 is not on this machine, so the lab window is the only view — "
                       f"`{INSTALL_HINT}` adds the ROS-side one")
    if value in ("true", "1", "yes", "on"):
        return False, missing_message()
    raise ValueError(f"rviz:={value} is neither auto, true nor false")


def missing_message() -> str:
    """The line a launch prints when RViz was asked for and is not on this machine."""
    return (f"rviz2 is not installed — the lab window shows the same topics, and `{INSTALL_HINT}` "
            f"adds the ROS-side view (`./lab rviz --robot <name>` starts it alone, `rviz:=false` stops "
            f"asking)")


def command(config_path: str, sim_time: bool = True) -> list:
    """The RViz 2 command line, with the config this run generated and the clock the sim stamps with.

    `use_sim_time` is not a detail: the simulator stamps its TF transforms in seconds since start, and a
    viewer on the wall clock compares those against now — the tree is complete, every topic is arriving,
    and the map stays empty because RViz believes all of it happened 1.7 billion seconds ago. That is one
    of the most confusing dead ends in this lab, so it is decided here, once, from the same `use_sim_time`
    the rest of the run is started with, rather than left to whoever remembers the flag.

    No `__ns:=` and no topic remapping: the topics of this project are absolute and already carry the
    robot name, so a namespace would only show the same robot a second time under a longer name. The DDS
    domain comes from ROS_DOMAIN_ID in the environment, as everywhere else here.
    """
    cmd = ["rviz2", "--display-config", config_path]
    if sim_time:
        cmd += ["--ros-args", "-p", "use_sim_time:=true"]
    return cmd
