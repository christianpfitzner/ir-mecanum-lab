"""The launch files and the RViz view they can start — checked without a GUI and without rviz2.

`ros2 launch` is how the students in the lab room start everything, and a launch file fails in ways a
unit test normally misses: an argument read but not declared is *silently dropped*, a second copy of a
process list drifts from the first, and a view that is not installed leaves a screen one window short of
what the documentation promised with no word about it. `tools/launchargs.py` already checks the argument
tables (name, default, one line of help) without ROS; what is checked here is the rest:

* a launch file must not shadow the ROS framework — the repository has a directory named `launch`, and a
  path entry in front of the ROS packages makes `import launch` answer with that directory, for that
  file *and* for every launch file the same process loads afterwards;
* the demos named for `demo:=` are the config files in `config/`, in both directions;
* the RViz decision (`auto` / `true` / `false`, and what to say when rviz2 is not there) is the helper's,
  and the config it generates names only topics this project really publishes, with the QoS the
  simulator really uses — a best-effort display and a reliable publisher never meet, and the diagnosis
  students then chase is "RViz shows nothing".
"""
import ast
import glob
import os
import re
import sys

import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH = os.path.join(REPO, "launch")
sys.path.insert(0, REPO)

from mecanum_lab import node as node_module                          # noqa: E402
from mecanum_lab import rviz_view                                   # noqa: E402
from mecanum_lab.types import MSG_SPECS, topic                      # noqa: E402
from mecanum_lab.tf_bcast import frames                             # noqa: E402

import importlib.util                                               # noqa: E402

# Where the ROS python packages live differs by distribution and by distro version (Kilted on Ubuntu 24.04
# installs into site-packages, Jazzy's Debian packaging into dist-packages), so this searches instead of
# guessing one path — a wrong guess here does not fail, it *skips* twelve tests that could have run.
def _ros_packages():
    distro = os.environ.get("ROS_DISTRO")
    roots = [f"/opt/ros/{distro}"] if distro else []
    roots += sorted(glob.glob("/opt/ros/*"), reverse=True)
    for root in roots:
        for site in ("site-packages", "dist-packages"):
            candidate = os.path.join(root, "lib", "python3.*", site)
            found = glob.glob(candidate)
            if any(os.path.isdir(os.path.join(f, "launch")) for f in found):
                return found[0], root
    return None, None


PACKAGES, ROS = _ros_packages()
node_main = node_module.main
HAVE_LAUNCH = PACKAGES is not None
FILES = sorted(name for name in os.listdir(LAUNCH) if name.endswith(".launch.py"))


def load(name):
    """Import one launch file the way ros2launch does: by path, with the ROS packages reachable."""
    if PACKAGES not in sys.path:
        sys.path.insert(0, PACKAGES)
    spec = importlib.util.spec_from_file_location(name.replace(".", "_"), os.path.join(LAUNCH, name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.skipif(not HAVE_LAUNCH, reason="launch files need a sourced ROS 2")
@pytest.mark.parametrize("name", FILES)
def test_a_launch_file_loads_and_does_not_shadow_the_framework_it_runs_in(name):
    """`import launch` has to keep meaning ROS's launch, also after this file has been loaded.

    The repository's own `launch/` directory is a candidate package for that name, so a launch file that
    puts the repository root at the *front* of `sys.path` — the reflex way to make a project importable —
    breaks the launcher for every file the same process loads afterwards. `lab.launch.py` imports
    `mecanum_lab` and appends for exactly that reason; this loads each file the way the launcher does and
    asks what `import launch` answers afterwards, which is the only way the mistake shows up at all.
    """
    module = load(name)
    assert module.generate_launch_description().entities, f"{name} describes nothing to start"
    import launch
    where = os.path.dirname(os.path.abspath(launch.__file__ or "none"))
    assert where.startswith(ROS), (
        f"{name} made `import launch` answer with {where}, and the repository has a directory of that "
        f"name — put it *behind* the ROS packages, never in front of them")


@pytest.mark.skipif(not HAVE_LAUNCH, reason="needs the launch files' own module")
def test_the_demos_named_for_demo_are_the_config_files():
    """`--show-args` is what a student reads; it must not list a demo that cannot be launched.

    Both directions: a name in the argument's text without `config/demo_<name>.json` is a promise the
    launch file cannot keep, and a config file that is not named there is a demo nobody finds.
    """
    module = load("demo.launch.py")
    assert set(module.documented_demos()) == set(module.available_demos())
    assert "gps_shadow" in module.documented_demos()


def test_the_rviz_answer_means_what_it_says(tmp_path, monkeypatch):
    """auto = if it is installed, true = insist, false = keep out — and never silence about a miss.

    `rviz2` is not part of a ROS base install, so on a good number of the machines in a lab course the
    answer to `rviz:=auto` is "no". That has to be a line on the screen and not an absent window.
    """
    monkeypatch.setattr(rviz_view.shutil, "which", lambda name: None)
    assert rviz_view.plan("false") == (False, None), "false must not start anything and not explain itself"
    for answer in ("auto", "", None, "true", "yes"):
        start, note = rviz_view.plan(answer)
        assert not start, f"rviz:={answer} cannot start a binary that is not there"
        assert note and "rviz2" in note, f"rviz:={answer} stays silent about the missing view"
        assert "apt install" in note, f"rviz:={answer} does not say how to get it: {note}"
    with pytest.raises(ValueError):
        rviz_view.plan("maybe")

    monkeypatch.setattr(rviz_view.shutil, "which", lambda name: "/opt/ros/kilted/bin/rviz2")
    assert rviz_view.plan("auto") == (True, None), "with rviz2 installed, auto starts it"
    assert rviz_view.plan("true") == (True, None)


def test_the_rviz_config_is_written_for_the_robot_of_the_run(tmp_path):
    """A display bound to `/alice/scan` shows nothing for a class that drives `muster`.

    The generated file has to carry the robot of that run in every topic — and the template, which is
    the file under version control, has to keep the placeholder that makes that possible.
    """
    out = rviz_view.render_config(REPO, "bob", str(tmp_path / "bob.rviz"))
    text = open(out, encoding="utf-8").read()
    assert "{robot}" not in text and "/bob/scan" in text and "/alice/scan" not in text
    with open(os.path.join(REPO, rviz_view.TEMPLATE), encoding="utf-8") as handle:
        assert "{robot}" in handle.read()
    with pytest.raises(FileNotFoundError, match="RViz template is missing"):
        rviz_view.render_config(str(tmp_path), "bob")     # no template -> a named error, not a raw one


def test_the_rviz_displays_show_topics_this_project_publishes():
    """No invented wires: every topic on the figure is one `types.topic()` produces for a robot.

    The fixed frame is checked too — `map` is the root of the tree `tf_bcast` broadcasts, and an RViz
    whose Fixed Frame is not in the tree draws nothing but a warning in its own Status bar.
    """
    text = open(os.path.join(REPO, rviz_view.TEMPLATE), encoding="utf-8").read()
    drawn = re.findall(r"Value: /\{robot\}/(\S+)", text)
    assert drawn, "the template shows no per-robot topic at all"
    published = {topic(kind, "{robot}").split("/", 2)[2] for kind in MSG_SPECS
                 if MSG_SPECS[kind][2] == "robot"}
    assert set(drawn) <= published, f"RViz shows topics nobody publishes: {set(drawn) - published}"
    for kind in ("scan", "odom"):
        assert topic(kind, "{robot}").split("/", 2)[2] in drawn, f"no display for /{kind}"
    assert "Fixed Frame: map" in text
    assert frames("muster")["map"] == "map", "the fixed frame of the view and the tf tree disagree"


def test_the_rviz_displays_ask_for_the_qos_the_simulator_publishes():
    """Reliable publisher, best-effort display: they never find each other, and RViz shows an empty map.

    `ros_bridge` publishes standard QoS on purpose, so every topic block in the template has to ask for
    Reliable. The default of RViz's own LaserScan display is best effort, which is why this is a test and
    not an assumption.
    """
    text = open(os.path.join(REPO, rviz_view.TEMPLATE), encoding="utf-8").read()
    blocks = re.findall(r"Topic:\n((?:        \S.*\n)+)", text)
    assert len(blocks) >= 5, f"only {len(blocks)} topic blocks in the template"
    for block in blocks:
        assert "Reliability Policy: Reliable" in block, block
        assert "Best Effort" not in block, block


def test_demo_launch_includes_lab_launch_rather_than_copying_it():
    """One process list, one set of arguments. A second copy is a second thing to keep true.

    The demo launcher's job is to pick a config file; the sim, the node and RViz are started by
    `lab.launch.py`, which stays the one place that knows how. If somebody pastes a process list into the
    demo file again, the two drift and the students find out first.
    """
    text = open(os.path.join(LAUNCH, "demo.launch.py"), encoding="utf-8").read()
    assert "IncludeLaunchDescription" in text and "lab.launch.py" in text
    started = {alias.name for node in ast.parse(text).body if isinstance(node, ast.ImportFrom)
               for alias in node.names}
    assert "ExecuteProcess" not in started, (
        "demo.launch.py imports ExecuteProcess: it starts processes of its own again, and the sim/node/"
        "rviz list is then maintained twice — which is the drift this test exists for")
    lab = open(os.path.join(LAUNCH, "lab.launch.py"), encoding="utf-8").read()
    for argument in ("config", "view", "layers", "rviz"):
        assert f'("{argument}"' in lab, f"lab.launch.py no longer declares {argument}"


def test_the_viewer_is_told_to_use_the_clock_the_simulator_stamps_with():
    """RViz on the wall clock and TF stamps in seconds since start: a complete tree, an empty map.

    The transform is there, the topics arrive, `ros2 topic echo /tf` is full of data — and the display
    drops everything as 1.7 billion seconds old. The launch files used to remember that flag individually
    (two of three did), so it is decided once here and derived from the same `use_sim_time` the run is
    started with.
    """
    on = rviz_view.command("/tmp/x.rviz", sim_time=True)
    assert on[:3] == ["rviz2", "--display-config", "/tmp/x.rviz"], on
    assert on[-3:] == ["--ros-args", "-p", "use_sim_time:=true"], on
    assert "use_sim_time" not in " ".join(rviz_view.command("/tmp/x.rviz", sim_time=False))


def test_lab_rviz_writes_the_config_of_the_robot_it_was_asked_about(tmp_path, capsys):
    """`./lab rviz --robot bob` — the standalone case, and the honest refusal without rviz2.

    The return code matters: a command that could not open the window it was asked for has not succeeded,
    and the shell that scripted it should know.
    """
    target = tmp_path / "view.rviz"
    code = node_main(["rviz", "--robot", "bob", "--rviz-config", str(target)])
    written = open(target, encoding="utf-8").read() if target.exists() else ""
    assert "/bob/scan" in written, f"`./lab rviz --robot bob` wrote nothing about bob: {written[:120]}"
    assert "/alice/scan" not in written
    printed = capsys.readouterr().out
    if rviz_view.available():
        assert code == 0, printed                       # a real RViz ran and exited; here: not this box
    else:
        assert code == 1, f"no rviz2 on this machine, yet the command reports success: {printed}"
        assert "rviz2 is not installed" in printed, printed


def test_no_launch_file_starts_rviz2_by_itself():
    """One rule for the view: `rviz_view` decides whether it starts, what it is told, and what is said.

    Three launch files used to carry their own copy of `["rviz2", "-d", …]` with an `os.path.exists` in
    front of it — which is how `rviz:=true` came to do nothing at all, quietly, when the config file moved.
    """
    for name in FILES:
        path = os.path.join(LAUNCH, name)
        text = open(path, encoding="utf-8").read()
        tree = ast.parse(text)
        # read the code, not the prose: a docstring that merely *mentions* rviz_view is documentation,
        # and three of these guards have already been fooled by a word in a sentence
        imported = any(isinstance(node, ast.ImportFrom)
                       and any(alias.name == "rviz_view" for alias in node.names)
                       for node in ast.walk(tree))
        called = {node.attr for node in ast.walk(tree) if isinstance(node, ast.Attribute)}
        kwargs = {kw.arg for node in ast.walk(tree) if isinstance(node, ast.Call) for kw in node.keywords}
        assert '["rviz2"' not in text, f"{name} builds an rviz2 command line of its own again"
        if imported:
            assert {"plan", "render_config", "command"} <= called, (
                f"{name} imports the viewer helper and does not use all of it — the rest of the "
                f"repository decides with those three calls")
            assert "available" not in called, f"{name} asks about rviz2 itself, beside the helper"
            assert "sim_time" in kwargs, f"{name} opens a viewer that will show an empty map"
        else:
            assert not called & {"plan", "render_config", "command"}, (
                f"{name} calls the viewer helper without importing it, or forwards `rviz:=` — if it "
                f"forwards, nothing here may decide")

@pytest.mark.skipif(not HAVE_LAUNCH, reason="needs the launch files' own module")
def test_every_demo_name_in_the_documentation_launches():
    """`demo:=gps` stood in the README for a round of commits, and a reader got a raised ValueError.

    Three short names and one config file were invented on the way to a shorter front page: the launcher
    checks its names against `config/`, the tests checked the *launch file's* argument text, and the prose
    in between — the part a student actually copies — was nobody's subject. So the names on the
    documentation pages are looked up in the same list the launcher uses.
    """
    real = set(load("demo.launch.py").available_demos())
    named, files = set(), set()
    for page in ("README.md", os.path.join("docs", "demos.md")):
        text = open(os.path.join(REPO, page), encoding="utf-8").read()
        named |= set(re.findall(r"demo:=([a-z_]+)", text))
        files |= set(re.findall(r"config/demo_([a-z_]+)\.json", text))
    assert named, "no `demo:=` on the documentation pages at all — the scan is broken, not the docs"
    assert named <= real, (
        f"the documentation launches {sorted(named - real)}, the demos are {sorted(real)} — the "
        f"`demo:=` name is the file name without its `demo_` prefix")
    assert files <= real, f"the documentation names config files that do not exist: {sorted(files - real)}"
