"""docs/img/howitworks.png: the one figure that explains how the whole lab fits together.

The promise of this figure is in its docstring, and every clause of that promise is checked here:

* it is **drawn from the code**, not about it — `labmap.check_roller_axes()` asks
  `physics.forward_kinematics()` for every wheel whether the velocity that wheel produces stands
  perpendicular to the roller axis the figure is about to draw, so a figure that taught the mirrored
  robot (the mistake this repository made in its window for years) cannot be written at all;
* the topic names and message types come from `types.topic()` and `types.MSG_SPECS`, so the figure
  cannot go on naming a topic that was renamed;
* it is a **build product**: the file in the repository is byte for byte what the tool writes today,
  twice over — the same command twice gives the same bytes;
* and the layout guard is real. `Sheet.place()` is the reason the tool is a class and not a list of
  draw calls, so an overlap has to end the run; a tool that announces "colliding labels are a failed
  run" and then draws them is worse than a tool with no guard, because the figure looks reviewed.
"""
import hashlib
import os
import re
import subprocess
import sys

import pygame
import pytest

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "tools"))

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
import labmap                                                   # noqa: E402
from mecanum_lab import physics                                  # noqa: E402
from mecanum_lab.types import MSG_SPECS, topic                   # noqa: E402

PNG = os.path.join(REPO, "docs", "img", "howitworks.png")
TOOL = os.path.join(REPO, "tools", "labmap.py")


def render(path):
    """Run the tool the way README tells a reader to run it: `python3 tools/labmap.py --out FILE`."""
    done = subprocess.run([sys.executable, TOOL, "--out", path], capture_output=True, text=True,
                          cwd=REPO)
    assert done.returncode == 0, done.stdout + done.stderr
    return done.stdout


def digest(path):
    with open(path, "rb") as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def test_the_figure_in_the_repository_is_the_tool_run_today(tmp_path):
    """A documentation figure nobody can regenerate is a figure that is quietly out of date.

    The same rule as the two screenshots: `tools/worldpic.py` and the `--screenshot` commands are
    compared against their files, and this figure would be the odd one out if it could drift.
    """
    fresh = tmp_path / "howitworks.png"
    print(render(str(fresh)))
    assert digest(str(fresh)) == digest(PNG), (
        "docs/img/howitworks.png is not what tools/labmap.py draws today — regenerate it with "
        "`python3 tools/labmap.py`, do not touch the PNG")


def test_the_same_command_twice_gives_the_same_bytes(tmp_path):
    """No clock, no random, no dictionary order in the pixels: the figure is deterministic.

    This is the property that makes the check above usable rather than a source of noise — and the
    window's own header line, the one number in a screenshot that varies, has no place in a figure
    drawn from a module.
    """
    first, second = tmp_path / "a.png", tmp_path / "b.png"
    out_a, out_b = render(str(first)), render(str(second))
    assert digest(str(first)) == digest(str(second))
    # the line it prints names the file it wrote, so compare the report after the file name
    assert out_a.split(" — ", 1)[1] == out_b.split(" — ", 1)[1], "even its report has to be stable"


def test_the_roller_axes_are_verified_against_the_kinematics_before_any_pixel():
    """The tool checks its own subject matter: a mecanum figure with the wrong rollers is a lesson in the wrong layout.

    The check is asked here too, from the other side: `forward_kinematics()` is called per wheel with
    the velocity that wheel alone produces, and the answer must stand perpendicular to
    `render.ROLLERS` — which is what a free-gliding roller means. If somebody swaps the table (the X
    layout and the O layout differ by exactly one mirror), the tool must refuse to draw, not draw
    something plausible.
    """
    geometry = physics.Geometry()
    assert labmap.check_roller_axes(geometry) is None, "the tool refused its own figure"
    assert [row[0] for row in labmap.wheel_report(geometry)] == list(physics.WHEELS_EN)


def test_the_guard_refuses_overlapping_labels_a_collision_is_a_failed_run():
    """`Sheet.place()` promises that a label on another label ends the run. Prove it raises.

    Without this the promise is a docstring. A figure with two labels on top of each other is not a
    figure a reader can use, and it is exactly the kind of thing that survives a glance at a
    screenshot — the tool has to notice what the eye forgives.
    """
    sheet = labmap.Sheet(400, 200)
    sheet.label(10, 10, "one label here")
    with pytest.raises(labmap.Guard):
        sheet.label(12, 12, "one label here")                       # same place, same size
    with pytest.raises(labmap.Guard):
        sheet.label(390, 10, "a label that leaves the canvas")
    with pytest.raises(labmap.Guard):
        sheet.arrow((10, 10), (1400, 900), labmap.INK)              # a wire off the page


def drawn_loop_text():
    """Panel B on its own, as the flat list of strings it puts on the canvas."""
    sheet = labmap.Sheet(labmap.WIDTH, labmap.HEIGHT)
    labmap.panel_loop(sheet, pygame.Rect(labmap.MARGIN + labmap.PANEL_A_WIDTH + labmap.GAP,
                                         labmap.PANEL_TOP,
                                         labmap.WIDTH - 2 * labmap.MARGIN - labmap.PANEL_A_WIDTH
                                         - labmap.GAP, labmap.PANEL_BOTTOM - labmap.PANEL_TOP))
    return " | ".join(sheet.texts)


# The topics a node's own file subscribes to or publishes. The figure used to draw all ten in the table,
# which made it a second interface table nobody read; the rest belongs in docs/CONTRACT.md.
DRAWN = ("twist", "odom", "gps", "imu", "scan", "kf", "truth")


def test_the_figure_names_the_topics_the_code_publishes():
    """Panel B is a wiring diagram of this repository, so it may not invent wires.

    Every topic drawn on it is one `types.topic()` produces and every message type on it is the one in
    `types.MSG_SPECS` — which is how the figure can be read as documentation of the interface a
    student's node actually has. Checked with a pattern over the drawn strings rather than a second
    list of names, so a hand-edited topic in the tool is caught and not merely re-asserted.
    """
    blob = drawn_loop_text()
    for kind in DRAWN:
        assert topic(kind, labmap.ROBOT) in blob, f"panel B no longer draws {topic(kind, labmap.ROBOT)}"
        assert MSG_SPECS[kind][0].split("/")[-1] in blob, f"panel B names no type for /{kind}"
    real = {topic(kind, labmap.ROBOT) for kind in MSG_SPECS}
    invented = [name for name in re.findall(r"/[a-z_]+(?:/[a-z_]+)+", blob) if name not in real]
    assert not invented, f"panel B draws wires that do not exist: {invented}"


def test_the_loop_panel_stays_a_diagram_and_not_an_interface_table():
    """The whole point of the rewrite: seven wires, because ten wires is a list and a list is not a picture.

    A reader who is shown every topic in `MSG_SPECS` learns the figure is reference material and stops
    looking at it; the figure earns its place by showing the one round trip a node performs. Growing
    this panel is a decision, and this test makes it a decision somebody has to argue with.
    """
    wires = re.findall(r"/[a-z_]+(?:/[a-z_]+)+", drawn_loop_text())
    assert len(wires) <= len(DRAWN), f"panel B has grown to {sorted(set(wires))}"


def test_readme_shows_the_figure():
    """A figure no page links to is a figure no student sees.

    README carries it in the section that explains the lab, next to the sentence that tells the reader
    what they are looking at: the chassis on the left, the loop on the right, both drawn from the
    modules the exercise is graded with.
    """
    with open(os.path.join(REPO, "README.md"), encoding="utf-8") as handle:
        readme = handle.read()
    assert "docs/img/howitworks.png" in readme, "README no longer shows the how-it-works figure"
    assert os.path.getsize(PNG) > 20_000, "a PNG of a few hundred bytes is an empty figure"
