"""The readout line and the generated documentation pictures.

Two things that look like cosmetics and are not:

* a readout line that stops showing what a sensor says about itself just because a *drawing* was
  switched off — the numbers a student has to reason about then depend on the `k` key;
* a screenshot in the documentation that nobody can regenerate, which is how a figure starts to lie.

Both are checked here on the real code path: `Renderer._hud()` for the first, and the command README
quotes — `./lab sim … --frame-max N --screenshot FILE.png`, i.e. `node.main()` — for the second. The
pictures this writes are the GPS-shadow and the radio-link figures of the handout and of the demos page.
"""
import collections
import contextlib
import os
import pathlib
import re
import shlex
import struct
from types import SimpleNamespace

import pygame
import pytest

from mecanum_lab import node, overlays, stub
from mecanum_lab.render import KF_SIGMA, Renderer
from mecanum_lab.render import rgb as rgb_value
from support_docs import text as pages_text
from support_logging import logged, messages
from mecanum_lab.types import (Gps, Imu, Kf, Odom, PALETTE, Pose, Rect, Robot, RobotSpec, Twist,
                               World)

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")     # the pictures are drawn without a screen

CFG = {"width": 640, "height": 400, "gui_rate": 60, "rate": 50,
       "robot": {"lx": 0.14, "ly": 0.13, "r": 0.05, "footprint_r": 0.21},
       "gui_style": {"wall": [0.34, 0.36, 0.42], "floor": [0.13, .14, .17], "trail_len": 40}}
WORLD = World(name="fake", cell=0.5, walls=[Rect(0, 0, 6, 0.2)], spawns=[Pose(1, 1, 0)],
              goal=Pose(4.5, 3, 0), size=(6.0, 4.0))
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def make_robot(name="alice", kf=True):
    color, rgb = PALETTE[0]
    return Robot(spec=RobotSpec(name=name, index=0, color=color, rgb=rgb, marker="triangle"),
                 pose=Pose(1.0, 1.0, 0.0), twist=Twist(0.3, 0.0, 0.1),
                 odom=Odom(x=1.05, y=0.98, theta=0.02, vx=0.3),
                 gps=Gps(x=1.02, y=0.99, quality=2, sats=8),
                 imu=Imu(ax=0.11, ay=-0.02, gz=0.031, temp=27.4),
                 kf=Kf(x=1.01, y=1.0, sx=0.17, sy=0.05) if kf else None,
                 kf_err=0.02 if kf else None)


def make_engine(robot):
    return SimpleNamespace(world=WORLD, robots={robot.spec.name: robot}, t=3.25, task="",
                           drain=lambda: [], cfg=CFG)


@contextlib.contextmanager
def gui(robot):
    """Renderer over one robot; `close()` even when a test fails inside the block."""
    rend = Renderer(make_engine(robot), CFG)
    try:
        yield rend
    finally:
        rend.close()


def hud_labels(rend, robot):
    """Every string `_hud()` puts on screen — by recording what it renders, not by reading pixels."""
    seen = []
    rend._text = lambda text, x, y, color, big=False, center=False: seen.append(text) or 0
    rend._hud()
    return " | ".join(seen)


# ------------------------------------------------------------------ the readout line


def test_sigma_legend_names_metres_and_the_same_half_axis_in_pixels():
    """The ellipse is `2σ · px_per_metre` wide on screen; the line has to say both numbers."""
    robot = make_robot()
    with gui(robot) as rend:
        text = overlays.sigma_legend(rend, robot)[0][0]
        metres = KF_SIGMA * robot.kf.sx
        assert f"{metres:.2f} × {KF_SIGMA * robot.kf.sy:.2f} m" in text
        assert f"{round(metres * rend.s)} × {round(KF_SIGMA * robot.kf.sy * rend.s)} px" in text
        assert f"{rend.s:.0f} px/m" in text


def test_sigma_legend_is_empty_without_an_estimate():
    robot = make_robot(kf=False)
    with gui(robot) as rend:
        assert overlays.sigma_legend(rend, robot) == []


@pytest.mark.parametrize("show_kf", [True, False])
def test_the_sensor_of_the_run_stays_readable_when_an_estimate_layer_is_off(show_kf):
    """`k` switches the estimate *drawing* off — the sensor readout is not part of it.

    The IMU segment and the σ legend are the two places where a student reads what the estimate and
    the chip are worth, and both have to survive the layer key: hiding dots is not hiding numbers.
    """
    robot = make_robot()
    with gui(robot) as rend:
        rend.show_kf = show_kf
        line = hud_labels(rend, robot)
    assert "imu ax=+0.11 ay=-0.02 gz=+0.031 27.4 °C" in line
    assert "ellipse 2σ" in line
    assert "gps x=+1.02 y=+0.99 q2 8 sats" in line
    assert "kf x=+1.01" in line                      # the estimate itself stays in the line too


# ----------------------------------------------------------------- the picture command


def picture(tmp_path, name, extra, capsys):
    """Run the documented command once and report the PNG it wrote and the line it printed.

    A private bus per run on purpose: `cmd_sim` shuts its bus down at the end, and the in-process bus
    is a process singleton — without this, the second run of a test would find a bus that is already
    closed and draw zero frames.
    """
    out = tmp_path / name
    stub.set_bus(stub.StubBus(f"picture {name}"))
    try:
        code = node.main(["sim", "--stub", "--headless", "--fixed-step", "--frame-max", "30",
                          "--screenshot", str(out)] + extra)
    finally:
        stub.set_bus(None)
    assert code == 0
    return out, capsys.readouterr().out


def png_size(path):
    head = path.read_bytes()[:24]
    assert head[:8] == PNG_MAGIC, f"{path} is not a PNG"
    return struct.unpack(">II", head[16:24])


DOCS_IMG = pathlib.Path(__file__).resolve().parent.parent / "docs" / "img"
HEADLESS_ABOVE = 32         # the two text rows: line 1 carries the fps counter, nothing below it can vary


def readme_picture_commands():
    """The screenshot commands the documentation prints, as argv lists: figure and recipe in one place.

    A test that carries its own copy of a command proves nothing about the command in the documentation.
    The two drifted apart here: the README line lost its `--robots`, which drew the hall with nobody in it
    while the caption underneath went on describing a robot at its spawn pose. So the commands are read
    out of the documentation and run exactly as a reader would type them — out of every page, since a
    figure is allowed to live on the page about its subject (`docs/demos.md` carries the demo figures),
    and a reader finds that page from the front page.
    """
    found = {}
    for block in re.findall(r"```bash\n(.*?)\n```", pages_text(), re.S):
        for line in re.sub(r"\\\n\s*", " ", block).splitlines():
            line = line.strip()
            if "./lab sim" in line and "--screenshot docs/img/" in line:
                argv = shlex.split(line)
                assert argv[:2] == ["./lab", "sim"], f"not a sim command: {line}"
                found[os.path.basename(argv[argv.index("--screenshot") + 1])] = argv[2:]
    return found


def picture_argv(png: str, out, *extra) -> list:
    """README's command for one figure, aimed at `out` instead of at docs/img, and on the stub bus.

    `--stub` is the only liberty taken: the CI that runs this has no ROS daemon to talk to, and the
    frame is the same either way — the sim is stepped by `--fixed-step` and every sensor is seeded.
    """
    argv = readme_picture_commands()[png]
    where = [i for i, a in enumerate(argv) if a == "--screenshot"]
    assert len(where) == 1, f"{png} appears with no --screenshot in README"
    return (["sim", "--stub"] + argv[:where[0] + 1] + [str(out)] + argv[where[0] + 2:] +
            list(extra))


def pixels(surface) -> tuple:
    """(width, height, bytes below the two text rows) — the part of a frame that may be compared."""
    w, h = surface.get_size()
    raw = pygame.image.tostring(surface, "RGB")
    return w, h, raw[HEADLESS_ABOVE * w * 3:]


PICTURES = sorted(readme_picture_commands())
assert PICTURES == ["readout-gps-shadow.png", "readout-radio.png"], PICTURES


@pytest.mark.parametrize("png", PICTURES)
def test_the_documentation_picture_is_that_command_run_today(png, tmp_path, capsys):
    """docs/img/*.png is a build product, so it has to be the documented command's output — today's.

    Everything below the two text rows is compared byte for byte; the fps counter in line 1 is the one
    number in a frame that no headless run can promise, and the readout line above the map has its own
    tests. What is left is the hall, the zones, the access point and the robot: if any of them moves,
    or a drawing changes, the committed picture is wrong and this says so.
    """
    out = tmp_path / png
    assert node.main(picture_argv(png, out)) == 0
    assert out.exists(), f"{png} was not written"
    made = pixels(pygame.image.load(str(out)))
    committed = pixels(pygame.image.load(str(DOCS_IMG / png)))
    assert made[0] == committed[0] and made[1] == committed[1], \
        f"{png}: the repository holds a {committed[0]}x{committed[1]} picture, README draws " \
        f"{made[0]}x{made[1]}"
    assert made[2] == committed[2], (f"{png} is not what the command in README draws today; regenerate "
                                     f"it with that command — a stale figure is a figure that lies")


@pytest.mark.parametrize("png", PICTURES)
def test_the_documentation_picture_shows_a_robot(png, tmp_path, capsys):
    """A figure of a hall with no robot in it passed every size and colour check there was.

    The command that drew it had lost its `--robots`, and the caption described a robot at its spawn
    pose; the frame said `robots: 0`. So the promise is measured here: the colour a robot is drawn in
    has to be in the picture, in an amount no wall, zone hatch or goal bullseye contributes — those
    share no exact colour with a robot, which is why a picture of an empty hall has none of it.
    """
    out = tmp_path / png
    assert node.main(picture_argv(png, out, "--set", "width=640", "--set", "height=400")) == 0
    frame = pygame.image.load(str(out))
    bodies = {rgb_value(value) for _, value in PALETTE}
    seen = collections.Counter(tuple(frame.get_at((x, y))[:3])
                               for y in range(HEADLESS_ABOVE, frame.get_height())
                               for x in range(0, frame.get_width(), 2))
    on_robot = sum(count for colour, count in seen.items() if colour in bodies)
    assert on_robot > 10, f"no robot drawn in {png}: {seen.most_common(4)}"


@pytest.mark.parametrize("name,extra", [("gps-shadow", ["--world", "production",
                                                        "--config", "config/demo_gps_shadow.json",
                                                        "--robots", "muster"]),
                                        ("radio-link", ["--world", "production",
                                                        "--config", "config/demo_wifi.json",
                                                        "--robots", "muster"])],
                         ids=["gps-shadow", "radio-link"])
def test_the_documented_picture_command_writes_a_picture(tmp_path, name, extra, capsys):
    """`./lab sim … --frame-max N --screenshot FILE.png` draws a frame, not a black rectangle."""
    out, printed = picture(tmp_path, f"{name}.png", extra + ["--set", "width=640", "--set",
                                                             "height=400"], capsys)
    assert png_size(out) == (640, 400)
    assert "30 frame(s)" in printed and f"{30 / 50:.2f} s" in printed
    assert out.stat().st_size > 2000, "an empty frame compresses to a few hundred bytes"
    frame = pygame.image.load(str(out))          # a real frame, not a black rectangle
    assert frame.get_height() == 400
    colours = {tuple(frame.get_at((x, y))[:3]) for x in range(0, 640, 32) for y in range(0, 400, 32)}
    assert len(colours) >= 4, f"only {colours} in the frame"


def test_two_picture_runs_in_one_process_both_draw(tmp_path, capsys):
    """The second `node.main()` in one process used to end before its first frame.

    Teardown hands the process back a stub bus that is shut down and still *the* bus of the process,
    and `run_loop` reads `bus.ok() == False` as somebody pressing stop. The loop then never draws,
    `--frame-max` is satisfied by zero drawn frames, and the picture of the unpainted window is saved
    and announced as if it had been rendered. One `./lab` command per process hides that; a tool that
    generates two figures in a row does not, and a documentation figure of an empty hall — with a
    caption about a robot at its spawn pose — is what a run of zero frames looked like in git.
    """
    seen = []
    for name in ("first.png", "second.png", "third.png"):
        out = tmp_path / name
        assert node.main(picture_argv("readout-radio.png", out, "--set", "width=320",
                                     "--set", "height=200")) == 0
        printed = [line for line in capsys.readouterr().out.splitlines() if line.startswith("picture:")]
        assert len(printed) == 1, f"{name}: no picture line, so nothing was drawn: {printed}"
        assert "30 frame(s)" in printed[0], f"{name} drew nothing: {printed[0]}"
        seen.append(out.read_bytes())
    assert seen[0] == seen[1] == seen[2], "the same command gave three different frames"


def test_a_shut_down_bus_is_no_longer_the_bus_of_the_process():
    """The rule that the test above depends on, stated where it lives.

    `get_bus()` promises the bus of this process; after `shutdown()` it has to mean a bus that still
    delivers, or every caller that asks for "the" bus afterwards gets one that answers `ok()` with
    False — which the run loop cannot tell apart from a request to stop.
    """
    saved = stub.get_bus(create=False)
    try:
        first = stub.get_bus()
        first.shutdown()
        assert not first.ok()
        second = stub.get_bus()
        assert second is not first, "a shut-down bus is still what get_bus() hands out"
        assert second.ok()
        delivered = []
        second.sub_topic("test", delivered.append)
        second.publish("test", 1)
        assert delivered == [1], "the replacement bus does not deliver"
    finally:
        stub.set_bus(saved)


def test_a_picture_of_nothing_is_not_written(tmp_path):
    """An unpainted window is not a picture: refuse it, and say why.

    Without this the failure above stayed invisible — a PNG appeared, with a line about it on stdout.
    """
    target = tmp_path / "never.png"
    with logged("mecanum.node") as records:
        node.save_picture(SimpleNamespace(screen=None), str(target), frames=0, sim_t=0.0)
    assert not target.exists(), "a run that drew nothing still wrote a PNG"
    assert any("not written" in text for text in messages(records)), messages(records)


def test_the_picture_command_is_reproducible(tmp_path, capsys):
    """Same command, same seed, same frame: the sim time of frame N does not depend on the host.

    That is what makes a documentation figure regenerable instead of merely re-screenshotable — the
    one number in the header that cannot be reproduced is the fps counter of the window.
    """
    extra = ["--world", "arena", "--robots", "alice,bob", "--seed", "3"]
    first, printed_a = picture(tmp_path, "a.png", extra, capsys)
    second, printed_b = picture(tmp_path, "b.png", extra, capsys)
    def moments(text):
        """The "picture:" line without its path — the frame count and the sim time are the point."""
        return [line.split("— ")[-1] for line in text.splitlines() if line.startswith("picture:")]

    assert moments(printed_a) == moments(printed_b)
    assert png_size(first) == png_size(second)
    # Not a byte comparison: the fps counter in the header line is the one number in the frame that
    # cannot be reproduced. Same content, so the file sizes stay within a whisker of each other.
    assert abs(first.stat().st_size - second.stat().st_size) < 4096
