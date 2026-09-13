"""The readout line and the generated documentation pictures.

Two things that look like cosmetics and are not:

* a readout line that stops showing what a sensor says about itself just because a *drawing* was
  switched off — the numbers a student has to reason about then depend on the `k` key;
* a screenshot in the documentation that nobody can regenerate, which is how a figure starts to lie.

Both are checked here on the real code path: `Renderer._hud()` for the first, and the command README
quotes — `./lab sim … --frame-max N --screenshot FILE.png`, i.e. `node.main()` — for the second. The
pictures this writes are the GPS-shadow and the radio-link figures of the handout and of README.
"""
import contextlib
import math
import os
import struct
from types import SimpleNamespace

import pygame
import pytest

from mecanum_lab import node, overlays, stub
from mecanum_lab.render import Renderer, KF_SICHERHEIT
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
        metres = KF_SICHERHEIT * robot.kf.sx
        assert f"{metres:.2f} × {KF_SICHERHEIT * robot.kf.sy:.2f} m" in text
        assert f"{round(metres * rend.s)} × {round(KF_SICHERHEIT * robot.kf.sy * rend.s)} px" in text
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


PICTURES = [("gps-shadow", ["--world", "production", "--config", "config/demo_gps_shadow.json",
                            "--robots", "muster"]),
            ("radio-link", ["--world", "production", "--config", "config/demo_wifi.json",
                            "--robots", "alice"])]


@pytest.mark.parametrize("name,extra", PICTURES, ids=[p[0] for p in PICTURES])
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
