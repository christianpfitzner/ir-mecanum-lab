"""The documentation picture: docs/img/worlds.png is generated, so it must keep generating.

One test, two things: the tool still draws every shipped world (no window, no network), and the
PNG that the README links is present and non-trivial. If someone edits worlds/*.txt into
something unparsable, or breaks tools/worldpic.py, the README figure is broken too — that is
what this catches. How many panels there are follows `worlds/`: since `rooms` the set is six, in three columns, and
`tests/test_world_poi_w5.py` is where the numbers under the panels are compared with the README.
"""
import os
import subprocess
import sys

from support_docs import where

from mecanum_lab.worlds import list_worlds

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WERKZEUG = os.path.join(ROOT, "tools", "worldpic.py")
IMAGE = os.path.join(ROOT, "docs", "img", "worlds.png")


def test_worldpic_draws_every_world(tmp_path):
    target_path = tmp_path / "worlds.png"
    result = subprocess.run([sys.executable, WERKZEUG, "--out", str(target_path)],
                             capture_output=True, text=True, timeout=120, cwd=ROOT)
    assert result.returncode == 0, result.stderr[-400:]
    inhalt = target_path.read_bytes()
    assert inhalt[:8] == b"\x89PNG\r\n\x1a\n", "no PNG written"
    assert len(inhalt) > 5000, "picture too small to contain all the arenas"
    for name in ("arena", "maze", "open", "production", "track"):
        assert name in result.stdout, f"{name} not drawn: {result.stdout}"


def test_the_arena_picture_is_linked_by_a_page_and_present():
    """Some documentation page links docs/img/worlds.png, and the file has to be in the repository.

    Which page is an editorial decision — the figure moved from the front page to the page about the
    arenas when the README was cut. A guard that pins a picture to one filename checks the filename, not
    whether a reader can reach the figure, so it asks the whole documentation and says where it lives.
    """
    assert where("docs/img/worlds.png"), "no documentation page links docs/img/worlds.png"
    assert os.path.getsize(IMAGE) > 5000, "run: python3 tools/worldpic.py"


def test_the_hall_pictures_of_the_front_page_are_the_tool_run_today(tmp_path):
    """The README shows one picture per hall, so every one of them is a build product of this tool.

    A linked figure that nobody regenerates is a figure that lies: `worlds/*.txt` changes and the hall on
    the front page stays as it was. Compared as decoded pixels rather than as file bytes, because the PNG
    encoder differs between machines while the drawing does not — the same reason the readout figures in
    `tests/test_readout_pictures_w7.py` are compared below their text rows.
    """
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    import pygame
    pygame.display.init()

    for name in list_worlds():
        committed = os.path.join(ROOT, "docs", "img", f"world_{name}.png")
        fresh = tmp_path / f"world_{name}.png"
        result = subprocess.run([sys.executable, WERKZEUG, "--worlds", name, "--out", str(fresh)],
                                capture_output=True, text=True, timeout=120, cwd=ROOT)
        assert result.returncode == 0, f"{name}: {result.stderr[-300:]}"
        drew, holds = (pygame.image.load(str(p)) for p in (fresh, committed))
        assert drew.get_size() == holds.get_size(), \
            f"world_{name}.png: the README holds {holds.get_size()}, the tool draws {drew.get_size()}"
        drew_px = pygame.image.tostring(drew, "RGB")
        holds_px = pygame.image.tostring(holds, "RGB")
        assert drew_px == holds_px, \
            f"world_{name}.png is not what the tool draws today — regenerate: python3 tools/worldpic.py " \
            f"--worlds {name} --out docs/img/world_{name}.png"
