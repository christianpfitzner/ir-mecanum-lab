"""The documentation picture: docs/img/worlds.png is generated, so it must keep generating.

One test, two things: the tool still draws every shipped world (no window, no network), and the
PNG that the README links is present and non-trivial. If someone edits worlds/*.txt into
something unparsable, or breaks tools/worldpic.py, the README figure is broken too — that is
what this catches. How many panels there are follows `worlds/`: since `open` the grid is five wide
in three columns, and `tests/test_world_poi_w5.py` is where the numbers under the panels are
compared with the README.
"""
import os
import subprocess
import sys

from support_docs import where

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
