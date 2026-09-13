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


def test_readme_picture_is_in_the_repo():
    """The README links docs/img/worlds.png — the file has to be in the repository."""
    with open(os.path.join(ROOT, "README.md"), encoding="utf-8") as fh:
        assert "docs/img/worlds.png" in fh.read()
    assert os.path.getsize(IMAGE) > 5000, "run: python3 tools/worldpic.py"
