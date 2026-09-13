"""The documentation as one corpus.

Half of the documentation guards used to open `README.md` by name and assert that a sentence, a number
or a figure link was in it. That coupling made the front page grow: the moment a paragraph was checked
by a test, moving it off the front page failed the build, so everything a guard touched stayed on page
one — which is how this README reached 400 lines with no student reading past the first screen.

What the guards actually want is *the documentation*: the front page plus the topic pages under `docs/`.
Which file a fact lives in is an editorial decision, not a property of the code, so it is decided here
once and the tests ask for the corpus.
"""
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent


def pages():
    """`README.md`, every topic page under `docs/`, and the one page per demo, in a stable order.

    The demo pages are a directory of their own because there are six of them and they have a common
    shape; a guard that looked only at `docs/*.md` would not see them, which is how a fact checked in
    one place ends up pasted back onto the front page. The corpus is the corpus.
    """
    return ([ROOT / "README.md"] + sorted((ROOT / "docs").glob("*.md"))
            + sorted((ROOT / "docs" / "demos").glob("*.md")))


def text():
    """All of it as one string, in the order of :func:`pages`."""
    return "\n".join(path.read_text(encoding="utf-8") for path in pages())


def where(needle):
    """Which page contains `needle` — so a failure can say where a fact went, not only that it went."""
    return [path.relative_to(ROOT).as_posix() for path in pages()
            if needle in path.read_text(encoding="utf-8")]
