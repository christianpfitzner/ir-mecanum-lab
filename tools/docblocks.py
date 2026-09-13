#!/usr/bin/env python3
"""One command per code block — because a reader copies the whole block and types it blind.

    python3 tools/docblocks.py                      # every page, violations only
    python3 tools/docblocks.py README.md docs/*.md  # what these pages do wrong

The pages of this repository are read at a machine with one hand on a robot. A block with two commands
in it is a block where half the readers type one of the two and get something else than the sentence
above it promised — a `ros2 launch …` followed by a `./lab …` in the same block is also a block where
nobody can see that the second line is an alternative, not a second step. So: a fenced ```bash block
holds one command. It may be several lines long, continued with `\\` as usual, and a line that starts
with `#` is a comment about that command.

The block *before* a table of flags is still one command; the `| --- |` rows of a table are not commands,
and neither is a block that is not bash (`text`, `json`, the C++ and Python snippets of the API page).
`docs/praktikum/` is a document with its own rules and is not walked, and neither are the notes of
whoever built the simulator — where a block with a `for` loop in it is a recipe, not a command to type.
"""
from __future__ import annotations

import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
# The pages a student reads: the landing page, the pages next to it, the demo pages. Not `docs/notes/`
# (those are the notes of whoever built the simulator, where a block is a recipe with a loop in it), not
# `docs/praktikum/` (a handout with its own typesetting), not `build/` and `install/` (colcon's copies).
PAGES = ["README.md", "docs/*.md", "docs/demos/*.md"]
FENCE = re.compile(r"^```(\w*)\s*$")


def commands(body: list[str]) -> list[tuple[int, str]]:
    """The first line of every command in a block: a continuation line is not a new command."""
    found, previous = [], ""
    for offset, line in enumerate(body):
        text = line.strip()
        if not text or text.startswith("#"):
            continue
        if previous.endswith("\\"):               # still the same command, indented further
            previous = text
            continue
        found.append((offset, text))
        previous = text
    return found


def scan(path: Path) -> list[str]:
    """Every block in `path` that holds more than one command, as `file:line: what is in it`."""
    lines = path.read_text(encoding="utf-8").split("\n")
    problems, block, start = [], None, 0
    for number, line in enumerate(lines, 1):
        fence = FENCE.match(line)
        if fence and block is None:
            block, start = fence.group(1), number
            body = []
            continue
        if block is not None and line.startswith("```"):
            if block == "bash":
                found = commands(body)
                if len(found) > 1:
                    listed = ", ".join(text.split()[1] if text.startswith("./") else text.split()[0]
                                       for _, text in found)
                    problems.append(f"{path.relative_to(REPO)}:{start}: {len(found)} commands "
                                    f"({listed}) — first words: {found[0][1][:48]}")
            block = None
            continue
        if block is not None:
            body.append(line)
    return problems


def main(argv: list[str]) -> int:
    pages = ([Path(argument) for argument in argv] if argv
             else [page for pattern in PAGES for page in sorted(REPO.glob(pattern))])
    problems = [problem for page in pages for problem in scan(page)]
    for problem in problems:
        print(problem)
    if problems:
        print(f"\n{len(problems)} block(s) with more than one command: split them, one command per "
              "block — a reader copies the whole block.")
        return 1
    print(f"{len(pages)} page(s): every ```bash block holds one command")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
