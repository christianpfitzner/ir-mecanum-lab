#!/usr/bin/env python3
"""Language guard: written prose in this repository is English (CONTRACT section 1, rule 3).

Runs headless and without ROS. Two detectors, because German creeps back in two ways:

  * umlauts and sharp s — no identifier, topic, unit or file name in this project uses
    them, so any hit is leftover German text;
  * German function words in *prose* — comments, docstrings, Markdown and LaTeX body.
    Only prose is scanned for words: identifiers, keys and the deprecated launch aliases
    (`sekunden`, `aufgabe`, the `_LEGACY_KEYS` map) are data in this repository and would
    only produce noise — `tools/germanids.py` is the one that checks those.

Excluded paths are listed in EXCLUDE. Exit code 1 means "German left behind", with file
and line.

    python3 tools/langcheck.py            # whole repository
    python3 tools/langcheck.py --quiet    # only the summary line
"""
import os
import re
import sys

EXCLUDE = {"__pycache__", ".git", "build", "install", ".pytest_cache"}
CODE = (".py", ".sh")
# .rviz too: an RViz layout is read by students as well, and its `Name:` entries are the labels
# the sidebar shows — German survived there exactly once, while everything around it was English.
MARKUP = (".md", ".tex", ".txt", ".rst", ".makefile", ".json", ".xml", ".rviz")
NAMES = {"lab", "Makefile", "makefile"}
# The two guard tools carry the German vocabulary they search for; reading them would report the
# word list itself. Everything else in the repository is checked.
GUARDS = {os.path.abspath(__file__),
          os.path.abspath(os.path.join(os.path.dirname(__file__), "germanids.py"))}

UMLAUT = re.compile(r"[äöüÄÖÜß]")
DE_WORD = re.compile(
    r"\b(der|die|das|und|mit|für|wird|nicht|oder|wenn|aber|auch|einem|einen|einer|diesem|"
    r"diesen|dieser|diese|dieses|weil|dann|sonst|hier|dort|schon|immer|nur|sehr|mehr|"
    r"kann|muss|musste|soll|werden|wurde|haben|hatte|gibt|machen|meldet|setzt|liegt|"
    r"steht|falsch|richtig|zuerst|danach|vorher|auftrag|aufgaben|roboter|messung|"
    r"messungen|schätzung|simulationszeit|sensorprofil|bewerter|studierende|welt|halle|"
    r"wand|wanduhr|zustand|fenster|wert|datei|einmal|zwei|drei|vier|erste|zweite|"
    r"letzte|versuch|anleitung|arbeitsblatt|musterlösung)\b", re.I)


def files():
    for path, dirs, files in os.walk("."):
        dirs[:] = [o for o in dirs if o not in EXCLUDE]
        for name in files:
            full = os.path.abspath(os.path.join(path, name))
            if full in GUARDS:
                continue
            if name in NAMES or name.endswith(CODE + MARKUP):
                yield os.path.join(path, name)


SENTENCE = re.compile(r"""["'][^"']{12,}["']""")
INLINE_CODE = re.compile(r"`[^`]+`")      # a name in `backticks` is a name, not prose


def is_prose(line: str, suffix: str, in_docstring: bool) -> bool:
    """Where German words mean text rather than code.

    Markup and data files are prose throughout. In Python and shell that is comments,
    docstrings and longer string literals — the latter because the user-visible report
    text lives there. Bare identifiers (`world = auftrag.get(...)`) stay unscanned: a
    few German-derived names are API in this repository and would only produce noise. For the
    same reason `text in backticks` is dropped everywhere — in the contracts that is how a
    name is quoted, and the alias tables of §6.11 list the deprecated spellings as names.
    """
    if suffix in MARKUP:
        return True
    stripped = line.strip()
    return stripped.startswith("#") or in_docstring or bool(SENTENCE.search(line))


def main(argv):
    quiet = "--quiet" in argv
    hits = []
    for path in sorted(files()):
        suffix = os.path.splitext(path)[1].lower()
        try:
            text = open(path, encoding="utf-8").read()
        except (OSError, UnicodeDecodeError):
            continue
        in_docstring = False
        for nr, line in enumerate(text.splitlines(), 1):
            if suffix in CODE:
                in_docstring = doc_status(line, in_docstring)
            if UMLAUT.search(line):
                hits.append((path, nr, "umlaut", line.strip()[:76]))
            elif is_prose(line, suffix, in_docstring):
                words = {w.lower() for w in DE_WORD.findall(INLINE_CODE.sub("", line))}
                if len(words) >= 2:
                    hits.append((path, nr, "german words", line.strip()[:76]))
    if not quiet:
        for path, nr, reason, text in hits:
            print(f"{path}:{nr}: {reason}: {text}")
    print(f"langcheck: {len(hits)} leftover German spots in "
          f"{len({t[0] for t in hits})} files")
    return 1 if hits else 0


def doc_status(line: str, in_docstring: bool) -> bool:
    """Track whether we are inside a triple-quoted string; docstrings count as prose."""
    quotes = line.strip().count('"""') + line.strip().count("'''")
    return in_docstring if quotes % 2 == 0 else not in_docstring


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
