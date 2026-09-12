#!/usr/bin/env python3
"""Language guard: written prose in this repository is English (CONTRACT section 1, rule 3).

Runs headless and without ROS. Two detectors, because German creeps back in two ways:

  * umlauts and sharp s — no identifier, topic, unit or file name in this project uses
    them, so any hit is leftover German text;
  * German function words in *prose* — comments, docstrings, Markdown and LaTeX body.
    Only prose is scanned for words: German-derived identifiers (sim_profil, `wenn` as a
    local, the launch arguments `sekunden` and `aufgabe`) are API and stay as they are.

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
MARKUP = (".md", ".tex", ".txt", ".rst", ".makefile", ".json", ".xml")
NAMES = {"lab", "Makefile", "makefile"}
SELF = os.path.abspath(__file__)

UMLAUT = re.compile(r"[äöüÄÖÜß]")
DE_WORD = re.compile(
    r"\b(der|die|das|und|mit|für|wird|nicht|oder|wenn|aber|auch|einem|einen|einer|diesem|"
    r"diesen|dieser|diese|dieses|weil|dann|sonst|hier|dort|schon|immer|nur|sehr|mehr|"
    r"kann|muss|musste|soll|werden|wurde|haben|hatte|gibt|machen|meldet|setzt|liegt|"
    r"steht|falsch|richtig|zuerst|danach|vorher|auftrag|aufgaben|roboter|messung|"
    r"messungen|schätzung|simulationszeit|sensorprofil|bewerter|studierende|welt|halle|"
    r"wand|wanduhr|zustand|fenster|wert|datei|einmal|zwei|drei|vier|erste|zweite|"
    r"letzte|versuch|anleitung|arbeitsblatt|musterlösung)\b", re.I)


def dateien():
    for pfad, ordner, dateien in os.walk("."):
        ordner[:] = [o for o in ordner if o not in EXCLUDE]
        for name in dateien:
            weg = os.path.abspath(os.path.join(pfad, name))
            if weg == SELF:
                continue
            if name in NAMES or name.endswith(CODE + MARKUP):
                yield os.path.join(pfad, name)


SENTENCE = re.compile(r"""["'][^"']{12,}["']""")


def ist_prosa(zeile: str, endung: str, in_doc: bool) -> bool:
    """Where German words mean text rather than code.

    Markup and data files are prose throughout. In Python and shell that is comments,
    docstrings and longer string literals — the latter because the user-visible report
    text lives there. Bare identifiers (`welt = auftrag.get(...)`) stay unscanned: a
    few German-derived names are API in this repository and would only produce noise.
    """
    if endung in MARKUP:
        return True
    stripped = zeile.strip()
    return stripped.startswith("#") or in_doc or bool(SENTENCE.search(zeile))


def main(argv):
    leise = "--quiet" in argv
    treffer = []
    for pfad in sorted(dateien()):
        endung = os.path.splitext(pfad)[1].lower()
        try:
            text = open(pfad, encoding="utf-8").read()
        except (OSError, UnicodeDecodeError):
            continue
        in_doc = False
        for nr, zeile in enumerate(text.splitlines(), 1):
            if endung in CODE:
                in_doc = doc_status(zeile, in_doc)
            if UMLAUT.search(zeile):
                treffer.append((pfad, nr, "umlaut", zeile.strip()[:76]))
            elif ist_prosa(zeile, endung, in_doc):
                woerter = {w.lower() for w in DE_WORD.findall(zeile)}
                if len(woerter) >= 2:
                    treffer.append((pfad, nr, "german words", zeile.strip()[:76]))
    if not leise:
        for pfad, nr, grund, text in treffer:
            print(f"{pfad}:{nr}: {grund}: {text}")
    print(f"langcheck: {len(treffer)} leftover German spots in "
          f"{len({t[0] for t in treffer})} files")
    return 1 if treffer else 0


def doc_status(zeile: str, in_doc: bool) -> bool:
    """Track whether we are inside a triple-quoted string; docstrings count as prose."""
    anzahl = zeile.strip().count('"""') + zeile.strip().count("'''")
    return in_doc if anzahl % 2 == 0 else not in_doc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
