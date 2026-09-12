#!/usr/bin/env python3
"""Identifier guard: the names in this code base are English (CONTRACT §1, rule 3).

`langcheck.py` reads the prose of this repository; this tool reads its *names*. Both run as steps
in `tools/check.sh`, because German returns with every new feature — and after the migration of
2024-09 it would otherwise be back within a semester without anyone noticing.

Three checks over every Python file:

  * identifiers (functions, methods, arguments, attributes, variables) built from German words;
  * non-ASCII identifiers — umlauts never belong in a name here;
  * short bare strings that are data (dict keys, option names, enum-like values) in German.

What stays German is the documented exception list in CONTRACT §1 and is encoded here as
`ALLOW_TOKENS` plus the two compatibility maps read from the code itself:

  * `mecanum_lab.tasks._LEGACY_KEYS`   — keys of an old config/tasks.json,
  * `DEPRECATED` in `launch/*.launch.py` — launch arguments from printed handouts.

Their *values* are German on purpose, so the tool collects them and ignores them.

    python3 tools/germanids.py            # whole repository, summary and every hit
    python3 tools/germanids.py --quiet    # only the summary line
"""
import ast
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCAN = ("mecanum_lab", "tools", "tests", "student", "launch")
SKIP = {"__pycache__", ".git", "build", "install", ".pytest_cache"}
COMPAT_MAP = {"_LEGACY_KEYS", "DEPRECATED", "DEPRECATED_OPTIONS"}
# A file that *is* a documented compatibility interface (the legacy read and its test)
# carries this marker and may then spell the German names; elsewhere they are a bug.
LEGACY_MARKER = "germanids: legacy keys"

# German vocabulary of this project. Only words that are not English words are listed — `probe`,
# `serie`, `text` and friends would report half of the code without meaning anything.
GERMAN = set("""
abgabe abstand abstaende anfang angabe anzeige anzahl aufbau aufgabe auftrag aufzeichnung baum
beschreibung beschreibungen betrag befehl datei dateien dauer drehung drehrate ebene einlauf einzel
einzeln ende ende erfolg farbe fehler fenster folgen freiheitsfeld grund grundlinie gruppe gruppen
groesse halt haltezeit hebe hilfe hilfen indizes intervall knoten kontakt kontakteweg kante kanten
laenge leib leiste liste liste lauf laufe leise loesung luecke maßstab masse meldung
meldungen messung messungen messwert messwerte mit namen ordner ordnung offen pfad pfade platte
punkt punkte profil pruefe pruefen prueft quadrat quer rahmen rand rand reihe reihenfolge roboter
rolle rollen rueck rueckgabe schatzung schaetzung schluss schritt schritte schrift schwelle sekunden
sensorik skala skalen spalte spalten statt stelle stellen stufe textzeilen teil teile titel treffer
tol umgebung vorwaerts vorzeichen waehrend wahr wahrheit wand wanduhr weg welten welt werte wert
wiederhole wiederholung wirklich zeit zeile zeilen zeitpunkt zahl zaehlen zaehler zusammenfassung
zustand zonen ziel
""".split())
# Documented API exceptions (CONTRACT §1 and §6.11): the wheel labels as printed on the robot,
# the task groups and task ids one types on the command line.
ALLOW_TOKENS = {"vl", "vr", "hl", "hr", "fl", "fr", "rl", "rr",           # wheel labels
                "alle", "beide", "versuch", "versuche", "alle_versuche",  # task groups
                "kinematik", "quadrat", "korridor", "anfahrt",            # task ids
                "kf", "gps", "imu", "odom", "tf", "json", "csv"}

SNAKE = re.compile(r"^[a-z][a-z0-9_]*$")
DATA_STRING = re.compile(r"^[a-z][a-z0-9_.\-]*$")


def tokens(name: str) -> set:
    """The words of an identifier: snake_case parts plus camelCase parts."""
    parts = set(re.split(r"[_\d]+", name.lower()))
    parts |= {p.lower() for p in re.findall(r"[A-ZÄÖÜ][a-zäöü]+|[a-zäöü]+", name)}
    return {p for p in parts if p}


def compat_values(path: str) -> set:
    """The German names of the compatibility maps in this file — German is their content there."""
    found = set()
    try:
        tree = ast.parse(open(path, encoding="utf-8").read(), path)
    except (OSError, SyntaxError):
        return found
    for node in ast.walk(tree):
        if not isinstance(node, ast.Assign):
            continue
        if not any(getattr(t, "id", "") in COMPAT_MAP for t in node.targets):
            continue
        try:
            data = ast.literal_eval(node.value)
        except ValueError:
            continue
        entries = data.items() if isinstance(data, dict) else list(enumerate(data))
        for key, value in entries:
            for word in ([value] if isinstance(value, str) else [str(value)]):
                found.add(word)                       # the whole key, not only its parts
                found |= tokens(word)
            found |= tokens(str(key))
    return found


def all_compat_values() -> set:
    """German names from the compatibility maps of every file — they are API all over the tree."""
    found = set()
    for folder in SCAN:
        for base, dirs, files in os.walk(os.path.join(ROOT, folder)):
            dirs[:] = [d for d in dirs if d not in SKIP]
            for name in sorted(files):
                if name.endswith(".py"):
                    found |= compat_values(os.path.join(base, name))
    return found


def hits_for(path: str, compat: set) -> list:
    src = open(path, encoding="utf-8").read()
    try:
        tree = ast.parse(src, path)
    except SyntaxError as exc:
        return [(path, getattr(exc, "lineno", 0), f"syntax error: {exc.msg}")]
    allowed = ALLOW_TOKENS | compat_values(path)
    if LEGACY_MARKER in src:
        allowed |= compat                    # this file documents a legacy interface
    lines = src.splitlines()
    out = []

    def report(name, node):
        bad = tokens(name) - allowed
        if bad & GERMAN:
            out.append((path, getattr(node, "lineno", 0), f"{name}: {', '.join(sorted(bad & GERMAN))}"))
        elif name and not name.isascii():
            out.append((path, getattr(node, "lineno", 0), f"{name}: non-ascii identifier"))

    seen = set()
    for node in ast.walk(tree):
        for field in ("name", "attr", "arg"):
            value = getattr(node, field, None)
            if isinstance(value, str) and (value, getattr(node, "lineno", 0)) not in seen:
                seen.add((value, getattr(node, "lineno", 0)))
                report(value, node)
        if isinstance(node, ast.Constant) and isinstance(node.value, str):
            text = node.value
            if DATA_STRING.match(text) and SNAKE.match(text.replace("-", "_")):
                bad = tokens(text) - allowed
                if bad & GERMAN:
                    out.append((path, node.lineno, f"'{text}': german data key"))
    unique = []
    for hit in out:
        if hit not in unique:
            unique.append(hit)
    return unique


def main(argv) -> int:
    quiet = "--quiet" in argv
    compat = all_compat_values()
    hits = []
    for folder in SCAN:
        for base, dirs, files in os.walk(os.path.join(ROOT, folder)):
            dirs[:] = [d for d in dirs if d not in SKIP]
            for name in sorted(files):
                if name.endswith(".py"):
                    hits += hits_for(os.path.join(base, name), compat)
    if not quiet:
        for path, nr, text in hits:
            print(f"{os.path.relpath(path, ROOT)}:{nr}: {text}")
    print(f"germanids: {len(hits)} German identifiers in "
          f"{len({h[0] for h in hits})} files (allowlist: {len(ALLOW_TOKENS)} tokens, "
          "the two compatibility maps read from the code)")
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
