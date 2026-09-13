#!/usr/bin/env python3
"""Identifier guard: the names in this code base are English (CONTRACT §1, rule 3).

`langcheck.py` reads the prose of this repository; this tool reads its *names*. Both run as steps
in `tools/check.sh`, because German returns with every new feature — and after the migration of
2024-09 it would otherwise be back within a semester without anyone noticing.

Four checks over every Python file under `SCAN`:

  * identifiers built from German words, read from the ast fields `name`, `attr` and `arg` — that is
    every class, function, method and import that is defined here, every parameter and keyword
    argument, and every attribute read or set on an object;
  * the names a file assigns at module level — `SPUR_LEBEN = 4.0` and its siblings, which the three
    fields above cannot see because an assignment target is an `ast.Name`. They are not locals: a module
    constant is what an `import` elsewhere reads, so it is API. Closing this gap found thirteen of them
    (`KF_SICHERHEIT`, `SPUR_LEBEN`, `BIAS_PROBEN`, `RICHTUNGEN`, `VERJAEHT`, and seven in
    `tools/worldpic.py`); all renamed, `UNTER` and `POLSTER` among them. One of the thirteen was imported
    by a test — which is the reason this check sits in the release gate rather than in a review comment.
  * non-ASCII identifiers — umlauts never belong in a name here;
  * short bare strings that are data (dict keys, option names, enum-like values) in German.

**Local variables are counted on every run, and fail it only with `--locals`.** `ast.Name` nodes inside a
function body — the `wand = ` of a loop and every read of it — are not errors by default, and that is
stated rather than hidden. The count is printed in the summary line of every run (the day the module-level
check was added it read 294 spots in 56 identifiers, most of them in `student/solution.py`,
`student/kf_solution.py` and `student/kf_template.py` — the three files a submitted node is written
against), so the size of the open end is something the gate shows instead of something one person
remembers. Renaming them changes code the practicum reads aloud, with the grading runs of both experiments
hanging off it: a package of its own, not a line in a checker. Until that package exists this tool is
narrow about locals and loud about their number; CONTRACT §6.11 and the rules section of the README
repeat the same scope, so change all three together.

What stays German is the documented exception list in CONTRACT §1 and is encoded here as
`ALLOW_TOKENS` plus the two compatibility maps read from the code itself:

  * `mecanum_lab.tasks._LEGACY_KEYS`   — keys of an old config/tasks.json,
  * `DEPRECATED` in `launch/*.launch.py` — launch arguments from printed handouts.

Their *values* are German on purpose, so the tool collects them and ignores them.

    python3 tools/germanids.py            # whole repository, summary and every hit
    python3 tools/germanids.py --quiet    # only the summary line
    python3 tools/germanids.py --locals   # the counted locals become failures too
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
# `tol` was in this list and is not: TOL_XY and TOL_TH are English abbreviations for tolerance,
# and the detector has no idea where an abbreviation comes from. Words were added here as they
# were found in the code, never as a list of everything German — a vocabulary that guesses reports
# half the file and gets switched off.
GERMAN = set("""
abgabe abstaende abstand aktuelle anfang angabe anzahl anzeige aufbau aufgabe auftrag
aufzeichnung bahn baum befehl beschreibung beschreibungen betrag bild boden breite datei
dateien dauer drehrate drehung dunkel ebene ecke einlauf einschub einzel einzeln ende erfolg
fahrbar farbe fehler feld fenster folgen freiheit freiheitsfeld gierrate groesse grund
grundlinie gruppe gruppen halt haltezeit hebe hilfe hilfen indizes intervall kabel kamera unter
kandidat kante kanten karte karten knoten kontakt kontakteweg kurz laenge lauf laufe leben
legende leib leise leiste liste loesung luecke masse maßstab meldung meldungen menge messen
messt messung messungen messwert messwerte mit mitte nachfahren nah namen offen ordner
ordnung pfad pfade platte polster proben profil pruefe pruefen prueft punkt punkte quadrat
quer rahmen rand reihe reihenfolge richtung richtungen roboter rolle rollen rueck rueckgabe
schaetzung schatten schatzung schluss schrift schritt schritte schwelle seit seitlich
sekunden sensorik sicherheit skala skalen spalte spalten statt stelle stellen stil streifen
strich stufe teil teile textzeilen titel treffer umgebung umweg verbunden verjaeht vorwaerts
vorzeichen waehrend wahr wahrheit wand wandern wanduhr weg welt welten wert werte wiederhole
wiederholung wirklich zaehlen zaehler zahl zeichen zeile zeilen zeit zeitpunkt zellen ziel
zielwinkel zonen zurueck zusammenfassung zustand
""".split())
# Names that are allowed to read as German although the detector sees them: the documented API
# exceptions of CONTRACT §1 and §6.11 (the wheel labels as printed on the robot, the task groups and
# the task ids one types on the command line).
#
# Only `quadrat` survives. Whether an entry does anything at all is decidable: a name is reported
# over the words in `GERMAN`, and of the documented exceptions only that task id is a word the list
# knows — `vl`, `vr`, `hl`, `hr`, `fl`, `fr`, `rl`, `rr`, `kinematik`, `korridor`, `anfahrt`,
# `versuche`, `alle`, `kf`, `odom`, `gps`, `imu`, `tf`, `json`, `csv` are not in it, so listing them
# guarded nothing. An allowlist that cannot bite is worse than a short one: the next person reads it
# as "these names are checked and allowed" and stops looking. If a task is ever renamed to a word
# that _is_ in `GERMAN`, its id belongs here and nowhere else.
ALLOW_TOKENS = {"quadrat"}

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


def local_hits(path: str, compat: set) -> list:
    """German names in `ast.Name` outside the module level — the debt this tool does not fail on.

    Reported, not ignored: the count belongs on the same line the release gate prints, so the size of the
    remaining migration is visible every run instead of being remembered by one person. `--locals` turns
    them into failures; until the rename package has been done, it would fail on code that works.
    """
    try:
        tree = ast.parse(open(path, encoding="utf-8").read(), path)
    except SyntaxError:
        return []
    allowed = ALLOW_TOKENS | compat_values(path)
    module_level = set()
    for stmt in tree.body:
        if isinstance(stmt, (ast.Assign, ast.AugAssign)):
            targets = list(stmt.targets)
        elif isinstance(stmt, ast.AnnAssign):
            targets = [stmt.target]
        else:
            targets = []
        for target in targets:
            for node in ast.walk(target):
                if isinstance(node, ast.Name):
                    module_level.add(node.lineno)
    out = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Name) and node.lineno not in module_level:
            bad = tokens(node.id) - allowed
            if bad & GERMAN:
                out.append((path, node.lineno, f"{node.id}: {', '.join(sorted(bad & GERMAN))}"))
    return out


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

    for stmt in tree.body:                              # module level: what this file hands to the rest
        if isinstance(stmt, (ast.Assign, ast.AugAssign)):
            targets = list(stmt.targets)
        elif isinstance(stmt, ast.AnnAssign):
            targets = [stmt.target]
        else:
            targets = []
        for target in targets:
            for node in ast.walk(target):
                if isinstance(node, ast.Name):
                    report(node.id, node)
        if isinstance(stmt, (ast.ClassDef, ast.FunctionDef, ast.AsyncFunctionDef)):
            report(stmt.name, stmt)                     # already covered below, deduplicated by `seen`

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
    check_locals = "--locals" in argv
    compat = all_compat_values()
    hits, locals_ = [], []
    for folder in SCAN:
        for base, dirs, files in os.walk(os.path.join(ROOT, folder)):
            dirs[:] = [d for d in dirs if d not in SKIP]
            for name in sorted(files):
                if name.endswith(".py"):
                    path = os.path.join(base, name)
                    hits += hits_for(path, compat)
                    locals_ += local_hits(path, compat)
    if check_locals:
        hits += locals_
    if not quiet:
        for path, nr, text in hits:
            print(f"{os.path.relpath(path, ROOT)}:{nr}: {text}")
    spots = {hit[2].split(":")[0] for hit in locals_}
    print(f"germanids: {len(hits)} German identifiers in "
          f"{len({h[0] for h in hits})} files (allowlist: {len(ALLOW_TOKENS)} "
          f"{'token' if len(ALLOW_TOKENS) == 1 else 'tokens'}, "
          "the two compatibility maps read from the code; "
          f"locals: {len(locals_)} spots in {len(spots)} names, "
          f"{'checked' if check_locals else 'reported only — pass --locals to fail on them'})")
    return 1 if hits else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
