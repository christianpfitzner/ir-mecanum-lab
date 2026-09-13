#!/usr/bin/env python3
"""Launch-file guard: every argument a launch file reads is declared, and every declared one is
documented.

`ros2 launch launch/<file> --show-args` is the table a student reads before typing anything, and it
prints exactly one row per *declared* argument: the name, its default, its description. So a launch
file can lie there in two ways, and both cost an hour:

  * it reads an argument it never declares — then what was typed on the command line is quietly
    dropped, because an undeclared configuration is not an argument of that launch file;
  * it declares an argument without a description — then the row in `--show-args` is empty, and the
    table that was supposed to save reading the launch file does not.

    python3 tools/launchargs.py            # one line per launch file, problems spelled out
    python3 tools/launchargs.py --quiet    # only the summary line; exit 1 when something is off

Runs without ROS: the file is read with `ast` and its argument tables (`BASICS`, `SETTINGS`,
`DEPRECATED`) are evaluated as literals. That is also why this can be a step of `tools/check.sh` on
a machine with no ROS sourced. It reads the same three things `--show-args` prints — name, default,
description — so the two agree by construction and not by intention.
"""
import argparse
import ast
import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_DIR = os.path.join(ROOT, "launch")
DECLARED = "DeclareLaunchArgument"


def called(node: ast.Call) -> str:
    """Name of the function a call node invokes, with or without its module prefix (`L.X()`)."""
    return getattr(node.func, "id", "") or getattr(node.func, "attr", "")


def tables(tree: ast.Module) -> dict:
    """Module-level literal tables: {"BASICS": [(name, default, help), …], "DEPRECATED": {…}}."""
    out = {}
    for node in tree.body:
        if isinstance(node, ast.Assign) and len(node.targets) == 1 \
                and isinstance(node.targets[0], ast.Name):
            try:
                out[node.targets[0].id] = ast.literal_eval(node.value)
            except ValueError:
                pass
    return out


def render(value, known: dict) -> str:
    """Default or description of one declaration: a literal, or a variable of the loop it sits in.

    `known` maps the comprehension's loop variables to the value this argument has there, which is
    what makes `f"deprecated, use '{new}'"` printable per argument without running the launch file.
    """
    if value is None:
        return ""
    if isinstance(value, ast.Constant):
        return "" if value.value is None else str(value.value)
    if isinstance(value, ast.Name):
        return str(known.get(value.id, ""))
    if isinstance(value, ast.JoinedStr):                     # f"deprecated, use '{new}'"
        return "".join(part.value if isinstance(part, ast.Constant)
                       else render(part.value, known) if isinstance(part, ast.FormattedValue)
                       else "" for part in value.values)
    return ""


def loop_values(node: ast.comprehension, file_tables: dict) -> list:
    """[{loop variable: value}] for one `for name, default, text in BASICS` (or DEPRECATED.items())."""
    names = [t.id for t in getattr(node.target, "elts", [node.target]) if isinstance(t, ast.Name)]
    source = node.iter
    if isinstance(source, ast.Call) and called(source) == "sorted" and source.args:
        source = source.args[0]
    # `DEPRECATED.items()` is a call in 3.12 and an attribute before it — both spellings unwind here.
    if isinstance(source, ast.Call) and isinstance(source.func, ast.Attribute) \
            and source.func.attr == "items":
        source = source.func.value
    if isinstance(source, ast.Attribute) and source.attr == "items":
        source = source.value
    table = file_tables.get(getattr(source, "id", ""))
    rows = []
    if isinstance(table, dict):
        rows = [[key, value] for key, value in table.items()]
    elif isinstance(table, (list, tuple)):
        rows = [list(row) if isinstance(row, (list, tuple)) else [row] for row in table]
    return [{name: (row[i] if i < len(row) else "") for i, name in enumerate(names)}
            for row in rows]


def declared_in(call: ast.Call, known: dict) -> tuple:
    """(name, default, description) of one DeclareLaunchArgument call."""
    name = render(call.args[0] if call.args else None, known)
    default = help_text = ""
    for keyword in call.keywords:
        if keyword.arg == "default_value":
            default = render(keyword.value, known)
        elif keyword.arg == "description":
            help_text = render(keyword.value, known)
    return name, default, help_text


def arguments_of(path: str) -> list:
    """[(name, default, description)] of every DeclareLaunchArgument of one launch file."""
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    file_tables = tables(tree)
    found, seen = [], set()

    def add(name, default, help_text):
        if name and name not in seen:
            seen.add(name)
            found.append((name, default, help_text))

    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or called(node) != DECLARED:
            continue
        if node.args and isinstance(node.args[0], ast.Constant):      # written out one by one
            add(*declared_in(node, {}))
    for name, default, help_text in from_loops(tree, file_tables):
        add(name, default, help_text)
    return found


def from_loops(tree: ast.AST, file_tables: dict) -> list:
    """The declarations built by a comprehension over an argument table (the kf/wifi style).

    `ast.comprehension` does not contain the element an expression builds, so this walks the
    comprehension *expressions* (`[X for … ]`) and pairs their calls with every combination of the
    loop values — one declaration per row of the table, which is what the launch file passes on.
    """
    out = []
    for node in ast.walk(tree):
        if not isinstance(node, (ast.ListComp, ast.GeneratorExp, ast.SetComp, ast.DictComp)):
            continue
        product = [{}]
        for generator in node.generators:
            values = loop_values(generator, file_tables)
        product = [{**old, **new} for old in product for new in values]
        built = [node.elt] if hasattr(node, "elt") else [node.key, node.value]
        calls = [c for part in built for c in ast.walk(part)
                 if isinstance(c, ast.Call) and called(c) == DECLARED]
        out.extend(declared_in(call, known) for known in product for call in calls)
    return out


def read_names(path: str) -> set:
    """Every argument name this file asks the launch context for, as a literal string."""
    tree = ast.parse(open(path, encoding="utf-8").read(), path)
    read = set()
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call) or not node.args:
            continue
        first = node.args[0]
        if called(node) in ("LaunchConfiguration", "read_arg") \
                and isinstance(first, ast.Constant) and isinstance(first.value, str):
            read.add(first.value)
    return read


def main(argv=None) -> int:
    e = argparse.ArgumentParser(description="check the arguments of launch/*.launch.py")
    e.add_argument("--quiet", action="store_true", help="only the summary line")
    quiet = e.parse_args(argv).quiet
    files = sorted(n for n in os.listdir(LAUNCH_DIR) if n.endswith(".launch.py"))
    problems, total = [], 0
    for name in files:
        path = os.path.join(LAUNCH_DIR, name)
        declared = arguments_of(path)
        # The deprecated aliases count as read: `read_arg()` consults them through the DEPRECATED
        # map, so their name never appears as a literal in the file.
        read = read_names(path) | set(tables(ast.parse(
            open(path, encoding="utf-8").read(), path)).get("DEPRECATED", {}).values())
        names = {entry[0] for entry in declared}
        total += len(declared)
        undocumented = [entry[0] for entry in declared if not entry[2].strip()]
        for missing in sorted(read - names):
            problems.append(f"{name}: reads '{missing}' but never declares it, so `--show-args` "
                            "does not list it and `ros2 launch` ignores the value")
        for missing in undocumented:
            problems.append(f"{name}: '{missing}' is declared without a description, which is an "
                            "empty row in `--show-args`")
        if not quiet:
            print(f"{name:20} {len(declared):3} declared, {len(declared) - len(undocumented):3} with "
                  f"one line of help, {len(read & names):3} named literally in the file")
    for line in problems:
        print("  " + line)
    print(f"launchargs: {total} arguments in {len(files)} launch files, {len(problems)} problems")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
