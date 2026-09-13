"""W8: the task table of the student handout is a copy of `config/tasks.json`, so it is checked as one.

`anleitung.tex` prints its limits under the sentence "All thresholds stand in config/tasks.json" — and
six of the numbers in that table disagreed with the file this package started from: `yaw_min` 1.2 was
printed as "> 0.9", `closure_max` 0.25 as 0.20, `yaw_max_deg` 20 as 15, the path band 3.0…12.0 as
3.2…9.0, `target_max` 0.30 as 0.25 and 0.45 as 0.25, and the T4 row pointed at "your own start pose"
while the task grades the world's last spawn pose ("deliberately not your own start pose", says its own
`text`). A number written down twice drifts, so this test reads the numbers out of the LaTeX and compares
them with the JSON: change a threshold without editing the handout and this fails — which is the point.

Two rules keep the test honest. A limit that the table deliberately does not print has to be listed in
`NOT_IN_THE_TABLE` with a reason (below), so nothing is skipped silently; and the four experiment-1 tasks
have to appear in the table at all, so a fifth task cannot be forgotten either. Nothing here runs the
grader: this is about the promise the printed sheet makes.
"""
import json
import os
import re

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TEX = os.path.join(ROOT, "docs", "praktikum", "anleitung.tex")
TASKS_FILE = os.path.join(ROOT, "config", "tasks.json")

# (task, the words the table puts in front of the number, where the number comes from, how the table
# writes it, words behind it). Every entry is one criterion of one task; `pick()` reads the JSON path.
CHECKS = [
    ("kinematik", r"\(\Delta x>", "phases.0.expect.dx_min", "{:g}", ""),
    ("kinematik", r"\(\Delta y>", "phases.1.expect.dy_min", "{:g}", ""),
    ("kinematik", r"\(\Delta\theta>", "phases.2.expect.yaw_min", "{:g}", ""),
    ("kinematik", "cross-axis error \\(<", "phases.0.expect.dy_abs_max", "{:g}", ""),
    ("kinematik", "turning \\(<", "phases.2.expect.dx_abs_max", "{:g}", ""),
    ("quadrat", "four sides of \\(", "side", "{:.1f}", ""),
    ("quadrat", "completion error \\(<", "closure_max", "{:g}", ""),
    ("quadrat", r"\(\Delta\theta<", "yaw_max_deg", "{:.0f}", ""),
    ("quadrat", "path length \\(", "path_min", "{:.1f}", ""),
    ("quadrat", r"\ldots", "path_max", "{:.1f}", ""),
    ("quadrat", "", "contacts_max", "{:.0f}", " contacts"),
    ("quadrat", "under \\(", "timeout", "{:.0f}", ""),
    ("korridor", "target error \\(<", "target_max", "{:.2f}", ""),
    ("korridor", "", "contacts_max", "{:.0f}", " contacts"),
    ("korridor", "lateral clearance \\(>", "lateral_min", "{:g}", ""),
    ("korridor", r"\(|\omega|<", "straight_omega_max", "{:g}", ""),
    ("korridor", "under \\(", "timeout", "{:.0f}", ""),
    ("gps_anfahrt", "error \\(<", "target_max", "{:g}", ""),
    ("gps_anfahrt", "", "contacts_max", "{:.0f}", " contacts"),
    ("gps_anfahrt", "under \\(", "timeout", "{:.0f}", ""),
]

# Every limit a task names must either be checked above or be named here with the reason the printed
# sheet leaves it out. A reason that stops being true is caught by test_no_dead_exceptions.
NOT_IN_THE_TABLE = {
    ("kinematik", "phases.0.expect.yaw_abs_max"):
        "the heading residual of the two straight phases (0.2 rad) is not on the sheet; the row prints "
        "the three distances a student drives for",
    ("kinematik", "phases.1.expect.dx_abs_max"):
        "the same number as phases.0.expect.dy_abs_max, printed once as 'cross-axis error'",
    ("kinematik", "phases.1.expect.yaw_abs_max"):
        "see phases.0.expect.yaw_abs_max",
    ("kinematik", "phases.2.expect.dy_abs_max"):
        "the same number as phases.2.expect.dx_abs_max, printed once as 'turning <…'",
    ("kinematik", "hold_time"):
        "the pause between the phases is pacing, not a limit a student can miss",
    ("korridor", "path_min"):
        "the detour band is a sanity bound on the path length, not a criterion the row advertises",
    ("korridor", "path_max"): "see path_min",
    ("gps_anfahrt", "path_min"): "see korridor.path_min",
    ("gps_anfahrt", "path_max"): "see korridor.path_max",
}

# Keys that bound a mission (`MISSION_CRITERIA` in grade.py reads exactly these), plus `side` and the
# per-phase `expect` keys of T1.
MISSION_KEYS = ("target_max", "closure_max", "yaw_max_deg", "path_min", "path_max",
                "contacts_max", "lateral_min", "timeout", "straight_omega_max", "side")


def task_list():
    with open(TASKS_FILE, encoding="utf-8") as fh:
        return json.load(fh)["tasks"]


def first_experiment():
    return [a for a in task_list() if a.get("kind") != "kf"]


def pick(a, path):
    """`phases.0.expect.dx_min` out of the task dict — numbers come from the JSON, never from here."""
    step = a
    for part in path.split("."):
        step = step[int(part)] if isinstance(step, list) else step[part]
    return step


def rows():
    """The `tab:aufgaben` region as one string per task row, whitespace flattened."""
    src = open(TEX, encoding="utf-8").read()
    start = src.index(r"\label{tab:aufgaben}")
    table = src[start:src.index(r"\bottomrule", start)]
    ids = [a["id"] for a in first_experiment()]
    marks = []
    for task_id in ids:
        anchor = r"\thema{" + task_id.replace("_", r"\_") + r"}"
        assert anchor in table, f"{task_id} has no row in tab:aufgaben"
        marks.append((table.index(anchor), task_id))
    marks.sort()
    out = {}
    for i, (pos, task_id) in enumerate(marks):
        end = marks[i + 1][0] if i + 1 < len(marks) else len(table)
        out[task_id] = re.sub(r"\s+", " ", table[pos:end])
    return out


def plain(row):
    r"""A row as it reads on the page: `\emph{not} your own start pose` is one phrase, not two."""
    text = re.sub(r"\\(?:emph|textbf|texttt|mathsf|thema|ordner|kommando)\{([^{}]*)\}", r"\1", row)
    text = re.sub(r"\\[a-zA-Z]+", " ", text)
    return re.sub(r"\s+", " ", text.replace("{", "").replace("}", "").replace("\\", ""))


def test_table_lists_every_experiment_1_task():
    assert set(rows()) == {a["id"] for a in first_experiment()}


def test_every_printed_number_is_the_number_in_the_json():
    by_id = {a["id"]: a for a in task_list()}
    table = rows()
    for task_id, prefix, path, fmt, suffix in CHECKS:
        value = fmt.format(pick(by_id[task_id], path))
        pattern = re.escape(prefix) + re.escape(value) + re.escape(suffix) + r"(?![0-9.])"
        assert re.search(pattern, table[task_id]), (
            f"{task_id}: the handout does not print {path} = {value} after {prefix!r}")


def test_every_limit_of_a_task_is_printed_or_named_as_an_exception():
    table = rows()
    covered = {(task_id, path) for task_id, _, path, _, _ in CHECKS}
    for a in first_experiment():
        limits = [k for k in MISSION_KEYS if k in a]
        limits += [f"phases.{i}.expect.{key}"
                   for i, ph in enumerate(a.get("phases") or []) for key in ph.get("expect", {})]
        for key in limits:
            assert (a["id"], key) in covered or (a["id"], key) in NOT_IN_THE_TABLE, (
                f"{a['id']}: limit {key} is neither checked against the table nor declared as left "
                "out — put it in tab:aufgaben or add the reason to NOT_IN_THE_TABLE")
    assert table                                    # a table with no row would pass everything else


def test_no_dead_exceptions():
    """An exception that no longer matches a real limit is a lie about the sheet, so it is a failure."""
    by_id = {a["id"]: a for a in task_list()}
    covered = {(task_id, path) for task_id, _, path, _, _ in CHECKS}
    for (task_id, path), reason in NOT_IN_THE_TABLE.items():
        assert reason, f"{task_id}/{path}: an exception needs a reason"
        assert (task_id, path) not in covered, f"{task_id}/{path}: listed twice, check and exception"
        try:
            pick(by_id[task_id], path)
        except (KeyError, IndexError):
            raise AssertionError(f"{task_id}/{path}: no such limit in config/tasks.json any more")


def test_points_are_the_points_of_the_task():
    by_id = {a["id"]: a for a in first_experiment()}
    for task_id, row in rows().items():
        points = f"{by_id[task_id]['points']:g}"
        assert re.search(rf"&\s*{re.escape(points)}\s*\\{{2}}(?!\\)", row), (
            f"{task_id}: the table does not end in '& {points} \\\\'")


def test_t4_row_says_last_spawn_pose_and_not_own_start_pose():
    """The trap is the point of T4: the target is `spawns[-1]`, deliberately not where the robot began."""
    a = next(x for x in task_list() if x["id"] == "gps_anfahrt")
    assert a["target"] == "spawn" and int(a["target_index"]) == -1
    row = plain(rows()["gps_anfahrt"])
    assert "last spawn pose" in row
    assert "not your own start pose" in row, "the row must keep the negation, it is the whole exercise"
    assert not re.search(r"(?<!not )your own start pose", row), (
        "T4 must not promise 'your own start pose' anywhere except in the sentence that rules it out")
