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


# ---------------------------------------------------------------- the grading rubric table
#
# Two pages after the thresholds the sheet says how the points are awarded. It used to promise
# "T2 square path (completion 20, driving quality 10)" and "T3 goal without contact (target error 20,
# clearance 10)". The grader has no such split: `grade._eval_mission` awards `points` when every limit of
# the task holds and 0.0 when one of them does not. Only T1 is split — `_eval_phase` gives
# points / len(phases) per phase — so 10/10/10 is real and stays printed. The behaviour test below is the
# half that reads the code instead of the numbers: it satisfies one criterion of the real `quadrat` task
# and violates another, and asserts the award is 0.0.

RUBRIC_TASKS = ("kinematik", "quadrat", "korridor", "gps_anfahrt")


def rubric_rows():
    src = open(TEX, encoding="utf-8").read()
    start = src.index(r"\subsection*{Grading rubric}")
    block = src[start:src.index(r"\end{table}", start)]
    rows = {}
    for line in block.splitlines():
        for task_id in RUBRIC_TASKS:
            if line.startswith(f"T{RUBRIC_TASKS.index(task_id) + 1} "):
                rows[task_id] = re.sub(r"\s+", " ", line).strip()
    return rows


def test_rubric_points_are_the_points_of_the_task_and_sum_to_100():
    by_id = {a["id"]: a for a in task_list()}
    rows = rubric_rows()
    assert set(rows) == set(RUBRIC_TASKS), "the rubric must list exactly the experiment-1 tasks"
    total = 0
    for task_id, line in rows.items():
        points = f"{by_id[task_id]['points']:g}"
        assert re.search(rf"&\s*{re.escape(points)}(\s+or\s+0)?\s*&", line), (
            f"rubric row {task_id} does not carry {points}: {line}")
        total += by_id[task_id]["points"]
    assert total == 100, "T1+T2+T3+T4 are 100 autograder points (90 without the T4 bonus)"


def test_rubric_promises_no_partial_credit_where_the_grader_has_none():
    by_id = {a["id"]: a for a in task_list()}
    rows = rubric_rows()
    for task_id in ("quadrat", "korridor", "gps_anfahrt"):
        line = rows[task_id]
        criterion = line.split("&")[0]
        digits = re.findall(r"\d+(?:\.\d+)?", re.sub(r"^T\d+", "", criterion))
        assert not digits, f"{task_id}: the rubric splits points ({digits}) but the grader awards all-or-0"
        assert "at once" in line or "same rule" in line, (
            f"{task_id}: the row must say the award is every limit at once, not just drop the numbers")
    for task_id in ("quadrat", "korridor", "gps_anfahrt"):
        assert "or 0" in rows[task_id], f"{task_id}: the Points cell must print '… or 0'"
    t1 = rows["kinematik"]
    per_phase = by_id["kinematik"]["points"] / len(by_id["kinematik"]["phases"])
    assert f"{per_phase:g} P each" in t1, (
        "T1 really is split by phase (points / len(phases)); keep it printed, do not harmonise it away")
    assert "& 30 &" in t1 and "or 0" not in t1, "T1's row stays 30: a phase can pass while another fails"


def test_award_is_all_criteria_at_once_not_one_criterion_at_a_time():
    """The real `quadrat` task, one criterion met and one violated: the points are 0.0, not 20."""
    import json as _json
    from mecanum_lab.grade import Grader
    from mecanum_lab.stub import StubBus
    from mecanum_lab.types import Odom

    with open(TASKS_FILE, encoding="utf-8") as fh:
        task = next(a for a in _json.load(fh)["tasks"] if a["id"] == "quadrat")
    path_limit = float(task["path_max"])

    def grade_met_one_and_violated_one(m_per_s):
        bus = StubBus("test")
        start = Odom(t=0.0, x=2.0, y=2.0, theta=0.0)
        bus.publish("/alice/odom", start)
        gr = Grader("alice", task["id"], bus, {"tasks": [task], "order": [task["id"]]},
                    world_info={"name": "production", "size": [24, 16], "goal": None,
                                "spawns": [[2.0, 2.0, 0.0]], "walls": 8}).start()
        t, dt = 0.0, 0.02
        while not gr.tick(dt) and t < 40.0:
            t += dt
            bus.publish("/alice/odom", start)                  # pose never moves: closure error 0.0 m
            bus.publish("/sim/robots", _json.dumps([{
                "name": "alice", "index": 0, "contacts": 0, "distance": m_per_s * t,
                "mission": "running" if t < 20.0 else "done", "mode": "wheels", "task": "quadrat"}]))
        return gr.report()["tasks"][0]

    good = grade_met_one_and_violated_one(0.35)               # 7 m: inside path_min..path_max
    assert good["passed"] and good["points"] == task["points"], good["reason"]

    bad = grade_met_one_and_violated_one(0.35 + (path_limit + 1.0) / 20.0)     # 13 m of path
    assert not bad["passed"] and bad["points"] == 0.0, (
        f"a task that meets closure and touches no wall still awards points: {bad}")
    assert "path" in bad["reason"] and "closure" not in bad["reason"], (
        f"the report should name the one limit that failed: {bad['reason']}")


def test_the_front_page_carries_no_exercise_text():
    """What a task asks lives in `config/tasks.json` and on the printed sheet — not on the front page.

    The front page used to paraphrase the tasks, and a paraphrase of a limit is that limit written twice,
    which is the drift this file exists to catch — one station before the handout. The exercises are also
    on their way into a repository of their own, while this one is the simulator, so what the README keeps
    is the *pointer*: the data, `./lab docs` next to a running machine, and the sheet that prints it.

    Checked by taking the first words of each task's own `text`, and its title: either one on the front
    page means a task sheet was copied back out of the JSON.
    """
    with open(os.path.join(ROOT, "README.md"), encoding="utf-8") as fh:
        readme = fh.read()
    flat = " ".join(readme.split())
    with open(TASKS_FILE, encoding="utf-8") as fh:
        tasks = json.load(fh)["tasks"]

    for task in tasks:
        words = " ".join((task.get("text") or "").split())
        assert words, f"{task['id']} has no `text` — the exercises have no source any more"
        piece = " ".join(words.split()[:9])
        assert piece not in flat, f"{task['id']} is described on the front page again: {piece!r}"
        assert task["title"] not in flat, f"the front page pastes the task sheet: {task['title']!r}"

    for pointer in ("config/tasks.json", "./lab docs", "docs/praktikum/anleitung.tex",
                    "docs/praktikum/kalman.tex"):
        assert pointer in readme, f"the front page lost the pointer to {pointer}"
