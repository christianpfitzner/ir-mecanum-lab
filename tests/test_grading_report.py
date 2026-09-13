"""The grading report has to explain itself: the measured number beside the required one.

A run that scores 0/90 is the run a student reads. If its lines name neither the number nor the
threshold it was compared against, the report is a verdict and not feedback — which is what the K3
and T4 questions kept coming down to. So these tests pin the *shape* of the output, not the points:
every criterion that was applied shows up as `name value ≤ limit`, the limit is read from the same
task dictionary the check used, and a step that could not be measured at all says so in one piece.
"""
import re

from mecanum_lab import grade, tasks
from mecanum_lab.stub import StubBus

TASKS = tasks.load_tasks()
GPS_RUN = [a for a in TASKS["tasks"] if a["id"] == "gps_anfahrt"][0]
ROW = re.compile(r"^(\S+) (-?[\d.]+) ([≤≥]) (-?[\d.]+)$")


def mission_measured(**over):
    """What `_eval_mission` measures on a good GPS run, with single criteria spoiled by keyword."""
    numbers = {"time": 40.0, "path": 13.0, "contacts": 0, "target_error": 0.18,
               "target": [5.0, 5.0]}
    numbers.update(over)
    return numbers


def report_of(argv):
    """Grade against the bundled fake robot — the whole of experiment 1, in a fifth of a second."""
    import contextlib
    import io
    with contextlib.redirect_stdout(io.StringIO()) as out:
        grade.main(argv)
    return out.getvalue()


# ------------------------------------------------------------------ measured vs required


def test_every_criterion_that_was_applied_is_a_row_with_both_numbers():
    reasons, rows = grade._apply(grade.MISSION_CRITERIA, mission_measured(), GPS_RUN)
    assert reasons == []
    assert all(ROW.match(row) for row in rows)                # every row reads "name number ≤ number"
    assert len(rows) == 5                                    # target, path both ways, contacts, time
    assert "target_error 0.18 ≤ 0.45" in rows                # the limit is the one in tasks.json
    assert "time 40 ≤ 90" in rows
    assert not any(row.startswith("closure") for row in rows)   # not measured → not a criterion here


def test_a_failing_row_and_its_verdict_quote_the_same_two_numbers():
    reasons, rows = grade._apply(grade.MISSION_CRITERIA, mission_measured(target_error=0.9), GPS_RUN)
    assert reasons == [f"target missed: 0.9 m > {GPS_RUN['target_max']} m"]
    assert "target_error 0.9 ≤ 0.45" in rows                 # the row stays the requirement, not the
    assert all("0.9" in r and "0.45" in r for r in rows[:1])  # outcome, so the two can be compared


def test_a_heading_error_the_wrong_way_round_is_still_a_heading_error():
    """`yaw_max_deg` bounds the magnitude, as `dy_betrag_max` does and as T2's text says.

    The comparison used to be signed, so turning 25° past the start heading on the short side passed
    a task that calls that a violation; the signed value still reaches the report as a measurement.
    """
    numbers = mission_measured(closure=0.1, yaw_deg=-25.0)
    reasons, rows = grade._apply(grade.MISSION_CRITERIA, numbers, dict(GPS_RUN, yaw_max_deg=20.0))
    assert reasons == ["heading error 25.0° > 20.0°"]
    assert "yaw_deg 25 ≤ 20" in rows                          # the row shows what was compared
    assert numbers["yaw_deg"] == -25.0


def test_the_whole_report_shows_the_requirement_of_every_criterion():
    text = report_of(["--task", "alle"])
    required = [line.removeprefix("        required: ") for line in text.splitlines()
                if line.startswith("        required:")]
    # T2 has six criteria (closure, heading, path from both sides, contacts, time), T3 and T4 five
    # each; T1 is graded per phase and shows its rows on the phase lines. A new criterion in
    # config/tasks.json has to appear here, because that is the line a student reads.
    assert len(required) == 16
    for row in required:
        assert ROW.match(row), row
    # T2's square run is the one task graded on closure; its limit is 0.25 m in tasks.json.
    assert any(row.startswith("closure") and row.endswith("≤ 0.25") for row in required)


def test_a_task_that_passed_prints_what_it_had_to_meet():
    text = report_of(["--task", "kinematik"])
    assert "all tasks meet requirements" in text
    assert "≥ 0.35" in text                                   # T1's dx_min, with no failure to read it


# ------------------------------------------------------------------ an unmeasurable step


def test_a_phase_without_odometry_says_so_in_one_piece():
    """The reason was a bare string in a `"; ".join(...)`: it came out letter by letter."""
    bus = StubBus("no odometry")
    grader = grade.Grader("alice", "kinematik", bus, TASKS).start()
    for _ in range(6000):
        if grader.tick(0.02):
            break
    phases = grader.report()["tasks"][0]["phases"]
    assert phases, "the plan ran to its phase steps"
    for pid, phase in phases.items():
        assert phase["reason"] == "no odometry received", pid
        assert phase["criteria"] == []                          # nothing was compared, so: no rows
