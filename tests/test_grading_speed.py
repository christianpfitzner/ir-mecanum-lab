"""`--speed N` and `--fixed-step`: the pace of a run is a switch, not an accident of the host.

`run_loop` used to take `min(wall clock delta, 0.25)` as its dt, so a grading run measured whatever
the machine managed: whole seconds went away at that clamp without a word, and the same seed came
out differently on a loaded host (T4 0.07…0.33 m, K3 NEES 0.09…0.65 — see CONTRACT-KF §5).

These tests drive the clock themselves instead of waiting for it: a scripted `time.monotonic()` and
a bus that does nothing but record. Then "four simulation seconds per wall second", "exactly 1/rate
per round" and "warn once, with the number" are checkable to the digit in milliseconds.
"""
import logging
import os
import re

import pytest

from mecanum_lab import node
from mecanum_lab.engine import SimEngine
from mecanum_lab.types import load_config
from mecanum_lab.worlds import load_world

RATE = 50


class ScriptedClock:
    """`time.monotonic()` as the test wants it: every read moves on by `step` seconds."""

    def __init__(self, step: float, start: float = 100.0):
        self.t, self.step, self.start = start, step, start

    def __call__(self) -> float:
        value = self.t
        self.t += self.step
        return value

    def elapsed(self) -> float:
        """What the loop saw between its first and its last read — its own wall clock."""
        return self.t - self.start - self.step


class RecordingBus:
    """Just enough bus for `run_loop`: publish records, spin is the only thing that could sleep.

    `rounds` ends the run from the outside, so a test says "five rounds" instead of hoping the
    simulation time lands on its `--seconds` limit exactly (it lands one physics step away often).
    """

    def __init__(self, rounds: int = 10 ** 6):
        self.published = []
        self.spins = []
        self.rounds = rounds

    def ok(self) -> bool:
        return len(self.spins) < self.rounds

    def pub(self, kind, robot=None):
        return lambda payload: self.published.append((kind, payload))

    def sub(self, kind, robot, cb):
        pass

    def sub_topic(self, name, cb):
        pass

    def last(self, kind, robot=None):
        return (None, 1e9)

    def spin(self, timeout: float = 0.01) -> None:
        self.spins.append(timeout)


def an_engine(cfg_extra: dict | None = None) -> SimEngine:
    cfg = load_config(None, {"gui": False, "rate": RATE, **(cfg_extra or {})})
    eng = SimEngine(load_world("arena", cfg=cfg), cfg, seed=1)
    eng.spawn("muster")
    return eng


# ---------------------------------------------------------------------------- the two switches


def test_speed_multiplies_what_the_wall_clock_gives(monkeypatch):
    """One second of the loop is worth N seconds of simulation — that is the whole flag."""
    clock, eng, bus = ScriptedClock(step=0.05), an_engine(), RecordingBus(rounds=5)
    monkeypatch.setattr(node.time, "monotonic", clock)
    node.run_loop(eng, bus, speed=4.0, hz=1000)
    # eng.t only ever moves on the fixed physics step, so that step is the tolerance here.
    assert eng.t == pytest.approx(4.0 * clock.elapsed(), abs=eng.sub_step)
    assert clock.elapsed() == pytest.approx(0.25)     # five rounds of 0.05 s of wall clock


def test_the_default_pace_is_real_time(monkeypatch):
    clock, eng, bus = ScriptedClock(step=0.04), an_engine(), RecordingBus(rounds=5)
    monkeypatch.setattr(node.time, "monotonic", clock)
    node.run_loop(eng, bus, hz=1000)
    assert eng.t == pytest.approx(clock.elapsed(), abs=eng.sub_step)


def test_fixed_steps_exactly_one_physics_step_and_sleeps_never(monkeypatch):
    """dt is 1/rate every round — the wall clock is out of the loop, so the host load is too."""
    clock, eng, bus = ScriptedClock(step=1.7), an_engine(), RecordingBus(rounds=10)
    monkeypatch.setattr(node.time, "monotonic", clock)     # would be 85x realtime if it counted
    node.run_loop(eng, bus, fixed_step=True)
    assert eng.t == pytest.approx(10 * eng.sub_step, abs=1e-9)
    assert bus.spins and set(bus.spins) == {0.0}, "fixed-step must not hand out sleep time"


def recorded_steps(eng: SimEngine) -> list:
    """Every dt the loop hands the engine — so a test can add them up."""
    asked = []
    original = eng.step

    def step(dt: float) -> None:
        asked.append(dt)
        original(dt)

    eng.step = step
    return asked


def test_both_switches_ask_for_the_same_amount_of_simulation(monkeypatch):
    """The pace changes how the simulation time is chopped up, not how much of it there is.

    Twenty rounds of 0.1 s wall clock are 2 s of simulation at speed 1; `--fixed-step` ignores the
    wall clock and so gets 20 physics steps = 0.4 s. That is the whole difference the flag makes.
    """
    plain, steady = an_engine(), an_engine()
    asked_plain, asked_fixed = recorded_steps(plain), recorded_steps(steady)
    for eng, kwargs in ((plain, {}), (steady, {"fixed_step": True})):
        monkeypatch.setattr(node.time, "monotonic", ScriptedClock(step=0.1, start=0.0))
        node.run_loop(eng, RecordingBus(rounds=20), hz=1000, **kwargs)
    assert sum(asked_plain) == pytest.approx(20 * 0.1)
    assert sum(asked_fixed) == pytest.approx(20 * plain.sub_step)
    assert set(asked_fixed) == {plain.sub_step}          # exactly 1/rate, and nothing else
    assert max(asked_plain) - min(asked_plain) < 1e-12   # every round stepped what it waited for
    assert min(asked_plain) > plain.sub_step             # in wall-clock slices, not physics slices


def test_a_run_that_cannot_keep_up_says_so_once_with_the_number(monkeypatch, caplog):
    """The old code lost whole seconds in silence. That is how a seed stops being the seed."""
    eng, bus = an_engine(), RecordingBus(rounds=2)
    monkeypatch.setattr(node.time, "monotonic", ScriptedClock(step=2.0))   # 2 s per round
    with caplog.at_level(logging.WARNING, logger="mecanum.node"):
        node.run_loop(eng, bus, speed=1.0)
    warnings = [r.getMessage() for r in caplog.records if "wall clock" in r.getMessage()]
    assert len(warnings) == 1, warnings
    assert "--speed" in warnings[0] and "--fixed-step" in warnings[0]
    lost = re.search(r"by (\d+\.\d) s", warnings[0])
    assert lost and 1.5 <= float(lost.group(1)) <= 4.0, warnings[0]


# ------------------------------------------------------------------- what the engine itself drops


def test_the_engine_counts_the_seconds_its_accumulator_threw_away(caplog):
    eng = an_engine()
    with caplog.at_level(logging.WARNING, logger="mecanum.engine"):
        eng.step(2.0)                                      # the cap keeps 0.5 s of catch-up
        assert eng.dropped == pytest.approx(1.5)
        eng.step(1.0)
        assert eng.dropped > 1.5                           # and it goes on counting
    warnings = [r.getMessage() for r in caplog.records if "wall clock" in r.getMessage()]
    assert len(warnings) == 1, "one warning per run, not one per step"
    assert "1.5 s" in warnings[0], warnings[0]


def test_a_normal_step_drops_nothing(caplog):
    eng = an_engine()
    with caplog.at_level(logging.WARNING, logger="mecanum.engine"):
        for _ in range(100):
            eng.step(0.02)
    assert eng.dropped == 0.0
    assert eng.t == pytest.approx(2.0, abs=1e-9)
    assert not [r for r in caplog.records if "wall clock" in r.getMessage()]


def test_the_loop_is_told_the_step_size_it_would_use():
    assert an_engine().sub_step == pytest.approx(1.0 / RATE)


# ------------------------------------------------------ fixed-step leaves the node's sleeps alone


def test_fixed_step_switches_the_sleeps_off_for_everybody(monkeypatch):
    """The student node paces itself on the same switch; otherwise it cannot keep up at all."""
    monkeypatch.delenv("MECANUM_FAST", raising=False)
    args = node.parser().parse_args(["--fixed-step"])
    assert node.pacing(args) == (1.0, True)
    assert os.environ["MECANUM_FAST"] == "1"
    monkeypatch.delenv("MECANUM_FAST", raising=False)      # do not leak the switch into other tests


def test_speed_is_a_number_and_the_default_is_real_time():
    assert node.pacing(node.parser().parse_args([])) == (1.0, False)
    assert node.pacing(node.parser().parse_args(["--speed", "4"])) == (4.0, False)
    assert node.pacing(node.parser().parse_args(["--speed", "0.5"])) == (0.5, False)
