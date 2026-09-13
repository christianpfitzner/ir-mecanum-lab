"""Adding and removing a robot in a running simulation — the ways there are, and what each answers.

The spawn service had no test at all, and it shows. `docs/CONTRACT.md` promised `message="ok"` and an
error text `name already taken`, neither of which the code ever produced; and `/sim/spawn_robot` was the
only call a supervisor had on a machine where `mecanum_lab_interfaces` was never built — a service that
exists only when an optional package was compiled is not an interface, it is a maybe. So: the anonymous
`/sim/spawn_next` (`std_srvs/srv/Trigger`, no argument to format), and tests for what the handlers say.

CONTRACT §4.
"""
import pytest

from mecanum_lab import ros_bridge
from mecanum_lab.engine import SimEngine
from mecanum_lab.types import MSG_SPECS, load_config, topic
from mecanum_lab.worlds import load_world


def hall(**over):
    cfg = load_config(None, {"gui": False, **over})
    return SimEngine(load_world("arena", cfg=cfg), cfg, seed=1)


def test_asking_for_no_name_takes_the_first_free_one():
    """`robot1`, `robot2` — a supervisor that clicks a button does not know which names are taken."""
    eng = hall()
    out = ros_bridge.spawn_handler(eng)({})
    assert out["success"] and out["name"] == "robot1", out["message"]
    assert ros_bridge.spawn_handler(eng)({"variant": "steering"})["name"] == "robot2", \
        "an anonymous second call must not answer 'robot1 is taken'"
    assert eng.next_name() == "robot3"
    assert [r.spec.name for r in eng.robots.values()] == ["robot1", "robot2"]


def test_the_answer_is_a_sentence_about_a_robot_because_a_trigger_has_no_other_channel():
    """`spawned 'robot1' (stock, red) at (7.25, 4.75, 0 deg)` — and the same facts as fields."""
    out = ros_bridge.spawn_handler(hall())({"name": "carl"})
    assert out["success"] and out["name"] == "carl"
    assert out["message"].startswith("spawned 'carl' ("), out["message"]
    assert ["7.25", "4.75"] == [f"{out['x']:.2f}", f"{out['y']:.2f}"], "text and fields agree"
    assert {"index", "color", "marker", "variant", "theta"} <= set(out), "the typed service has them too"


def test_a_taken_name_and_a_full_hall_are_answers_rather_than_exceptions():
    """`ros2 service call` shows the message; a stack trace in the simulator shows nothing useful."""
    eng = hall()
    ros_bridge.spawn_handler(eng)({"name": "carl"})
    again = ros_bridge.spawn_handler(eng)({"name": "carl"})
    assert again["success"] is False and "already taken" in again["message"], again
    small = hall(spawn_limit=2)
    assert ros_bridge.spawn_handler(small)({"name": "one"})["success"], "one of two is allowed"
    assert ros_bridge.spawn_handler(small)({"name": "two"})["success"], "two of two is allowed"
    full = ros_bridge.spawn_handler(small)({"name": "three"})
    assert full["success"] is False and "Robot limit (2) reached" in full["message"], full
    assert sorted(small.robots) == ["one", "two"], "the refusal did not half-add a robot"
    bad = ros_bridge.spawn_handler(eng)({"name": "9live"})       # first character has to be a letter
    assert bad["success"] is False and "Invalid robot name" in bad["message"], bad
    assert "9live" not in eng.robots


def test_despawn_last_takes_the_robot_that_joined_last():
    """The pair of `/sim/spawn_next`: without it you can add three robots and never take one away."""
    eng = hall()
    drop = ros_bridge.despawn_handler(eng, last=True)
    assert drop({}) == {"success": False, "message": "no robot in this run"}
    for _ in range(3):
        ros_bridge.spawn_handler(eng)({})
    assert drop({})["message"] == "'robot3' removed"
    assert drop({})["message"] == "'robot2' removed"
    assert sorted(eng.robots) == ["robot1"]
    assert drop({"name": "robot1"})["success"] is True
    assert drop({"name": "nobody"}) == {"success": False, "message": "'nobody' not found"}


def test_the_named_call_still_names_and_the_anonymous_one_still_does_not():
    """Two handlers over one engine: `last=True` is the only difference, and it is only for empty input."""
    eng = hall()
    ros_bridge.spawn_handler(eng)({"name": "carl"})
    ros_bridge.spawn_handler(eng)({"name": "dora"})
    assert ros_bridge.despawn_handler(eng)({"name": "carl"})["success"] is True
    assert "carl" not in eng.robots and "dora" in eng.robots, "the named handler removed the named one"
    assert ros_bridge.despawn_handler(eng)(  # no `last`: an empty request is not "the last one"
        {}) == {"success": False, "message": "'' not found"}


def test_the_two_trigger_services_are_in_the_table_the_whole_lab_reads():
    """A topic that is not in `MSG_SPECS` cannot be named by `topic()`, so it does not exist."""
    assert topic("spawn_next") == "/sim/spawn_next" and topic("despawn_last") == "/sim/despawn_last"
    for kind in ("spawn_next", "despawn_last"):
        assert MSG_SPECS[kind][0] == "std_srvs/srv/Trigger", \
            f"{kind} has to be a Trigger: std_srvs has no service with a string in it (Kilted)"
        assert MSG_SPECS[kind][2] == "sim", "the robots are the simulator's, so is the request"
