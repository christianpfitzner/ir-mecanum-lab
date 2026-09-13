"""Adding and removing a robot in a running simulation — the ways there are, and what each answers.

The spawn service had no test at all, and it shows. `docs/CONTRACT.md` promised `message="ok"` and an
error text `name already taken`, neither of which the code ever produced; and `/sim/spawn_robot` was the
only call a supervisor had on a machine where `mecanum_lab_interfaces` was never built — a service that
exists only when an optional package was compiled is not an interface, it is a maybe. So: the anonymous
`/sim/spawn_next` (`std_srvs/srv/Trigger`, no argument to format), and tests for what the handlers say.

CONTRACT §4.

The last block is the supervisor's hand: `teleport()` puts one robot down somewhere else — same heading,
same run, and the same "no floor, no" answer a radiation source gets — and one test runs the whole chain
from a right click in the window to the pose in the engine.
"""
import math

import pytest

from mecanum_lab import ros_bridge
from mecanum_lab.engine import SpawnError, SimEngine
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


# ----------------------------------------------------------- putting one down somewhere else


def free_floor(world, spot) -> bool:
    """Does the world itself accept this spot for a robot? (The rule, asked, not guessed.)"""
    try:
        world.free("test robot", spot[0], spot[1])
        return True
    except ValueError:
        return False


def test_a_teleported_robot_arrives_where_it_was_put_and_keeps_the_heading_it_had():
    """`teleport()` is the hand of the supervisor: place it there, do not turn it, do not reset it."""
    eng = hall()
    eng.spawn("alice")
    before_pose = eng.robots["alice"].pose
    moved = eng.teleport("alice", 3.0, 2.0)
    assert (round(moved.x, 2), round(moved.y, 2)) == (3.0, 2.0)
    assert moved.theta == pytest.approx(before_pose.theta), "placement moves, turning is a command"
    assert eng.robots["alice"].pose == moved, "the truth pose moves with it, not one step later"
    assert (eng.robots["alice"].chassis.pose.x, eng.robots["alice"].chassis.pose.y) == (3.0, 2.0)
    believed = eng.robots["alice"].odometer.pose
    assert (round(believed.x, 2), round(believed.y, 2)) == (3.0, 2.0), \
        "the odometry still believes the old spot: the exercise would start with a 20 m lie"


def test_a_teleport_leaves_the_rest_of_the_run_alone():
    """Time, task, the other robot, and the counters of the drive that happened stay as they were."""
    eng = hall()
    alice, bob = eng.spawn("alice"), eng.spawn("bob")
    eng.step(0.5)
    alice.contacts, alice.distance, alice.mission_state, alice.kf = 3, 4.5, "running", object()
    clock, bob_was = eng.t, bob.pose
    eng.teleport("alice", 3.0, 2.0)
    assert eng.t == clock and bob.pose == bob_was, "one robot was picked up, the run was not"
    assert (alice.contacts, alice.distance, alice.mission_state) == (3, 4.5, "running"), \
        "a robot that drove into three walls before it was moved has not un-driven them"
    assert alice.kf is None, "the estimate of the old drive says nothing about the new spot"


def test_a_spot_that_is_no_floor_is_refused_for_a_robot_as_it_is_for_a_source():
    """The same `World.free()` answers for both, and a refused placement has moved nothing."""
    eng = hall()
    eng.spawn("alice")
    stand = eng.robots["alice"].pose
    rack = eng.world.walls[0]
    for blocked in (((rack.x0 + rack.x1) / 2, (rack.y0 + rack.y1) / 2), (0.0, 1.0),
                    (eng.world.size[0] + 1.0, 1.0)):
        with pytest.raises(ValueError, match="robot 'alice'"):
            eng.teleport("alice", *blocked)
        assert eng.robots["alice"].pose == stand, "refused, and still refused: not moved at all"
    with pytest.raises(SpawnError, match="ghost"):
        eng.teleport("ghost", 3.0, 2.0)


def test_a_right_click_in_the_window_moves_the_robot_all_the_way_through_the_engine():
    """Window → run loop → engine, once through. Each of the three ends has its own tests already.

    The renderer only ever reports a spot (it is a view, and a test guards that it moves nothing), and
    the engine only ever knows a spot; what joins them is `node.run_loop`. A chain tested at its two ends
    is a chain nobody has run — the middle is where the name of one field and the order of two calls live.
    """
    import pygame

    from mecanum_lab import node, render, stub

    eng = hall()
    eng.spawn("alice")
    # A spot the world itself calls free, so this test does not hard-code the arena's racks.
    wide, high = eng.world.size
    candidates = [(wide * fx, high * fy) for fy in (.3, .5, .7) for fx in (.3, .5, .7)]
    spot = next(p for p in sorted(candidates, key=lambda p: math.dist(p, (wide / 2, high / 2)))
                if free_floor(eng.world, p))

    rend = render.Renderer(eng, eng.cfg)
    try:
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=3, pos=rend.px(*spot)))
        rend.show_trails = True
        node.run_loop(eng, stub.StubBus(), rend=rend, fixed_step=True, frame_max=3)
        assert rend.pick.open, "the right click never reached the window"
        row = rend.pick.row_rect(0)
        pygame.event.post(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1,
                                             pos=(row.x + 10, row.y + 5)))
        node.run_loop(eng, stub.StubBus(), rend=rend, fixed_step=True, frame_max=1)
    finally:
        rend.close()

    got = eng.robots["alice"].pose
    assert (round(got.x, 2), round(got.y, 2)) == (round(spot[0], 2), round(spot[1], 2)), \
        f"asked for {spot}, the robot stands at ({got.x:.2f}, {got.y:.2f})"
    # `forget()` drops the old history and the frame drawn right after it starts a new one *here*, so
    # what must never appear again is a point from the way to the old spot.
    remembered = rend.trails.get("alice", [])
    assert all(math.dist(p, spot) < 0.02 for p in remembered), \
        f"the trail still draws the way from the old spot: {remembered}"
