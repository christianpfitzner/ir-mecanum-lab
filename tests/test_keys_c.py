"""The keyboard: one table, no double binding, and the keys turn the way their names promise.

Three bugs lived here, all invisible from inside the file that caused them — which is why they are
checked against `mecanum_lab/keys.py`, the single table, rather than against expectations written
by the same hand as the code:

  * `q` turned right while it lies to the left of the driving hand (and quit at the same time);
  * `w`, `s`, `d` switched view layers, so the driving letters a WASD layout promises were taken;
  * the help line in the window's header named keys that no longer did anything.
"""
import os
import re
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from mecanum_lab import keys
from mecanum_lab.keys import DRIVING, LAYERS, check_bindings


def test_the_table_has_no_double_binding():
    """The rule of the whole file: no key means two things — see `keys.check_bindings()`."""
    assert check_bindings() == []


def test_the_rule_is_actually_enforced():
    """A guard that cannot fail guards nothing: hand it both kinds of collision, expect both back."""
    crossed = dict(keys.layer_table(), w=("show_wheels", ""))
    assert any("'w' drives and switches the layer" in line
               for line in check_bindings(crossed)), check_bindings(crossed)

    original, keys.LAYERS = keys.LAYERS, (("l", "show_trails", "odometry trail", ""),) + keys.LAYERS
    try:
        problems = check_bindings()
    finally:
        keys.LAYERS = original
    assert any("layer key 'l' appears twice" in line for line in problems), problems
    assert any("renderer attribute 'show_trails' appears twice" in line
               for line in problems), problems


def test_the_driving_letters_are_never_layer_switches():
    """`w` used to show the wheels, `s` the GPS shadow, `d` the painted floor — that blocked WASD."""
    for letter in "wasd":
        assert letter in DRIVING, f"{letter} should drive"
        assert letter not in keys.layer_table(), f"{letter} still switches a layer"
    moved = {"show_wheels": "r", "show_markers": "c", "show_zones": "x"}
    for attribute, key in moved.items():
        assert keys.key_of(attribute) == key, f"{attribute} lost its way to the keyboard"
    assert set(keys.LAYER_ATTRIBUTES) == {attr for _k, attr, _l, _e in LAYERS}, "a layer disappeared"


def test_turning_turns_the_way_the_keys_lie():
    """Left of the driving hand turns left, right of it turns right — all three pairs agree."""
    for key in ("a", "q", ","):
        assert keys.driving_twist({key})[2] > 0, f"{key} must turn left, omega > 0 (CONTRACT §5)"
    for key in ("d", "e", "."):
        assert keys.driving_twist({key})[2] < 0, f"{key} must turn right"
    assert keys.driving_twist({"q"}) == keys.driving_twist({"a"}), "the two turn pairs must agree"


def test_driving_the_obvious_way():
    assert keys.driving_twist({"w"})[0] > 0 and keys.driving_twist({"s"})[0] < 0
    assert keys.driving_twist({"left"})[1] > 0, "a mecanum base steers sideways: left is +y"
    assert keys.driving_twist({"right"})[1] < 0
    assert keys.driving_twist(set()) == (0.0, 0.0, 0.0)


def test_two_hands_do_not_make_twice_the_speed():
    """w+s cancel, a+d cancel, and q together with a is still one turn and not two."""
    assert keys.driving_twist({"w", "s"})[0] == pytest.approx(0.0)
    assert keys.driving_twist({"a", "d"})[2] == pytest.approx(0.0)
    assert keys.driving_twist({"a", "q"}) == keys.driving_twist({"a"})
    assert keys.driving_twist({"w"}) == keys.driving_twist({"w", "up"})
    assert keys.driving_twist({"w", "d"}) == (keys.TELEOP_SPEED, 0.0, -keys.TELEOP_YAW)


def test_the_speed_is_the_one_the_contract_names():
    """0.35 m/s and 0.9 rad/s, written down once instead of in three formulas."""
    assert keys.TELEOP_SPEED == pytest.approx(0.35) and keys.TELEOP_YAW == pytest.approx(0.9)


def test_a_key_this_keyboard_lacks_reads_as_not_pressed(monkeypatch):
    """`pressed_names` runs every loop round: a missing key must not raise in the middle of a run."""
    def no_such_key(name):
        raise ValueError(f"no key named {name} on this keyboard")
    monkeypatch.setattr(keys, "code", no_such_key)
    assert keys.pressed_names(object()) == set()


class FakeKeyboard(dict):
    """A `pygame.key.get_pressed()` look-alike: indexable by key code, everything else not pressed."""

    def __getitem__(self, code):
        return super().get(code, 0)


def test_pressed_names_reads_a_keyboard_state():
    import pygame
    state = FakeKeyboard({pygame.key.key_code("w"): 1, pygame.key.key_code("left"): 1})
    assert keys.pressed_names(state) == {"w", "left"}
    assert keys.driving_twist(keys.pressed_names(state)) == (keys.TELEOP_SPEED, keys.TELEOP_SPEED, 0)


def test_the_run_loop_asks_the_table_and_not_itself(monkeypatch):
    """`node.teleop_keys()` is the table's polling side — no second set of formulas in the loop."""
    from mecanum_lab import node

    class Pressed:
        def __getitem__(self, code):
            import pygame
            return int(code == pygame.key.key_code("w"))

    import pygame
    monkeypatch.setattr(pygame.key, "get_pressed", lambda: Pressed())
    assert node.teleop_keys() == (keys.TELEOP_SPEED, 0.0, 0.0)


def test_every_layer_is_reachable_by_key_and_by_panel():
    """A row of the panel must be a key and a key must be a row — those were two lists that drifted."""
    assert [attr for attr, _label, _key in keys.menu_rows()] == list(keys.LAYER_ATTRIBUTES)
    for attribute in keys.LAYER_ATTRIBUTES:
        assert keys.key_of(attribute), f"{attribute} has no key"
    assert keys.key_of("paused") == "", "a switch that is not a layer must not invent a key"


def test_window_and_panel_and_table_are_the_same_keys():
    """`render.KEYS` is the table plus pause; the panel shows the table. Neither adds a letter."""
    from mecanum_lab import menu, render
    assert set(render.KEYS) == {keys.PAUSE} | set(keys.layer_table())
    assert menu.LAYERS == keys.menu_rows()
    for key, (attribute, edge) in render.KEYS.items():
        assert attribute == "paused" or attribute in keys.LAYER_ATTRIBUTES, key
        if key != keys.PAUSE:
            assert keys.layer_table()[key] == (attribute, edge), key


def test_the_help_line_names_no_key_that_does_nothing():
    """The header line is the only documentation read with the window open, so it may not lie.

    Every short token of the line is asked whether the table binds it — that is the check that would
    have caught the line advertising a zoom key that was never handled and a grid that had already
    been deleted, both of which it did.
    """
    words = re.split(r"[·\s]+", " ".join((keys.help_line(True), keys.help_line(False))))
    named = {part for word in words for part in re.split(r"[/=,.]", word) if len(part) <= 3}
    allowed = (set(DRIVING) | set(keys.layer_table()) | set(keys.ROBOTS) | set(keys.ZOOM_IN)
               | set(keys.ZOOM_OUT) | {keys.PAUSE, keys.MENU, keys.FIT, keys.QUIT, keys.ALL_ROBOTS})
    text = {"drive", "drives", "turn", "strafe", "pause", "quit", "layers", "all", "follow", "zoom",
            "zooms", "pans", "drag", "wheel", "cursor", "world", "whole", "the", "or", "to", "of",
            "and", "9", "1.", "1..9", "ESC", "", "1..", "."}
    assert named - allowed - text == set(), f"the help line advertises keys nothing handles"


def test_the_help_line_names_every_layer():
    """A layer nobody can find is a layer that does not exist for the person in front of the window."""
    line = keys.help_line(True) + keys.help_line(False)
    for key, _attr, label, _edge in LAYERS:
        assert key in line, f"{label} ({key}) is not in the help line"


def test_the_readme_table_is_the_table_and_not_a_memory_of_it():
    """README's input table is the second place the keyboard is written down — so it is checked.

    The window's help line is generated from `keys.py`, but a table in the documentation is typed by
    hand, and a hand-typed copy is where the keyboard rotted last time: it went on advertising
    "`q` turn right" after `q` had been made to turn left, and "`l t g k w v d …`" after `w`, `s` and
    `d` had been handed back to driving. Both are sentences a reader would act on.

    So the two things that must agree with the code are taken from the code: the sequence of layer
    keys, and the direction the turn keys turn in. A row that renames a key or flips a direction fails
    here rather than in front of a class.
    """
    readme = (os.path.dirname(os.path.dirname(os.path.abspath(__file__))) + "/README.md")
    with open(readme, encoding="utf-8") as fh:
        text = fh.read()
    rows = [line for line in text.splitlines() if line.startswith("| `")]

    layer_keys = keys.layer_hint().split("layers: ")[1]
    assert f"| `{layer_keys}`" in text, f"README does not list the layer keys {layer_keys}"
    for key, _attr, label, _edge in LAYERS:
        assert key in layer_keys, f"{label} is on key {key}, which README cannot show"

    turning = [row for row in rows if "`q`" in row and "`e`" in row]
    assert len(turning) == 1, f"README documents the turn keys {len(turning)} times"
    assert "left/right" in turning[0][turning[0].index("`q`"):], (
        f"README promises q turns the other way again: {turning[0]}")

    driving = " ".join(rows)
    for pair in ("`w`/`s` drive", "`a`/`d` turn", "`Up`/`Down` drive", "`Left`/`Right` **strafe**"):
        assert pair in driving, f"README has stopped documenting {pair}"
