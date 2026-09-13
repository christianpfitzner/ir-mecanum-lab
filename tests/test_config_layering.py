"""The config layers: a `None` in an override means "nothing overridden", and `gui` is read.

Both bugs came from the same place — the CLI hands `load_config()` one override tree built from its
arguments, and an argument that was not given is `None`:

* `_merge` treated `None` as "delete this key", so `{"world": None}` erased the `world` from
  `config/default.json` and every run without `--world` silently landed in the built-in default.
  The same rule was why a task profile had to say `"gap": null` to switch an outage off; that
  spelling is now `[]` (an empty list), and `config/tasks.json` uses it.
* `cfg["gui"]` was in the config for appearance's sake: whether a window opened was decided by the
  `--headless` flag alone, so a config file switching the window off still got one.
"""
import argparse
import json

from mecanum_lab import node
from mecanum_lab import tasks as T
from mecanum_lab.engine import SimEngine
from mecanum_lab.types import _merge, cfg_get, load_config
from mecanum_lab.worlds import load_world


def write_config(tmp_path, name: str, body: dict) -> str:
    """A config file on disk — the layering bug is about files, not about dicts in memory."""
    path = str(tmp_path / name)
    with open(path, "w", encoding="utf-8") as fh:
        json.dump(body, fh)
    return path


def cli_args(**over) -> argparse.Namespace:
    """The argparse view of `./lab grade` with only what the test names on the command line."""
    args = argparse.Namespace(world=None, robots="", robot="muster", controller=None, task="",
                              seconds=0.0, headless=True, stub=True, no_teleop=True, seed=1,
                              config=None, set=None, truth=False, log=None, log_interval=0.05,
                              grade=None, json=None, name="", variant="", speed=1.0,
                              fixed_step=False)
    for key, value in over.items():
        setattr(args, key, value)
    return args


# --------------------------------------------------------------------------- None is not delete


def test_none_in_an_override_leaves_the_value_below_it_alone():
    dst = {"world": "maze", "gps": {"gap": [1.0, 2.0], "sigma_xy": 0.06}}
    _merge(dst, {"world": None, "gps": {"gap": None}})
    assert dst["world"] == "maze"
    assert dst["gps"]["gap"] == [1.0, 2.0]


def test_an_override_that_is_not_none_still_overwrites():
    dst = {"world": "maze", "gui": True, "gps": {"gap": None}}
    _merge(dst, {"world": "track", "gui": False, "gps": {"gap": []}})
    assert dst["world"] == "track" and dst["gui"] is False
    assert dst["gps"]["gap"] == []


def test_a_config_file_that_names_a_world_survives_the_missing_command_line_option(tmp_path):
    """The regression itself: a file sets the world, `--world` is not given -> that world is used."""
    cfg = load_config(write_config(tmp_path, "own_world.json", {"world": "track"}),
                      {"world": None, "gui": None})
    assert cfg["world"] == "track"


def test_the_engine_builds_the_world_of_its_config_file(tmp_path):
    """Same through the real entry point: `./lab grade --config own_world.json` (no --world)."""
    args = cli_args(config=write_config(tmp_path, "own_world.json", {"world": "track"}))
    assert node.make_engine(args).world.name == "track"


def test_a_world_named_on_the_command_line_still_wins(tmp_path):
    args = cli_args(world="arena", config=write_config(tmp_path, "own_world.json",
                                                       {"world": "track"}))
    assert node.make_engine(args).world.name == "arena"


def test_the_task_world_recommendation_still_works():
    """No world named: the tasks' recommendation counts; a named world still beats it."""
    cfg = load_config(None, {"world": None})
    assert node.cfg_get_world(cfg, cli_args(world=None), {}) == cfg["world"]
    assert node.cfg_get_world(cfg, cli_args(world="track"), {}) == "track"


# --------------------------------------------------------------------------------- the gui key


def test_headless_wins_and_otherwise_the_config_decides():
    assert node.wants_gui(cli_args(headless=True), {"gui": True}) is False
    assert node.wants_gui(cli_args(headless=False), {"gui": False}) is False
    assert node.wants_gui(cli_args(headless=False), {}) is True         # default: window on
    assert node.wants_gui(cli_args(headless=False), {"gui": True}) is True


def test_a_config_file_can_switch_the_window_off(tmp_path):
    """The whole chain: the file says gui:false, --headless is not given, no window is asked for."""
    args = cli_args(headless=False,
                    config=write_config(tmp_path, "no_window.json", {"gui": False}))
    eng = node.make_engine(args)
    assert cfg_get(eng.cfg, "gui") is False
    assert node.wants_gui(args, eng.cfg) is False


def test_headless_still_overrides_a_config_that_wants_a_window(tmp_path):
    args = cli_args(headless=True, config=write_config(tmp_path, "wants_window.json",
                                                       {"gui": True}))
    eng = node.make_engine(args)
    assert cfg_get(eng.cfg, "gui") is False           # the flag is the stronger voice
    assert node.wants_gui(args, eng.cfg) is False


# ------------------------------------------------------------- the "off" spelling in tasks.json


def test_a_task_profile_switches_an_outage_off_with_an_empty_list():
    """kf_fusion opens a GPS gap; the three tasks after it must not drive with it still open."""
    order = [t for t in T.load_tasks()["tasks"] if t.get("kind") == "kf"]
    assert [cfg_get(t["sim"], "gps.gap") for t in order] == [[], [15, 8], [], []]
    eng = SimEngine(load_world("arena"), load_config(None, {"gui": False}), seed=1)
    for task in order:
        if task.get("sim"):
            eng.set_sensor_profile(task["sim"])
        open_now = eng._gps.gap is not None
        assert open_now == (task["id"] == "kf_fusion"), \
            f"{task['id']} is graded with gap={eng._gps.gap}"
