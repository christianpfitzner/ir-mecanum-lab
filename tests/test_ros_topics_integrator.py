"""Every topic the simulator can put on the bus must be translatable to ROS.

The story of this test: a package added the `sensorinfo` message to `types.MSG_SPECS` and the
conversion in `ros_bridge.to_ros()` — but not the `KIND_MSG` entry. Under ROS the first attempt to
publish it died with `KeyError: 'sensorinfo'` inside `run_loop`, so the simulator crashed three
seconds into a run and six steps of `tools/check.sh --ros` went red at once. The unit tests were
green, because stub mode never builds a real publisher. That gap is exactly what this file closes,
and it is checked from the data, so it cannot rot again.
"""
from mecanum_lab import ros_bridge as rb
from mecanum_lab.types import MSG_SPECS

# kinds whose ROS type is a service, not a topic: no publisher exists for them at all
SERVICE_KINDS = {k for k, spec in MSG_SPECS.items() if "srv" in spec[0] or spec[0] == "service"}


def test_every_topic_kind_has_a_ros_message_type():
    missing = sorted(k for k in MSG_SPECS if k not in SERVICE_KINDS and k not in rb.KIND_MSG)
    assert not missing, f"no ROS message type for {missing} — a ROS run raises on the first message"


def test_the_bridge_does_not_invent_kinds_of_its_own():
    invented = sorted(k for k in rb.KIND_MSG if k not in MSG_SPECS)
    assert not invented, f"KIND_MSG knows topics the type table does not: {invented}"


def test_the_message_names_are_names_the_bridge_can_ask_for():
    # the values are the attribute names of the imported message modules (M.<name>); an empty or
    # wrong one would fail at the same place KIND_MSG failed
    assert all(isinstance(name, str) and name for name in rb.KIND_MSG.values())


def test_services_are_services_and_nothing_else():
    # spawn/despawn are the own interface or the JSON handshake; spawn_next/despawn_last/reset are
    # Triggers, which is what makes them available on a machine that never built the interface (§4).
    assert SERVICE_KINDS == {"spawn", "despawn", "spawn_next", "despawn_last", "reset"}
    assert not (SERVICE_KINDS & set(rb.KIND_MSG))
