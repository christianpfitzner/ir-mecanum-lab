"""In-process message bus — the ROS interface without ROS.

The same `Bus` contract as `ros_bridge` (CONTRACT §6.7), only without rclpy, without
a daemon, without a ROS installation. That brings two things:

* `./lab run` starts the simulator and the student node in one process — quick
  start without ROS, ideal for the first day in the lab.
* Unit tests need no ROS master and run in milliseconds.

Payloads are the dataclasses from `types.py` (Twist, Scan, Odom, Gps, str, list).
One bus per process (`get_bus()`). Delivery runs synchronously in the sender's
thread; received values are only stored, which keeps the callbacks cheap.
"""
import os
import threading
import time

from .types import topic

_LOCK = threading.Lock()
_BUS = None


class StubBus:
    """Topic name -> callbacks, plus the last value per topic (latched) and services."""

    def __init__(self, name: str = "stub"):
        self.name = name
        self._subs: dict[str, list] = {}
        self._last: dict[str, tuple] = {}
        self._srv: dict[str, object] = {}
        self._ok = True

    # ------------------------------------------------------------------ publisher side

    def pub(self, kind: str, robot: str | None = None):
        """Returns a send function, just like the ROS bridge does."""
        name = topic(kind, robot)

        def send(payload):
            self.publish(name, payload)
        return send

    def publish(self, name: str, payload) -> None:
        with _LOCK:
            self._last[name] = (payload, time.monotonic())
            subs = list(self._subs.get(name, ()))
        for cb in subs:
            try:
                cb(payload)
            except Exception:                        # a broken node must not block the bus
                import logging
                logging.getLogger("mecanum.stub").exception("Subscriber on %s", name)

    # ------------------------------------------------------------------ subscriber side

    def sub(self, kind: str, robot: str | None, cb) -> None:
        self.sub_topic(topic(kind, robot), cb)

    def sub_topic(self, name: str, cb) -> None:
        with _LOCK:
            self._subs.setdefault(name, []).append(cb)

    def last(self, kind: str, robot: str | None = None) -> tuple:
        """(last payload or None, age in seconds) — squeezed in, as rclpy reports it."""
        got = self._last.get(topic(kind, robot))
        return (got[0], time.monotonic() - got[1]) if got else (None, 1e9)

    # ------------------------------------------------------------------------- services

    def service(self, name: str, handler) -> None:
        with _LOCK:
            self._srv[name] = handler

    def call(self, name: str, req: dict, timeout: float = 2.0) -> dict:
        handler = self._srv.get(name)
        if handler is None:
            return {"success": False, "message": f"Service {name} not available"}
        return handler(dict(req or {}))

    # -------------------------------------------------------------------------- lifecycle

    def spin(self, timeout: float = 0.01) -> None:
        # MECANUM_FAST=1 for accelerated runs (tools/fastgrade.py): then it does not
        # sleep, it computes as fast as the machine allows.
        if os.environ.get("MECANUM_FAST") != "1":
            time.sleep(min(max(timeout, 0.0), 0.05))     # callbacks run synchronously in the sender

    def ok(self) -> bool:
        return self._ok

    def shutdown(self) -> None:
        self._ok = False

    def topics(self) -> list:
        return sorted(set(self._last) | set(self._subs))


def get_bus(create: bool = True) -> StubBus | None:
    """The bus of this process; the first call creates it."""
    global _BUS
    if _BUS is None and create:
        _BUS = StubBus()
    return _BUS


def set_bus(bus: StubBus | None) -> None:
    global _BUS
    _BUS = bus
