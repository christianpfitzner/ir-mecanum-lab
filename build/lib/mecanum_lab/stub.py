"""Nachrichtenbus im eigenen Prozess — die ROS-schnittstelle ohne ROS.

Derselbe `Bus`-Vertrag wie `ros_bridge` (CONTRACT §6.7), nur ohne rclpy, ohne
Daemon, ohne ROS-Installation. Das bringt zwei Dinge:

* `./lab run` startet Simulator und Studierendenknoten in einem Prozess —
  Schnelleinstieg ohne ROS, ideal für den ersten Tag im Praktikumsraum.
* Unit-Tests brauchen keinen ROS-Master und laufen in Millisekunden.

Payloads sind die Dataclasses aus `types.py` (Twist, Scan, Odom, Gps, str, list).
Ein Bus gilt pro Prozess (`get_bus()`). Zustellung erfolgt synchron im Sender-
thread; empfangene Werte werden nur gespeichert, deshalb sind die Callbacks billig.
"""
import os
import threading
import time

from .types import topic

_LOCK = threading.Lock()
_BUS = None


class StubBus:
    """Themenname -> Callbacks, plus letzter Wert je Thema (latched) und Services."""

    def __init__(self, name: str = "stub"):
        self.name = name
        self._subs: dict[str, list] = {}
        self._last: dict[str, tuple] = {}
        self._srv: dict[str, object] = {}
        self._ok = True

    # ------------------------------------------------------------------ publisher-seitig

    def pub(self, kind: str, robot: str | None = None):
        """Gibt eine send-Funktion zurück, wie die ROS-Bridge auch."""
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
            except Exception:                        # ein kaputter Knoten bremst den Bus nicht
                import logging
                logging.getLogger("mecanum.stub").exception("Subscriber auf %s", name)

    # ------------------------------------------------------------------ subscriber-seitig

    def sub(self, kind: str, robot: str | None, cb) -> None:
        self.sub_topic(topic(kind, robot), cb)

    def sub_topic(self, name: str, cb) -> None:
        with _LOCK:
            self._subs.setdefault(name, []).append(cb)

    def last(self, kind: str, robot: str | None = None) -> tuple:
        """(letzter Payload oder None, Alter in Sekunden) — wie bei rclpy gequetscht."""
        got = self._last.get(topic(kind, robot))
        return (got[0], time.monotonic() - got[1]) if got else (None, 1e9)

    # ------------------------------------------------------------------------- services

    def service(self, name: str, handler) -> None:
        with _LOCK:
            self._srv[name] = handler

    def call(self, name: str, req: dict, timeout: float = 2.0) -> dict:
        handler = self._srv.get(name)
        if handler is None:
            return {"success": False, "message": f"Service {name} nicht vorhanden"}
        return handler(dict(req or {}))

    # -------------------------------------------------------------------------- lebenszyklus

    def spin(self, timeout: float = 0.01) -> None:
        # MECANUM_FAST=1 fuer beschleunigte Laeufe (tools/fastgrade.py): dann wird nicht
        # geschlafen, sondern so schnell gerechnet wie der Rechner kann.
        if os.environ.get("MECANUM_FAST") != "1":
            time.sleep(min(max(timeout, 0.0), 0.05))     # Callbacks laufen synchron im Sender

    def ok(self) -> bool:
        return self._ok

    def shutdown(self) -> None:
        self._ok = False

    def topics(self) -> list:
        return sorted(set(self._last) | set(self._subs))


def get_bus(create: bool = True) -> StubBus | None:
    """Der Bus dieses Prozesses; beim ersten Aufruf entsteht er."""
    global _BUS
    if _BUS is None and create:
        _BUS = StubBus()
    return _BUS


def set_bus(bus: StubBus | None) -> None:
    global _BUS
    _BUS = bus
