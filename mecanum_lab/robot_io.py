"""Client for the student node: measurements in, wheel speeds and estimate out.

Topics (CONTRACT §3, CONTRACT-KF §2), the first ones relative to the robot name, the last absolute:
    cmd_vel        Twist         in   commanded body speed (vx, vy, omega)
    wheel_speeds   4 rad/s       out  [VL, VR, HL, HR] — the task in experiment 1
    odom gps scan  Odom Gps Scan in   dead reckoning, global pose, 360 LIDAR rays
    imu            Imu           in   6-DOF inertial measurement (experiment 2, 100 Hz)
    kf/pose        Kf            out  your own estimate incl. 1σ (experiment 2)
    mission_state  String        out  idle | running | done | failed:<reason>
    /sim/task /sim/robots /sim/world /sim/config   task, robot list, world, sensor profile
Runs over real ROS (ros_bridge) or the in-process bus (stub.py) — the same code in ROS
mode and in the `./lab run` quick start. `serve()` is the runner behind it.
"""
import json
import logging
import os
import sys
import time

from . import stub, types
from .types import Kf

log = logging.getLogger("mecanum.robotio")


def make_bus(node_name: str = "student"):
    """Real ROS when it is present and ready; otherwise the in-process bus (no ROS needed)."""
    try:
        from . import ros_bridge                     # written by agent B
        bus = ros_bridge.make_bus("auto", node_name=node_name)
        if bus is not None:
            return bus
    except Exception as exc:                         # no ROS installed or not sourced
        log.debug("ros_bridge unusable (%s) — using in-process bus.", exc)
    return stub.get_bus()


def robot_info(bus, name: str) -> dict:
    # This robot's entry from /sim/robots: mode, contacts, distance, mission, index
    payload, _ = bus.last("robots")
    try:
        return next((r for r in json.loads(payload or "[]") if r.get("name") == name), {})
    except (ValueError, TypeError):
        return {}


class RobotIO:
    """One robot from the outside: read sensors, write wheels and cmd_vel, report status."""

    def __init__(self, name: str, role: str = "controller", bus=None, cfg: dict | None = None,
                 stale_timeout: float = 2.0):
        self.name = types.sanitize_name(name)
        self.role = role
        self.bus = bus if bus is not None else make_bus(f"{role}_{self.name}")
        self.owns_bus = bus is None and not isinstance(self.bus, stub.StubBus)
        self.cfg = cfg                                  # sensing config, loaded lazily
        self.stale_timeout = float(stale_timeout)
        self._wheels, self._cmd = self.bus.pub("wheels", self.name), self.bus.pub("twist", self.name)
        self._state, self._reported = self.bus.pub("mission", self.name), None
        self._kf, self._kfinfo = self.bus.pub("kf", self.name), self.bus.pub("kfinfo", self.name)
        self.ik = None            # serve() stores module.inverse_kinematics here, see below

    def spin(self, timeout=0.01): self.bus.spin(timeout)
    def running(self) -> bool: return bool(self.bus.ok())
    def age(self, kind: str) -> float: return self.bus.last(kind, self.name)[1]   # 1e9: nothing yet
    def present(self) -> bool: return min(self.age("odom"), self.age("gps")) < self.stale_timeout

    def sleep(self, sec: float) -> None:
        """Sleep while spinning — without spin() the sensor data goes stale."""
        ende = time.monotonic() + max(sec, 0.0)
        while self.running() and time.monotonic() < ende:
            self.spin(min(0.02, ende - time.monotonic()))

    def _value(self, kind: str, fresh: bool = False):
        """Sampler: last value of a topic; `fresh` applies cmd_timeout as a watchdog."""
        value, age = self.bus.last(kind, self.name)
        if fresh:
            if self.cfg is None:
                self.cfg = types.load_config()
            return value if age <= float(self.cfg.get("cmd_timeout", 0.35)) else None
        return value

    def odom(self): return self._value("odom")
    def gps(self): return self._value("gps")
    def scan(self): return self._value("scan")
    def imu(self): return self._value("imu")
    def kf(self): return self._value("kf")
    def cmd_vel(self): return self._value("twist", fresh=True)
    def task(self): return str(self.bus.last("task")[0] or "")
    def mission_state(self): return str(self.bus.last("mission", self.name)[0] or "")
    def robots(self) -> list: return json.loads(self.bus.last("robots")[0] or "[]")

    def sensor_profile(self) -> dict:
        """Active sensor profile of the simulation (gps/odom/imu …) — from /sim/config.

        Do not guess which noise values apply: they are listed here. The grader checks
        those same values against the test profile of the task.
        """
        try:
            return json.loads(self.bus.last("config")[0] or "{}")
        except ValueError:
            return {}

    def truth(self):
        """Exact pose — **only** for self-checks and grading, never inside your filter!

        Only exists when the simulation runs with `debug_truth` (otherwise always None);
        the grader measures against it anyway. A filter that reads this is not a filter.
        """
        return self.bus.last("truth", self.name)[0]

    def world(self) -> dict:
        # {name, size, goal, spawns}: /sim/world, else worlds/<name>.txt directly (no sim tick)
        payload, _ = self.bus.last("world")
        try:
            if payload:
                return json.loads(payload)
        except ValueError:
            log.warning("world topic contains no JSON — reading the world locally.")
        try:
            from . import worlds
            cfg = self.cfg or types.load_config()
            w = worlds.load_world(str(cfg.get("world")), cfg=cfg)
        except Exception as exc:
            log.warning("world not readable (%s) — goal unknown.", exc)
            return {}
        return {"name": w.name, "size": list(w.size),
                "goal": None if not w.goal else [w.goal.x, w.goal.y, w.goal.theta],
                "spawns": [[p.x, p.y, p.theta] for p in w.spawns]}

    def send_wheels(self, w) -> None:
        # Four wheel speeds [VL, VR, HL, HR] in rad/s — the order is the contract
        try:
            radius = [float(v) for v in w]
        except (TypeError, ValueError):
            radius = []
        if len(radius) != 4 or not all(abs(v) < 1e6 for v in radius):
            raise ValueError(f"wheel_speeds needs exactly four finite values in rad/s: {w!r}")
        self._wheels(radius)

    def publish_cmd_vel(self, vx: float, vy: float = 0.0, omega: float = 0.0) -> None:
        """Set the body speed — the drive command for T2..T4.

        If `ik` is set (serve() does that), the command goes straight through your own
        inverse kinematics onto wheel_speeds. That is deliberate: mission() blocks, so
        the runner could not pass anything through while it runs — with `ik` your own
        kinematics is the only drive path in every task. To drive through send_wheels
        directly, just leave publish_cmd_vel out.
        """
        self._cmd(types.Twist(vx=vx, vy=vy, omega=omega))
        if self.ik is not None:
            self.send_wheels(self.ik(vx, vy, omega))

    def set_mission_state(self, state: str) -> None:
        if state != self._reported:                      # do not flood the topics
            self._reported = state
            self._state(str(state))

    def send_kf(self, x: float, y: float, theta: float = 0.0, sx: float = 0.0,
                sy: float = 0.0, sth: float = 0.0, info: dict | None = None) -> None:
        """Report your own state estimate (experiment 2) — including the uncertainty.

        `sx/sy/sth` are standard deviations (1σ) in m and radius, not variances. Without
        these numbers the task `kf_kovarianz` cannot be graded; a filter that claims 5 m
        and is 0.1 m off has estimated nothing. `info` goes to `/<robot>/kf/info` as JSON
        and shows up in the GUI — for Q, R, counters, whatever.
        """
        self._kf(Kf(t=0.0, x=float(x), y=float(y), theta=float(theta),
                    sx=abs(float(sx)), sy=abs(float(sy)), sth=abs(float(sth))))
        if info is not None:
            self._kfinfo(json.dumps(info, ensure_ascii=False))

    def config(self, path: str, standard=None):
        """Sensing value of the running simulation, e.g. `rob.config("imu.rate")`.

        Values from `/sim/config` first (those are the ones that count), then the local
        config tree — so a test profile set by the grader is not overlooked.
        """
        value = types.cfg_get(self.sensor_profile(), path, None)
        if value is not None:
            return value
        if self.cfg is None:
            self.cfg = types.load_config()
        return types.cfg_get(self.cfg, path, standard)

    def spawn(self, name: str, variant: str = "") -> dict:
        return self.bus.call(types.topic("spawn"), {"name": name, "variant": variant})

    def close(self) -> None:
        if self.owns_bus:                             # shared in-process bus stays
            self.bus.shutdown()


def _name_from_argv(argv: list) -> str:
    for i, arg in enumerate(argv):
        if arg in ("--robot", "--name") and i + 1 < len(argv):
            return argv[i + 1]
    return argv[0] if argv and not argv[0].startswith("-") else ""


def serve(module, name: str | None = None, hz: float = 50.0, argv: list | None = None,
          bus=None, startup: float = 60.0) -> None:
    """Runner behind the one line in the template: for task ""/"kinematik" it converts
    cmd_vel into four wheel speeds every tick, otherwise module.mission(rob, task) runs
    once. Ends when the robot disappears or the bus closes."""
    name = (name or _name_from_argv(sys.argv[1:] if argv is None else argv)
            or os.environ.get("MECANUM_ROBOT") or "student")
    rob, tick_time = RobotIO(name, bus=bus), 1.0 / min(max(float(hz), 1.0), 200.0)
    rob.ik = getattr(module, "inverse_kinematics", None)   # publish_cmd_vel -> own IK
    started, last_task, last_error, saw_robot = time.monotonic(), None, 0.0, False
    log.info("node '%s' starting for robot '%s' (%s).", getattr(module, "__name__", "?"),
             rob.name, "ROS" if rob.owns_bus else "in-process bus")
    while rob.running():
        saw_robot = saw_robot or rob.age("odom") < 1e8 or rob.age("gps") < 1e8
        if not saw_robot and time.monotonic() - started < startup:
            rob.spin(0.05)                               # still waiting for the spawn
            continue
        if not saw_robot or not rob.present():               # robot gone or no fresh data
            log.info("no robot '%s' (spawn it: ./lab spawn --name %s) or no data anymore.",
                     rob.name, rob.name)
            break
        task, start = rob.task(), time.monotonic()
        kinematik = task in ("", "kinematik")
        if task != last_task:
            last_task, _ = task, rob.set_mission_state("idle" if kinematik else "running")
            if not kinematik:
                try:
                    module.mission(rob, task)            # mission brings its own loop
                    rob.set_mission_state("done")
                except Exception as exc:                 # show errors, do not swallow them
                    log.exception("mission('%s') failed", task)
                    rob.set_mission_state(f"failed:{type(exc).__name__}: {exc}")
                continue
        command = rob.cmd_vel() if kinematik else None
        if command is not None:
            try:
                rob.send_wheels(module.inverse_kinematics(command.vx, command.vy, command.omega))
            except Exception as exc:
                if time.monotonic() - last_error > 2.0:
                    last_error = time.monotonic()
                    log.error("inverse_kinematics(%.2f, %.2f, %.2f) -> %s: %s",
                              command.vx, command.vy, command.omega, type(exc).__name__, exc)
        rob.spin(max(0.0, tick_time - (time.monotonic() - start)))
    rob.close()
