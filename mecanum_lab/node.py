"""Simulator command line — `./lab <command>` (CONTRACT §6.9, CONTRACT-KF §4).

A single simulation run core (`run_loop`) sits behind every variant: with and without a
window, with and without ROS, with a student node and with a grader. The only difference
is the bus: `run` uses the in-process bus (no ROS needed), `sim` takes real ROS whenever
rclpy is available.

    ./lab run  --robot alice --controller student/controller_template.py   # Quick start
    ./lab sim  --world track --robots alice,bob                            # Simulator (ROS)
    ./lab grade --robot alice --task alle                                  # Grade
    ./lab run  --task kf_gps --controller student/kf_template.py --truth    # Lab 2
    ./lab spawn --name bob ; ./lab robots ; ./lab docs

Settings without a JSON file: repeat `--set gps.sigma_xy=0.8 --set imu.rate=400` as often
as you like; `--log messung.csv` writes the measurement log.

How fast: `--speed 4` runs four simulation seconds per wall second, `--fixed-step` leaves the wall
clock out of the loop entirely. Without both, a grading run paces on the wall clock and so
measures whatever this machine happens to manage.
"""
import argparse
import importlib.util
import json
import logging
import math
import os
import sys
import threading
import time

from . import render as R
from . import ros_bridge, robot_io, stub, tasks as T
from .engine import SpawnError, SimEngine
from .logbook import Logbook
from .robot_io import RobotIO
from .types import MSG_SPECS, Twist, load_config, topic
from .worlds import list_worlds, load_world

log = logging.getLogger("mecanum.node")
WORLD_LIST = ", ".join(list_worlds())
MAX_LOOP_DT = 0.25        # s of sim time one loop round may make up; beyond that: time is lost


# --------------------------------------------------------------------- Run core


def run_loop(eng, bus, rend=None, graders=(), seconds=0.0, teleop=False, hz=60.0, tap=None,
             speed: float = 1.0, fixed_step: bool = False):
    """One tick: step the simulation, put measurements on the bus, draw, grade, log.

    The pace is `speed` **simulation seconds per wall second** (default 1 = real time, as in the
    lab course); `fixed_step` drops the wall clock out of the loop and steps exactly one physics
    step per round, as fast as the CPU allows. Either way the physics keeps its fixed step, so one
    seed is the same measurement series at every speed — only how often the wall clock is sampled
    changes.

    What the loop could not make up is counted (MAX_LOOP_DT per round) and named once, from 0.5 s
    of loss up: a run that quietly loses whole seconds is not grading the seed, it is grading the
    load on the host.
    """
    pubs, prev_t, t_clock, t_json, sim_t = {}, time.monotonic(), 0.0, 0.0, eng.t
    last_estimate = {}                      # so the log counts each kf/pose message only once
    last_robots, t_robots = None, 0.0
    pub_task = bus.pub("task")
    behind, warned = 0.0, False
    if fixed_step and rend is not None:
        log.info("--fixed-step with a window: the frame rate paces the simulation — use --headless")
    while bus.ok() and (rend is None or rend.ok) and (not seconds or eng.t < seconds):
        now = time.monotonic()
        if fixed_step:
            dt = eng.sub_step
        else:
            want = (now - prev_t) * speed            # sim seconds this wall slice is worth
            dt = min(want, MAX_LOOP_DT)
            if want > dt:
                behind += want - dt
                if not warned and behind >= 0.5:
                    warned = True
                    log.warning("sim time fell behind the wall clock by %.1f s — use --speed or "
                                "--fixed-step", behind)
        prev_t = now
        if rend is None or not rend.paused:
            eng.step(dt)
        elapsed, sim_t = eng.t - sim_t, eng.t   # what the simulator really advanced
        for kind, robot, payload in eng.drain():          # measurements -> topics
            if tap:
                tap.tap(kind, robot, payload)
            pubs.setdefault((kind, robot), bus.pub(kind, robot))(payload)
        # Robot list (mission_state, distance, contacts): the grader hangs on it, and a ROS
        # client that subscribes late only sees what is sent afterwards — the engine pushes
        # the list on spawn/reset only. So publish here at once on change (otherwise the
        # grader waits a second for the mission to end), but at least once per second:
        # `ros2 topic echo /sim/robots --once` would otherwise hang on nothing.
        robots = json.dumps(eng.robots_info())
        if robots != last_robots or now - t_robots > 1.0:
            pubs.setdefault(("robots", None), bus.pub("robots"))(robots)
            last_robots, t_robots = robots, now
        if tap:                                   # kf/pose comes from the students, not from the sim
            for name in eng.robots:
                schatzung, _ = bus.last("kf", name)
                if schatzung is not None and schatzung is not last_estimate.get(name):
                    last_estimate[name] = schatzung
                    tap.tap("kf", name, schatzung)
            tap.tick()
        if now - t_clock > 0.05:                        # /clock for use_sim_time
            bus.pub("clock")(eng.t)
            t_clock = now
        if now - t_json > 1.0:                          # world info, robot list, task, profile
            bus.pub("world")(eng.world_json())
            bus.pub("config")(eng.config_json())
            pub_task(eng.task)
            t_json = now
        if teleop and rend is not None:
            rend.teleop = True                             # HUD shows the driving keys
            tw = teleop_keys()
            if tw and tw != (0, 0, 0):
                for name in eng.robots:
                    pubs.setdefault(("twist", name), bus.pub("twist", name))(Twist(*tw))
        if rend is not None:
            if rend.poll()["quit"]:
                break
            rend.draw()
        for g in graders:
            g.tick(elapsed)                 # its clock is simulation time, not the wall clock
        if graders and all(getattr(g, "done", False) for g in graders):
            break                                       # grader is done -> end the run
        bus.spin(0.0 if fixed_step else (1.0 / hz if rend is None else 0.002))
    return eng


def pacing(args) -> tuple:
    """(speed, fixed_step) from the command line — `--fixed-step` switches the sleeps off.

    MECANUM_FAST is what the in-process bus and the student node read: without it the node would
    still pace its own loop on the wall clock and fall hopelessly behind a simulation that waits
    for nobody (tools/fastgrade.py has used this switch since it was written).
    """
    fixed = bool(getattr(args, "fixed_step", False))
    if fixed:
        os.environ["MECANUM_FAST"] = "1"
    return max(float(getattr(args, "speed", 1.0) or 1.0), 0.01), fixed


def wants_gui(args, cfg: dict) -> bool:
    """Window or not: `--headless` wins, otherwise the config decides — `"gui": false` is real.

    Until now nobody read `cfg["gui"]`, so a config file that switched the window off still got
    a window opened for it.
    """
    return not getattr(args, "headless", False) and bool(cfg.get("gui", True))


def teleop_keys() -> tuple:
    """Keys to body speed: up/down drive, left/right strafe (mecanum), q/, right, e/. left."""
    import pygame
    k = pygame.key.get_pressed()
    return (0.35 * (k[pygame.K_UP] - k[pygame.K_DOWN]), 0.35 * (k[pygame.K_LEFT] - k[pygame.K_RIGHT]),
            0.9 * (k[pygame.K_e] + k[pygame.K_PERIOD]
                   - k[pygame.K_q] - k[pygame.K_COMMA]))


def parse_set(text: str) -> tuple:
    """`--set gps.sigma_xy=0.8` -> ("gps", {"sigma_xy": 0.8}); the value is parsed as JSON.

    Numbers, booleans and lists work directly (`imu.rate=400`, `debug_truth=true`,
    `gps.bias_xy=[0.4,-0.2]`), everything else stays a string. Two dots in the path allow
    settings as deep as you like — the launch file can carry the whole wish list.
    """
    path, equals, value = str(text).partition("=")
    if not equals or not path.strip():
        raise ValueError(f"--set wants 'path.sub.path=value', got: '{text}'")
    try:
        value = json.loads(value.strip())
    except ValueError:
        pass
    parts = path.strip().split(".")
    tree = {parts[-1]: value}
    for part in reversed(parts[:-1]):
        tree = {part: tree}
    return parts[0], tree


def merge_sets(raw: list) -> dict:
    """Merge all `--set` values into one override tree; the one named last wins."""
    tree = {}
    for given in raw or []:
        key, zweig = parse_set(given)
        _nest(tree, key, zweig[key])
    return tree


def _nest(tree: dict, key: str, value) -> None:
    ziel = tree.setdefault(key, {})
    if not isinstance(value, dict):
        tree[key] = value
        return
    if not isinstance(ziel, dict):
        tree[key] = ziel = {}
    for teil, unter in value.items():
        _nest(ziel, teil, unter)


def make_engine(args):
    """Config layers: DEFAULT <- config/default.json <- --config <- test profile <- --set."""
    overrides = {"world": args.world, "gui": False if args.headless else None}
    if getattr(args, "truth", False):
        overrides["debug_truth"] = True
    if args.task:
        try:
            overrides.update(T.sim_profile(T.load_tasks(), args.task))
        except (ValueError, FileNotFoundError) as exc:
            log.warning("test profile not readable (%s) — grading runs with the "
                        "simulator's own settings", exc)
    overrides.update(merge_sets(getattr(args, "set", None)))
    cfg = load_config(getattr(args, "config", None), overrides)
    world_info = cfg_get_world(cfg, args, overrides)
    eng = SimEngine(load_world(world_info, cfg=cfg), cfg, seed=args.seed)
    # What was given via --set must never be outbid by a task profile: the engine carries
    # those values as a permanent override.
    eng.forced = {k: v for k, v in merge_sets(getattr(args, "set", None)).items()
                     if k != "world"}
    for name in [r for r in (args.robots or "").split(",") if r.strip()]:
        try:
            eng.spawn(name.strip(), getattr(args, "variant", ""))
        except (SpawnError, ValueError) as exc:
            log.error("robot '%s': %s", name, exc)
    if args.task:
        eng.set_task(first_task(args.task))
    return eng


def first_task(kette: str) -> str:
    """Reduce the --task value to its first real task: "alle" -> "kinematik".

    The task travels on /sim/task, and the students' runner switches when it arrives. A
    group is not a task, though: reporting "alle" as a task leaves the node with
    "unknown task" and an error message nobody caused.
    """
    try:
        picked = T.resolve(T.load_tasks(), kette)
    except (ValueError, FileNotFoundError):
        return kette
    return picked[0]["id"] if picked else kette


def cfg_get_world(cfg: dict, args, profile: dict) -> str:
    """Pick the world: what was named first, then the tasks' recommendation, then the default.

    Tasks name their arena through `"world"` in config/tasks.json — lab 1 wants
    `production`, lab 2 `arena`. That recommendation counts without `--world auto` too,
    otherwise you grade a square drive in the wrong arena and wonder about walls.
    `--world auto` is the same thing, only explicit; a named world always wins.
    """
    if args.world not in (None, "", "auto"):
        return args.world
    standard = cfg.get("world", "maze")
    if not args.task:                                     # no task announced -> no recommendation
        return standard
    try:
        picked = T.resolve(T.load_tasks(), args.task)
    except (ValueError, FileNotFoundError):
        return standard
    if not picked:
        return standard
    kf = all(a.get("kind") == "kf" for a in picked)      # blind drive with no world given: arena
    return T.world_for({"tasks": picked}, [a["id"] for a in picked],
                       default="arena" if kf else standard)


def bus_for(args, node_name, cfg: dict | None = None):
    """Stub when --stub is set or there is no ROS; otherwise the real ROS bus."""
    bus = ros_bridge.make_bus("stub" if args.stub else "auto", node_name=node_name, cfg=cfg)
    if bus is None:
        bus = stub.get_bus()
        log.info("No ROS 2 active — using the in-process bus (runs the same, only without ros2 topic).")
    return bus


def subscribe(bus, eng, name: str) -> None:
    """Wire a robot's command and estimate topics into the simulation."""
    bus.sub("twist", name, lambda t, n=name: eng.set_cmd_vel(n, t))
    bus.sub("wheels", name, lambda w, n=name: eng.set_wheel_speeds(n, w))
    bus.sub("mission", name, lambda s, n=name: eng.set_mission_state(n, s))
    bus.sub("kf", name, lambda k, n=name: eng.set_kf(n, k))


def task_profiles() -> dict:
    """{task id: task} — test profile and drive mode per task, not global."""
    try:
        return {a["id"]: a for a in T.load_tasks()["tasks"]}
    except Exception as exc:
        log.warning("tasks not readable (%s) — no automatic test profiles.", exc)
        return {}


def wire_task(bus, eng, profile: dict | None = None, robot: str | None = None) -> None:
    """Put /sim/task into the simulation: task, sensor profile, start pose.

    `robot` is the robot a grader is driving through a blind command run: for a KF task it
    is dropped at the spawn pose for that. The command sequence is never reported back, so
    it has to begin where the world parked the robot — otherwise the second task drives
    into a wall, because the first one ended somewhere.
    """
    profile = task_profiles() if profile is None else profile

    def on_task(text) -> None:
        name = str(text or "")
        task = profile.get(name) or {}
        if robot and task.get("kind") == "kf" and name != eng.task:
            eng.reset_robot(robot)
        eng.set_task(name)
        if task.get("sim"):
            eng.set_sensor_profile(task["sim"])
    bus.sub_topic(topic("task"), on_task)


def add_node(path, name, bus):
    """Student node as a thread: import the module, then let robot_io.serve() run."""
    spec = importlib.util.spec_from_file_location("student_node", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)

    def loop():
        try:
            robot_io.serve(mod, name=name, bus=bus)
        except Exception:
            log.exception("node %s aborted", path)

    t = threading.Thread(target=loop, daemon=True, name=f"node-{name}")
    t.start()
    log.info("node %s started for robot '%s'", os.path.basename(path), name)
    return t


# --------------------------------------------------------------------- Commands


def open_logbook(args, eng):
    """Open the CSV log when `--log` was given — otherwise None (nothing is written)."""
    if not getattr(args, "log", None):
        return None
    try:
        return Logbook(eng, args.log, getattr(args, "log_interval", 0.05) or 0.05)
    except OSError as exc:
        log.error("cannot create log '%s': %s", args.log, exc)
        return None


def cmd_run(args):
    """Simulator + student node in one process, without ROS — the quick start."""
    args.stub = True
    eng, bus = make_engine(args), stub.get_bus()
    tap = open_logbook(args, eng)
    if args.robot and args.robot not in eng.robots:
        # `--robot` is the name of your own robot — without spawning it explicitly the arena
        # would stay empty, and the student would see a blank map with a running node.
        try:
            eng.spawn(args.robot, getattr(args, "variant", ""))
        except (SpawnError, ValueError) as exc:
            log.error("robot '%s': %s", args.robot, exc)
    graders = [_grader(args.robot, args.task, bus, eng)] if args.grade else []
    wire_task(bus, eng, robot=args.robot if graders else None)
    node_threads = [add_node(c, args.robot, bus) for c in (args.controller or [])]
    rend = None if not wants_gui(args, eng.cfg) else R.Renderer(eng, eng.cfg)
    speed, fixed = pacing(args)
    run_loop(eng, bus, rend, graders, args.seconds, teleop=not args.no_teleop, tap=tap,
             speed=speed, fixed_step=fixed)
    if rend:
        rend.close()
    for k in node_threads:
        k.join(timeout=0.5)
    if tap:
        tap.close()
    return _print_reports(graders)


def _print_reports(graders, json_path: str | None = None) -> int:
    """Print the grading reports; 2 means at least one task was not met."""
    from .grade import format_report
    any_failed = False
    for g in graders:
        rep = g.report()
        print(format_report(rep))
        any_failed = any_failed or not rep.get("passed")
        if json_path:
            with open(json_path, "w", encoding="utf-8") as fh:
                json.dump(rep, fh, indent=1, ensure_ascii=False)
    return 2 if (graders and any_failed) else 0


def _grader(robot, task, bus, eng):
    from .grade import Grader
    world_info = json.loads(eng.world_json()) if eng else {}
    g = Grader(robot, task or "alle", bus, T.load_tasks(), world_info)
    return g.start()


def cmd_sim(args):
    """Simulator alone — with real ROS when available (the normal case in the lab room)."""
    eng = make_engine(args)
    bus = bus_for(args, "mecanum_sim", eng.cfg)
    if hasattr(bus, "enable_tf"):
        bus.enable_tf(eng, eng.cfg)               # /tf and /tf_static, so RViz can show the map
    tap = open_logbook(args, eng)
    for name in list(eng.robots):
        subscribe(bus, eng, name)

    def spawn(req):
        out = ros_bridge.spawn_handler(eng)(req)
        if out.get("success"):
            subscribe(bus, eng, str(req.get("name", "")))       #the new robot is audible too
        return out

    bus.service(topic("spawn"), spawn)
    bus.service(topic("despawn"), ros_bridge.despawn_handler(eng))
    wire_task(bus, eng, robot=args.robot if args.grade else None)
    bus.service(topic("reset"), lambda req: (eng.reset(), {"success": True,
                                                           "message": "world reset"})[1])
    graders = [_grader(args.robot, args.grade, bus, eng)] if args.grade else []
    rend = None if not wants_gui(args, eng.cfg) else R.Renderer(eng, eng.cfg)
    speed, fixed = pacing(args)
    if not args.stub:
        log.info("Topics: %s/<cmd_vel,wheel_speeds,odom,scan,gps,imu,kf/pose>  "
                 "/sim/<robots,world,task,config>  "
                 "/sim/<spawn_robot,despawn_robot,reset>  /clock  /tf /tf_static", "/<robot>")
    run_loop(eng, bus, rend, graders, args.seconds, teleop=not args.no_teleop, tap=tap,
             speed=speed, fixed_step=fixed)
    if rend:
        rend.close()
    bus.shutdown()
    if tap:
        tap.close()
    return _print_reports(graders)


def cmd_controller(args):
    """Only a student node (the simulator runs in another terminal)."""
    if not args.controller:
        log.error("--controller file.py missing")
        return 2
    add_node(args.controller[0] if isinstance(args.controller, list) else args.controller,
                 args.robot, bus_for(args, f"node_{args.robot}")).join()
    return 0


def cmd_client(args, command):
    """spawn/despawn/reset/robots/task: one short ROS request, nothing more."""
    bus = ros_bridge.make_bus("auto", node_name=f"client_{command}")
    if bus is None:
        log.error("No ROS 2 active. In a stub run these commands belong in the same process: "
                  "./lab run --robots %s ...", args.name or "alice")
        return 2
    if command == "robots":
        for _ in range(60):
            bus.spin(0.02)
            payload, age = bus.last("robots")
            if payload:
                for r in json.loads(payload):
                    print(f"{r['name']:14} {r['color']:8} {r['variant']:6} {r['mode']:12} "
                          f"pose=({r['pose'][0]:6.2f},{r['pose'][1]:6.2f}) "
                          f"dist={r['distance']:6.2f} m  contacts={r['contacts']}  {r['mission']}")
                return 0
        log.error("no answer from /sim/robots — is the simulator running?")
        return 2
    if command == "task":
        bus.pub("task")(args.task or args.name or "")
        print(f"task '{args.task or args.name}' sent.")
        bus.spin(0.1)
        return 0
    reply = bus.call(topic(command), {"name": args.name or "", "variant": args.variant or ""})
    print(("ok   — " if reply.get("success") else "error  — ") + str(reply.get("message", "")))
    if command == "spawn" and reply.get("success"):
        print(f"     {reply.get('color')} / {reply.get('marker')} / {reply.get('variant')}"
              f" at ({reply.get('x', 0):.2f}, {reply.get('y', 0):.2f})")
    bus.shutdown()
    return 0 if reply.get("success") else 1


def cmd_grade(args):
    """Grade. The tick comes from the simulator — here in a stub run, one process."""
    args.stub = True
    eng, bus = make_engine(args), stub.get_bus()
    tap = open_logbook(args, eng)
    if args.robot not in eng.robots:
        eng.spawn(args.robot)
    subscribe(bus, eng, args.robot)                      # kf/pose must get back into the sim
    wire_task(bus, eng, robot=args.robot)   # each task its own test profile, KF: spawn pose
    node_threads = [add_node(c, args.robot, bus)
                    for c in (args.controller or []) if not c.endswith(".json")]
    g = _grader(args.robot, args.task or "alle", bus, eng)
    rend = None if not wants_gui(args, eng.cfg) else R.Renderer(eng, eng.cfg)
    speed, fixed = pacing(args)
    run_loop(eng, bus, rend, [g], args.seconds, teleop=False, tap=tap,
             speed=speed, fixed_step=fixed)
    if rend:
        rend.close()
    if tap:
        tap.close()
    return _print_reports([g], args.json)


def cmd_docs(args):
    print(f"worlds: {WORLD_LIST}\n")
    print("Topics per robot:")
    for kind in ("twist", "wheels", "odom", "scan", "gps", "imu", "kf", "kfinfo", "mission"):
        print(f"  {topic(kind, 'alice'):24} {MSG_SPECS[kind][0]}")
    print("  /alice/truth (only with --truth or debug_truth, then at truth.rate)")
    print("  /sim/robots /sim/world /sim/task /sim/config  (std_msgs/String, JSON)")
    print("  /tf /tf_static  (tf2_msgs/TFMessage: map -> alice/odom -> alice/base_link "
          "-> laser, imu_link)")
    print("  /sim/spawn_robot /sim/despawn_robot (mecanum_lab_interfaces/srv/SpawnRobot "
          "oder JSON-Handshake)  /sim/reset (std_srvs/srv/Trigger)  /clock")
    print("\nTasks: " + T.short_help(T.load_tasks()))
    print("Groups: --task alle | kf_alle | v1 | v2 | a single task")
    print("\nExamples for lab 2 (Kalman filter):")
    print("  ./lab grade --task kf_alle --controller student/kf_solution.py --log messung.csv")
    print("  ./lab run --world arena --task kf_gps --robot alice \\")
    print("        --controller student/kf_template.py --truth --log messung.csv")
    print("  ros2 launch launch/kf.launch.py aufgabe:=kf_fusion headless:=true")
    return 0


# --------------------------------------------------------------------- Arguments


def parser():
    p = argparse.ArgumentParser(prog="lab", description="Small mecanum simulator (CONTRACT §6.9)")
    p.add_argument("--world", default=None,
                   help=f"World: {WORLD_LIST}, 'auto' = the tasks' recommendation "
                        "(KF tasks pick their arena automatically)")
    p.add_argument("--robots", default="", help="Comma-separated robot names to start")
    p.add_argument("--robot", default="alice", help="Your robot name (node, grader)")
    p.add_argument("--controller", action="append", help="Student node file, repeatable")
    p.add_argument("--task", default="",
                   help=f"Task or group: {', '.join(T.task_ids(T.load_tasks()))}, kf_alle, v1, v2")
    p.add_argument("--seconds", type=float, default=0.0, help="End after N s of simulation time")
    p.add_argument("--speed", type=float, default=1.0, metavar="N",
                   help="simulation seconds per wall second (default 1.0 = real time): the same "
                        "seed, the same physics steps, N times as fast")
    p.add_argument("--fixed-step", action="store_true",
                   help="step exactly 1/rate per round and never sleep: independent of the wall "
                        "clock, as fast as this CPU allows (combine with --headless)")
    p.add_argument("--headless", action="store_true", help="Without the Pygame window")
    p.add_argument("--stub", action="store_true", help="In-process bus instead of ROS")
    p.add_argument("--no-teleop", action="store_true",
                   help="turn keyboard control off (Up/Down drive, Left/Right strafe, "
                        "q/, right, e/. left)")
    p.add_argument("--seed", type=int, default=1, help="Noise seed (reproducibility)")
    p.add_argument("--config", default=None, help="JSON config on top of config/default.json")
    p.add_argument("--set", action="append", metavar="PATH=VALUE",
                   help="A single setting, e.g. --set gps.sigma_xy=0.8 (repeatable)")
    p.add_argument("--truth", action="store_true",
                   help="Publish the exact pose on /<robot>/truth (lab 2)")
    p.add_argument("--log", default=None, metavar="FILE.csv",
                   help="Write the measurement log (truth/gps/odom/kf/imu) as CSV")
    p.add_argument("--interval", "--log-intervall", dest="log_interval", type=float,
                   default=0.05, help="Spacing of log lines in s of sim time (default 0.05)")
    p.add_argument("--grade", nargs="?", const="alle", default=None,
                   help="Run the grader too: tasks or group for --robot (default: alle)")
    p.add_argument("--json", default=None, help="Write the grading report as JSON")
    p.add_argument("--name", default="", help="Robot name for spawn/despawn")
    p.add_argument("--variant", default="", help="Drive variant: stock|slow|fast|agile, "
                   "steering|steering-big (Ackermann car, see README)")
    return p


def cmd_teleop(args):
    """`./lab teleop --robot alice` — one robot, the keyboard is the remote (CONTRACT §6.9).

    Up/Down drive, Left/Right strafe, q or , turn right, e or . turn left. The handout's first
    exercise names this command, so it has to be one — and it is `sim` with one robot and the
    keyboard left on, nothing else.
    """
    if args.robot and not (args.robots or "").strip():
        args.robots = args.robot
    args.no_teleop = False
    return cmd_sim(args)


COMMANDS = {"run": cmd_run, "sim": cmd_sim, "teleop": cmd_teleop,
            "controller": cmd_controller, "grade": cmd_grade,
            "spawn": lambda a: cmd_client(a, "spawn"),
            "despawn": lambda a: cmd_client(a, "despawn"),
            "reset": lambda a: cmd_client(a, "reset"),
            "robots": lambda a: cmd_client(a, "robots"),
            "task": lambda a: cmd_client(a, "task"), "docs": cmd_docs}


# old German option names still work, but print which one to use now
DEPRECATED_OPTIONS = (("--log-intervall", "--interval"),)


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    for old, new in DEPRECATED_OPTIONS:
        if any(a == old or a.startswith(old + "=") for a in argv):
            print(f"deprecated option '{old}', use '{new}'")
    command = argv[0] if argv and not argv[0].startswith("-") else "sim"
    if command in ("-h", "--help", "help"):
        parser().print_help()
        print("\nCommands: " + ", ".join(sorted(COMMANDS)))
        return 0
    args = parser().parse_args(argv[1:] if argv and argv[0] in COMMANDS else argv)
    from . import setup_logging
    setup_logging()
    if command not in COMMANDS:
        log.error("unknown command '%s' — ./lab -h", command)
        return 2
    return COMMANDS[command](args)


if __name__ == "__main__":
    raise SystemExit(main())
