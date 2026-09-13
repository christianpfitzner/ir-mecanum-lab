#!/usr/bin/env python3
"""Example: drive out of the range of the access point and see what that does to a command.

    ./lab run --world production --config config/demo_wifi.json --robot alice \\
              --controller student/link_autonomy_example.py --headless --seconds 60

Watch the bar above the robot (layer `n`), or `ros2 topic echo /alice/link`. It goes out on purpose:
the route is the hall's aisles, entered at the corner furthest from the AP, and the first thing that
stops it is the link and not a wall.

Two numbers make this a demonstration and not a drive. **q <= 0.35** is where this controller gives
up — a deliberate margin above the simulator's own failsafe (`wifi.link_up_q`, 0.15), because a robot
that waits until the link is *gone* to start thinking has already lost every command it sends after
that. And **1.5 s** is `wifi.link_timeout`: the time the low level has to last before the simulation
declares the link down, stops delivering external commands altogether and switches the robot to
`mode = autonomy`. Between those two numbers lies the whole exercise: the link is bad *before* it is
down, and only the topic says so.

Why the AP's position is in the message: nothing here knows the map. `link.ap` and the odometry are
enough to know which way is out of coverage and which way home — that is the difference between a
robot that reports its link and one that is rescued by it.

The LIDAR brake is not part of the exercise; it is there because the hall is a warehouse and a
controller that only listens to the radio drives into a shelf on its way out of range.
"""
import math
import sys
import time

from mecanum_lab import robot_io
from mecanum_lab.types import wrap_angle

TICK = 0.02                  # s between two controller ticks (50 Hz)
V_WALK = 0.60                # m/s — the robot's full default speed, this is a hallway
TURN_P = 2.2                 # 1/s: heading P gain, turn-in-place like a mecanum robot may
TURN_MAX = 1.6               # rad/s ceiling on that
Q_BRAKE = 0.35               # below this the link is not drivable: stop, report, do not guess
FREE_AHEAD = 0.55            # m: what the LIDAR has to see in front of the chassis
REACH = 0.35                 # m: from here a corner counts as driven
INSET = 1.0                  # m: the aisle circuit is driven this far inside the walls
CAUSE = {"faded": "the link went bad here (q at or below the brake level), I stopped where I noticed",
         "down": "the link is DOWN: nothing I send arrives any more, the failsafe has the robot",
         "obstacle": "a rack stopped me before the radio did — the hall ended first",
         "radio": "the /link topic stopped: this simulation is running without wifi.enabled",
         "odometry": "no odometry, so no idea where the AP is", "bus": "the bus ended mid-corner"}


def first_fix(rob, kind: str, wait: float = 30.0):
    """Wait for one sensor to say something once — no route can be planned from nothing.

    The deadline is the one wall clock in this file, and it is only there to turn "this simulation runs
    without `wifi.enabled`" into a message instead of a hang. Nothing about the drive depends on it:
    in a `--fixed-step` run the whole lab is faster than a wall clock, so counting ticks here would
    have given up before the first `/link` message was ever published.
    """
    deadline = time.monotonic() + wait
    while rob.running() and time.monotonic() < deadline:
        value = getattr(rob, kind)()
        if value is not None:
            return value
        rob.spin(TICK)
    return None


def circuit(size, start, ap) -> list:
    """The hall's aisle circuit, entered so that the first leg drives *away* from the AP.

    On the spawn's own line, then up the far aisle, then back along the far wall and home. Aisles and
    not a straight line because the shipped halls are warehouses: the direct way out of coverage runs
    through a rack, and a robot that hits a shelf learns nothing about its radio.
    """
    x0, x1 = INSET, float(size[0]) - INSET
    y0, y1 = INSET, float(size[1]) - INSET
    south, north = max(y0, start.y), y1
    if ap[0] <= start.x:                                   # AP west of the robot: go east first
        return [(x1, south), (x1, north), (x0, north), (x0, south)]
    return [(x0, south), (x0, north), (x1, north), (x1, south)]


def gap_ahead(scan, sector_deg: float = 14.0):
    """Free distance straight ahead: the shortest echo of a sector in front (None: no scan yet).

    A sector and not beam 0 alone, because the robot is rarely pointed exactly square at a shelf.
    Beams that found nothing are `inf` and stay `inf` here — a missing echo is open floor, never a
    wall at `range_max`.
    """
    if not scan or not scan.ranges:
        return None
    beams = max(1, int(sector_deg / math.degrees(scan.angle_increment)))
    return min(list(scan.ranges[:beams]) + list(scan.ranges[-beams:]))


def leg(rob, target) -> str:
    """Drive to one corner of the circuit and answer why it ended. `arrived` is the only success."""
    while rob.running():
        link = rob.link()
        if link is None:
            return "radio"
        if not link.up:
            return "down"
        if link.quality <= Q_BRAKE:
            rob.last_link = link
            return "faded"
        o = rob.odom()
        if o is None:
            return "odometry"
        free = gap_ahead(rob.scan())
        if free is not None and free < FREE_AHEAD:
            rob.last_link = link
            return "obstacle"
        dx, dy = target[0] - o.x, target[1] - o.y
        if math.hypot(dx, dy) < REACH:
            return "arrived"
        err = wrap_angle(math.atan2(dy, dx) - o.theta)
        rob.publish_cmd_vel(V_WALK if abs(err) < 0.5 else 0.0, 0.0,
                            max(-TURN_MAX, min(TURN_MAX, TURN_P * err)))
        rob.spin(TICK)                                      # spinning keeps the sensors fresh
    return "bus"


def drive(rob):
    """The whole program: find the AP, drive away from it until the link gives out, report.

    `serve()` calls this without a task: like the steering example, the node here is the driver.
    """
    start = first_fix(rob, "odom")
    link = first_fix(rob, "link")
    if start is None or link is None:
        raise RuntimeError(f"no {'odometry' if start is None else '/link'} — is the simulator "
                           "running, and with wifi.enabled?")
    rob.last_link = link
    points = circuit(rob.world().get("size") or [20.0, 12.0], start, tuple(link.ap))
    print(f"link example: AP at ({link.ap[0]:.2f}, {link.ap[1]:.2f}), q={link.quality:.2f} "
          f"({link.rssi_dbm:.1f} dBm) — driving the aisles until the link gives out")
    for target in points:
        cause = leg(rob, target)
        rob.publish_cmd_vel(0.0, 0.0, 0.0)                  # let go of the throttle in every case
        rob.spin(TICK)
        if cause == "arrived":
            continue
        seen = getattr(rob, "last_link", link)
        o = rob.odom()
        print(f"t={seen.t:6.1f}s at ({o.x:+.2f}, {o.y:+.2f}) m, {CAUSE[cause]}"
              f" — {seen.rssi_dbm:.1f} dBm, q {seen.quality:.2f}, {seen.dropped} frames lost, "
              f"{seen.latency_ms:.0f} ms on the wire")
        return
    print("link example: the whole circuit with the link intact — move the AP and try again")


if __name__ == "__main__":
    robot_io.serve(sys.modules[__name__])
