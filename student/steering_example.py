#!/usr/bin/env python3
"""Small example for the second drive train: one lap of a rounded rectangle, then parking.

Run it in the hall with the shelves, exactly like this:

    ./lab run --world production --robot car --variant steering \\
        --controller student/steering_example.py --headless --seconds 60 --truth

Three numbers tell you everything about this car. They are read from the running simulation
below, so they are the numbers of the car that is actually driving:

    wheel base  L = 1.00 m              distance between the two axles
    rack stop   32 deg                  the angle the steering cannot move past
    R_min = L / tan(32 deg) = 1.60 m    the smallest radius it can drive. Full stop.

That is why this example looks the way it does:

* Every corner is an arc of R_min + 0.15 m. A tighter corner is not a driving mistake here, it is a
  car that does not exist: `R = L / tan(delta)` has no workaround, and delta ends at 32 degrees.
* Corners are driven slower than straights. The rack moves at 60 deg/s, so it needs 0.5 s to get
  from straight ahead to 30 deg — at the 0.55 m/s of a corner that is a quarter of a metre of
  straight line where the corner should already have begun, and the odometry reports it afterwards.
  (A real car has a second reason: lateral friction is finite. This model is kinematic, so it does
  not slide — but it does steer late, and that is enough to miss the spot you aimed at.)
* `vy` is never used, because it can never help. A steering car cannot strafe; the simulation
  answers "steering robot cannot strafe, vy=0.25 dropped" once per robot and keeps driving. Every
  controller here is therefore (v, omega) — one speed, one angle, for two axles.
* The route starts on the spawn's own line, facing down it. A steering robot cannot turn while it
  stands, so "get onto the route" is already a maneuver: see the BOX comment.
* Everything is measured with the odometry, not with time. And this car's odometry drifts in yaw
  faster than a mecanum robot's: the mecanum integrator averages four wheel speeds, this one
  integrates a single steering angle that it only *believes* (there is no steering encoder on this
  car). Try `--set odom.steer_max_scale=1.1`: an end stop 10 % off turns every corner the same wrong
  way, and nothing ever cancels it out.
"""
import math
import sys
import time

from mecanum_lab import robot_io
from mecanum_lab.types import wrap_angle

TICK = 0.02                                       # s between two controller ticks (50 Hz)
V_STRAIGHT, V_CORNER, V_PARK = 0.75, 0.55, 0.30   # m/s — all of them below v_max = 0.8 m/s
CORNER_MARGIN = 0.15                              # m of radius we keep free above R_min
AIM_FLOOR = 0.35                                  # m: below it the aim circle becomes absurd
REACH = 0.30                                      # m: from here a waypoint counts as arrived
ARC_STEP_DEG = 15                                 # one waypoint every 15 deg of corner
STOP_GAP = 0.95                                   # m from the car centre to the shelf in front
APPROACH = 0.6                                    # 1/s: how the remaining gap sets the speed
# The lap as the tangent box of the rounded rectangle, in metres relative to the spawn — the free
# floor in the hall's lower left corner. Relative numbers on purpose: the odometry starts at the
# spawn, so these are the coordinates a driver in the real world would have to work with too. The
# box is placed so that its bottom straight lies on the spawn's own line facing east (a steering
# car cannot turn on the spot, so getting onto the route must not need a turn) and so that the last
# leg runs straight onto one shelf face.
BOX = (-0.80, 0.0, 3.50, 3.85)                    # (x0, y0, x1, y1) of the tangent box
AIM = (3.50, 5.40)                                # m: the shelf to park in front of, straight ahead


def car_numbers(rob):
    """L, delta_max and what follows from both: R_min. Asked of the simulation, not guessed."""
    wheel_base = float(rob.config("steering.wheel_base", 1.0))
    delta_max = math.radians(float(rob.config("steering.steer_max_deg", 32.0)))
    return wheel_base, delta_max, wheel_base / math.tan(delta_max)


def at_start(start, point) -> tuple:
    """A point planned from the start pose, into the coordinates the odometry reports.

    The odometry does not count from (0, 0) — it starts at the spawn pose, and the spawn has a
    heading. So the route below is planned in the car's own start frame (x forward, y to the left)
    and becomes a waypoint by turning it by that heading and adding the start position. Without this
    the example would only work in a world whose spawn points exactly east.
    """
    c, s = math.cos(start.theta), math.sin(start.theta)
    return (start.x + point[0] * c - point[1] * s, start.y + point[0] * s + point[1] * c)


def corner(cx, cy, radius, from_deg) -> list:
    """One corner as waypoints: its tangent point, then the arc in steps of ARC_STEP_DEG."""
    start = math.radians(from_deg)
    points = [(cx + radius * math.cos(start), cy + radius * math.sin(start))]
    for i in range(1, int(90.0 / ARC_STEP_DEG) + 1):
        alpha = math.radians(from_deg + i * ARC_STEP_DEG)
        points.append((cx + radius * math.cos(alpha), cy + radius * math.sin(alpha)))
    return points


def rounded_rectangle(box, radius) -> list:
    """One lap, counter-clockwise, as the four corners of the tangent box `box`.

    The corners are the arcs; the straights between them are what the tangent points of two
    consecutive corners imply. Every arc begins where its tangent touches the straight, so the path
    stays continuous and the rack never has to jump.
    """
    x0, y0, x1, y1 = box
    return [corner(x1 - radius, y0 + radius, radius, -90.0),         # bottom right
            corner(x1 - radius, y1 - radius, radius, 0.0),           # top right
            corner(x0 + radius, y1 - radius, radius, 90.0),          # top left
            corner(x0 + radius, y0 + radius, radius, 180.0)]         # bottom left


def steer(rob, target, v, k_max) -> float:
    """One tick of the only controller here: point the wheels at `target` and drive at `v`.

    The turn rate is the curvature of the circle that leads through that point — omega =
    v · 2 sin(alpha) / distance — which for waypoints on an arc of `radius` is v / radius again.
    Two things in here are specific to a steering car:

    * the distance to the point **is** the lookahead. With a fixed one, the arc waypoints (0.5 m
      apart) are under-commanded, every corner comes out a decimetre too wide, and the odometry
      only notices afterwards.
    * `k_max` clamps the command. Without the clamp the controller asks for whatever it wants and
      the rack sits on its end stop: the car drives R_min regardless of the planned corner, and
      nothing in the log says so.

    Returns the distance to the target — that is what the two maneuvers below count on.
    """
    o = rob.odom()
    if o is None:
        rob.publish_cmd_vel(0.0, 0.0, 0.0)
        return 1e9
    dx, dy = target[0] - o.x, target[1] - o.y
    dist = math.hypot(dx, dy)
    alpha = wrap_angle(math.atan2(dy, dx) - o.theta)
    curvature = min(k_max, max(-k_max, 2.0 * math.sin(alpha) / max(dist, AIM_FLOOR)))
    rob.publish_cmd_vel(v, 0.0, v * curvature)
    return dist


def follow(rob, points, v, k_max) -> bool:
    """Drive through a list of waypoints. False if the bus ended in the middle of them.

    A steering car can only roll in the direction its wheels point, so there are two ways to
    correct a drift: drive towards the point (what happens here) or strafe (impossible).
    """
    for target in points:
        while rob.running() and steer(rob, target, v, k_max) > REACH:
            rob.spin(TICK)                         # while spinning, the sensors stay fresh
        if not rob.running():
            return False
    return True


def gap_ahead(scan, sector_deg: float = 14.0):
    """Free distance straight ahead: the shortest echo of a sector in front of the car.

    A sector and not beam 0 alone, because the car is never pointed exactly perpendicular at a
    shelf, and one beam of 360 is one spot on the wall. Beams that found nothing are `inf` and stay
    `inf` here — a clipped range read as a number would look like a wall that is not there.
    """
    if not scan or not scan.ranges:
        return None
    beams = max(1, int(sector_deg / math.degrees(scan.angle_increment)))
    return min(list(scan.ranges[:beams]) + list(scan.ranges[-beams:]))


def park(rob, aim, k_max, gap=STOP_GAP):
    """Drive up to the shelf in front and stop with a measured gap.

    With the LIDAR and not with the odometry on purpose: the loading spot is a *distance to a
    shelf*, and wheel counters cannot tell you that. The aim point stays behind the shelf, so the
    wheels are straight long before the gap closes — a heading hold here would still carry the last
    10 degrees of the corner and drive the car east past the spot.

    The braking distance of this car is v² / (2 · max_accel) ≈ 2 cm at 0.30 m/s, so approaching
    slowly and then simply letting go of the throttle is not a gamble.
    """
    while rob.running():
        free = gap_ahead(rob.scan())
        if free is None:
            rob.spin(TICK)                            # the first scan has not arrived yet
            continue
        if free <= gap:
            break
        speed = min(V_PARK, APPROACH * (free - gap))   # P on the gap: arrives without a step
        steer(rob, aim, speed, k_max)
        rob.spin(TICK)
    rob.publish_cmd_vel(0.0, 0.0, 0.0)                 # and stay standing


def drive(rob):
    """The whole program: wait for the sensors, one lap, one corner, park.

    `serve()` calls this without a task: a steering car has nothing to pass through, so here the
    node is the driver.
    """
    deadline = time.monotonic() + 30.0
    while rob.running() and rob.odom() is None and time.monotonic() < deadline:
        rob.spin(TICK)                                # no odometry, no idea where the lap starts
    if not rob.odom():
        raise RuntimeError("no odometry — is the simulation running?")
    start = rob.odom()                                # the lap starts here, not at (0, 0)
    wheel_base, delta_max, r_min = car_numbers(rob)
    radius = r_min + CORNER_MARGIN                    # the car's own limit, plus a little air
    k_max = math.tan(delta_max) / wheel_base          # 1 / R_min: the sharpest curvature it has
    print(f"steering example: L={wheel_base:.2f} m, rack end stop "
          f"{math.degrees(delta_max):.0f} deg -> minimum radius {r_min:.2f} m, driven corner "
          f"{radius:.2f} m")
    lap = rounded_rectangle(BOX, radius)
    lap = [[at_start(start, p) for p in arc] for arc in lap]     # from the start frame to the world
    aim = at_start(start, AIM)
    for arc in lap:
        follow(rob, arc[:1], V_STRAIGHT, k_max)       # the straight up to the corner
        follow(rob, arc[1:], V_CORNER, k_max)         # the corner itself, and slower
    follow(rob, lap[0][:1], V_STRAIGHT, k_max)        # the bottom straight once more
    follow(rob, lap[0][1:], V_CORNER, k_max)          # and its corner: after it lies the spot
    park(rob, aim, k_max)                             # up to the loading spot, by LIDAR


if __name__ == "__main__":
    # serve() calls drive(rob) once — a task would run mission() instead, as in lab 1.
    robot_io.serve(sys.modules[__name__])
