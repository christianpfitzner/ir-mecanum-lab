#!/usr/bin/env python3
"""Lab 1 — driving Mecanum wheels. This is the file you hand in.

Convention (handout §2, stated nowhere else this exactly):
    x  points forward                [m/s]
    y  points to the LEFT            [m/s]   <- left is positive, not right!
    theta is counter-clockwise       [rad/s] <- turning left is positive
    Wheels in the order [VL, VR, HL, HR], given in rad/s (not rpm!)

    Top view, roller axes in X layout:              inverse kinematics = your task:
        VL ╱····╲ VR                                  a = lx + ly
        (one drive motor per wheel, roller diagonal)  VL = (vx - vy - a·ω) / r
        HL ╲····╱ RR                                  ...three more to follow

Start (one command, no ROS needed):
    ./lab run --robot alice --controller student/controller_template.py
Check it yourself:
    ./lab grade --robot alice --task kinematik     # checks your signs only
Drive with the keyboard (only if you do nothing at all, simulator remote control):
    ./lab sim --world maze            # then in a second terminal ./lab teleop --robot alice

You change only two things: `inverse_kinematics` (subtask 1) and the three
`fahre_*` functions (subtasks 2-4). The rest is plumbing.
"""
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import cfg_get, load_config

# Drive geometry from config/default.json — on the real robot these are the
# measured values; here they are given. Do not change them, or your calculation
# will not agree with the simulation.
CFG = load_config()
LX = cfg_get(CFG, "robot.lx")            # half the wheelbase in x [m]
LY = cfg_get(CFG, "robot.ly")            # half the width in y     [m]
R = cfg_get(CFG, "robot.r")              # wheel radius            [m]
A = LX + LY                              # lever arm for the yaw motion


# ------------------------------------------------------------------- Subtask 1 (T1)

def inverse_kinematics(vx, vy, omega):
    """Body velocity -> four wheel speeds [VL, VR, HL, HR] in rad/s.

    Derivation in the handout: the rollers allow motion along their own axis, the
    driven wheel supplies the component across it. Add the body velocity (vx, vy)
    and the yaw term ω × lever arm for one wheel and exactly the roller direction
    remains — that is why one half cancels in every term.

    Mnemonic instead of copying:  VL and RR run the same way, VR and HL are their
    mirrors at the x-axis; to turn left (ω > 0) the right wheels have to run faster
    than the left ones.
    """
    vl = 0.0      # TODO 1: term for VL, front left
    vr = 0.0      # TODO 2: term for VR, front right  (VL mirrored at y)
    hl = 0.0      # TODO 3: term for HL, rear left  (VR mirrored at x)
    hr = 0.0      # TODO 4: term for HR, rear right
    return [vl, vr, hl, hr]


# ------------------------------------------------------------------- Subtasks 2 to 4

def fahre_quadrat(rob):
    """T2: 1 m per side, 90° corners, odometry as feedback, closed loop.

    Success (./lab grade --task quadrat): after four sides be back within 0.20 m
    and 15° of the start pose, total time below 90 s, no wall contact. Idea: drive
    straight at the commanded vx and correct the lateral offset from odometry; at a
    corner turn until the odometry theta has advanced by 90°.
    """
    start = rob.odom()
    if start is None:
        raise RuntimeError(f"no odometry for '{rob.name}'")
    # TODO 5: for four sides call fahre_strecke(rob, 1.0) and drehe(rob, pi/2)
    raise RuntimeError("T2 not implemented yet")


def fahre_korridor(rob):
    """T3: out of the maze to the goal with LIDAR, without brushing a wall.

    Success (./lab grade --task korridor): goal reached (distance < 0.30 m),
    contacts == 0, and the side wall distance never fell below 0.25 m. Idea: the
    median of the front left and front right beams keeps you centered, the
    frontmost beam slows you down as soon as it gets tight.
    """
    # TODO 6: use rob.scan() — ranges[0] is front, angles counter-clockwise
    raise RuntimeError("T3 not implemented yet")


def fahre_zu_gps(rob):
    """T4 (bonus): drive to the goal with the global position instead of odometry.

    Success (./lab grade --task gps_anfahrt): goal from rob.world()["goal"] reached
    within 0.25 m; the GPS noise carpet must not shake you around (a dead zone and
    a damped turn rate are allowed).
    """
    goal = (rob.world() or {}).get("goal")
    if not goal:
        raise RuntimeError("no goal in rob.world() — a world without G?")
    # TODO 7: rotate the error vector into the body frame, then inverse_kinematics
    raise RuntimeError("T4 not implemented yet")


# ------------------------------------------------------------------- Plumbing (below)

def mission(rob, task):
    """Called by the runner exactly once for T2-T4, as long as rob.running().

    Finished = return normally (the runner then reports "done"). Given up = raise an
    exception: the runner reports "failed:<reason>", which is more honest for the
    grade than a "done" without reaching the goal.
    """
    for name, funktion in (("quadrat", fahre_quadrat), ("korridor", fahre_korridor),
                           ("gps_anfahrt", fahre_zu_gps)):
        if task.startswith(name):
            return funktion(rob)
    raise RuntimeError(f"unknown task '{task}'")


if __name__ == "__main__":
    # The runner takes over: connect the node, pass T1 straight through
    # (cmd_vel -> inverse_kinematics -> wheel_speeds) or call mission() for T2-T4.
    robot_io.serve(sys.modules[__name__])
