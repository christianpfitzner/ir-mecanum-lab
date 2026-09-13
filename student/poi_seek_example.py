#!/usr/bin/env python3
"""Example: find the radiation source of the empty hall by climbing the count.

    ./lab run --world open --config config/demo_poi_exploration.json --robot alice \\
              --controller student/poi_seek_example.py --headless --seconds 120

On screen while it drives: the `poi` segment of the readout line, and with the `p` layer the source
itself — the number this file prints at the end is the one the counter published.

**One reading tells you nothing.** The counter is a Poisson process over `poi.counts` events per
second, so its relative noise is 1/√intensity. Measured with the shipped settings, 4000 readings per
distance (`config/demo_poi_exploration.json`): 0.800 ± 0.045 at 0.5 m — 5.6 %; 0.200 ± 0.022 at 2 m —
11 %; 0.059 ± 0.012 at 4 m — 21 %. At the edge of a source's range one reading is worth about a fifth
of itself, so two single readings cannot say whether the robot came closer or the counter simply had a
good second. Everything here compares means over `WINDOW` seconds and never single values, and the
whole steering rule is the one the demo file's third exercise gives — "drive by whether the last
reading was bigger than the one before it" — where "last" means one window and "before it" the window
before that.

**Turning on the spot tells you nothing either**, and that is what makes this field different from a
maze: `intensity = activity / (1 + (d/d0)²)` (`mecanum_lab/pois.py`) depends on the distance alone, so
at one spot every heading reads the same. A gradient here is not a direction you can look for, it is
one you have to drive through. So when the mean stops rising this file does what a moth does — it keeps
driving and starts to curve, on an arc of one metre radius, because a metre of arc is the smallest
distance this field changes by more than the counter's own noise (+19 % per metre at 2 m, against 5 %
noise on a window mean). Straight while the count rises, an arc while it does not.

**The fourth rule is memory, and it is the one a counter forces on you.** Drive out of the range and
the reading is 0.000 — the same number as in a hall with no source at all, and the second version of
this file could not tell those two apart: it took silence for "nothing to steer by", drove straight
on, and finished 12 m away from a source it had measured at 0.16. So once the source has been heard
even once, silence means *I have left it*, and the way back is not another reading but the odometry:
the spot where the loudest window was measured is a place this robot knows, and it drives there again.
That is the whole difference between the counter and the LIDAR — one of them can only ever say "how
much here", so the map has to come from the wheels.

**The LIDAR does not help with the finding, and that is the point.** Gamma goes through the shelf.
Measured in `production`, standing 3.6 m from a source behind the corner of a table: the beam that
points at the source reports the table at 0.60 m, the counter answers 0.072 ± 0.014 and does not care
what is in between. The brake below is therefore not how the source is found — it is there so that a
robot staring at a counter stops in front of the shelf that only the LIDAR sees.
"""
import math
import sys

from mecanum_lab import robot_io
from mecanum_lab.types import wrap_angle

TICK = 0.02                  # s between two ticks of this node
WINDOW = 1.0                 # s of readings a mean is taken over: 5 of them at poi.rate
V_WALK = 0.35                # m/s: one second of driving adds a metre of series and of arc
SPIRAL = 0.35                # rad/s while looking around: at this speed, an arc of 1 m radius
TURN_P = 2.0                 # 1/s: heading P gain for the way back to the loudest spot
TURN_MAX = 1.2               # rad/s ceiling on that gain, for the brake and for the return alike
REACH = 0.5                  # m: from here the loudest spot counts as reached again
PATIENCE = 25.0              # s of arcing without a new best: the hill ends here
# A rise is 15 % over the previous window — over the *previous* window, not over the best mean seen so
# far, because a best that only ratchets upward cannot express "I am driving away from it". The first
# version of this file compared against the best: it drove out of the field at x = 9.6 m and never
# heard it again (highest reading of that whole 60 s run: 0.093).
# 15 % is above the noise of a five-reading mean at every distance (half the spreads quoted above:
# 2.5 %, 5 %, 9 %) and below the field's own step over one metre (+23 % at the range edge, +19 % at
# 2 m, +24 % at 1 m). Within about 0.3 m of the source the two stop being distinguishable, which is
# where the run ends — that is the answer, not a failure of the search.
REL = 0.15
# Under this the counter reports nothing. The source of `demo_poi_exploration.json` is 9.75 m from the
# spawn with a range of 4 m, so the first 17 s of the drive are silence, and silence is worth a
# different sentence depending on whether anything was ever heard: see the fourth rule above.
DEAD = 0.02
BRAKE = 0.55                 # m the LIDAR has to keep free in front of the chassis
SEARCH = 150.0               # s: give up when even a lap of the hall has found nothing


def mean_of(kept):
    """The mean of the readings inside the window — the only number this file trusts."""
    return sum(value for _t, value in kept) / len(kept) if kept else None


def collect(rob, kept, t) -> None:
    """Every new /poi reading into the window, and out of it again once `WINDOW` seconds passed.

    Keyed on the reading's own stamp and not on the tick: `/poi` arrives at `poi.rate` (5 Hz) while this
    node ticks at 50 Hz, so nine ticks out of ten have nothing to add — and the simulation time in the
    message is the one the window has to be measured in.
    """
    reading = rob.poi()
    if reading is not None and (not kept or reading.t > kept[-1][0]):
        kept.append((reading.t, reading.intensity))
    while kept and t - kept[0][0] > WINDOW:
        kept.pop(0)


def close_ahead(rob, sector_deg=40.0):
    """Metres to the first thing the LIDAR sees in front of the robot; None while it has no scan."""
    scan = rob.scan()
    if not scan or not scan.ranges:
        return None
    beams = max(1, int(sector_deg / math.degrees(scan.angle_increment)))
    return min(list(scan.ranges[:beams]) + list(scan.ranges[-beams:]))


def turn_to(o, target):
    """Yaw rate that points the robot at a spot its odometry remembers — a place, not a map."""
    err = wrap_angle(math.atan2(target[1] - o.y, target[0] - o.x) - o.theta)
    return max(-TURN_MAX, min(TURN_MAX, TURN_P * err))


def go(rob, ahead, turn) -> None:
    """One tick's command, with the brake in front of whatever the counter happens to be saying.

    A blocked front turns the robot instead of driving on, and does not stop it: standing still would
    end the run at a spot the counter never measured, and the hall is walkable in every direction but
    the one the shelf is in.
    """
    free = close_ahead(rob)
    if ahead > 0.0 and free is not None and free < BRAKE:
        ahead, turn = 0.0, TURN_MAX
    rob.publish_cmd_vel(ahead, 0.0, turn)


def report(rob, best, best_at, kept, looked) -> None:
    """Where the search ended, in the numbers the counter delivered and the wheels located."""
    o = rob.odom()
    print(f"poi seek: stopped at ({o.x:+.2f}, {o.y:+.2f}) m at t={kept[-1][0]:.1f} s after "
          f"{looked:.0f} s of arcing; loudest window mean {best:.3f} from {len(kept)} readings, "
          f"measured at ({best_at[0]:+.2f}, {best_at[1]:+.2f}) m — {math.dist((o.x, o.y), best_at):.2f} m back")
    print("  That spot is as far as a counter reaches: its mean at half the range carries 5 % noise, so "
          "the last metre is not resolvable from readings alone. For a distance instead of a place, "
          "drive straight through and take the two crossings of half the peak — they are 2·d0 apart, "
          "and d0 is the falloff of the field (`config/demo_poi_exploration.json` exercise 1).")


def drive(rob):
    """The whole program, one rule per branch, in the order the senses become useful.

    Silence at the start of a run is not information, so: drive until the counter says something.
    A rising window means the field is brighter than a second ago, so: drive on, that way. A window
    that stopped rising means this heading is done, so: keep the speed and curve until it rises.
    Silence after the source was heard means the robot has driven out of the field, so: back to the
    spot where it was loudest — which is a place in the odometry and the only map this file has. And
    when `PATIENCE` seconds of curving bring no new best, the field has been climbed as far as a
    Poisson counter can be climbed, which is what `report()` says.
    """
    kept, best, best_at, told, t = [], 0.0, None, -9.0, 0.0
    previous, sampled, looked = 0.0, 0.0, 0.0
    while rob.running() and t < SEARCH:
        collect(rob, kept, t)
        level = mean_of(kept) or 0.0
        t = kept[-1][0] if kept else t + TICK
        o = rob.odom()
        if t - sampled >= WINDOW:
            previous, sampled = level, t              # one comparison per window, not per tick
        if level > best:
            best, best_at = level, (o.x, o.y)         # the place where the counter said "loudest"
            if t - told > 2.0:                        # one line per step up, not one per tick
                print(f"t={t:5.1f}s window mean {level:.3f} at ({o.x:+.2f}, {o.y:+.2f}) m")
                told = t
        if level > previous * (1.0 + REL):
            looked = 0.0                              # this heading works: keep it
            go(rob, V_WALK, 0.0)
        elif best_at is None:
            go(rob, V_WALK, 0.0)                      # never heard anything: cover ground
        elif looked > PATIENCE:
            report(rob, best, best_at, kept, looked)
            return
        elif level < DEAD and math.dist((o.x, o.y), best_at) > REACH:
            go(rob, V_WALK, turn_to(o, best_at))      # out of the field: back to the loudest spot
        else:
            looked += TICK                            # arc on the spot's scale until it answers
            go(rob, V_WALK, SPIRAL)
        rob.spin(TICK)
    print(f"poi seek: {SEARCH:g} s and no source found — is `pois` set in this world's config?")


if __name__ == "__main__":
    robot_io.serve(sys.modules[__name__])
