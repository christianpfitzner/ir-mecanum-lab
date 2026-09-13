"""Points of Interest: a radiation source the robot can measure **through** a wall.

The scenario this exists for: the robot has to find a source it cannot see, by driving towards
whatever the counter says is louder. That makes it the second exploration sensor of the lab and the
deliberate opposite of the LIDAR — so the two are best taught side by side in one hall.

The field model is the shortest one that still has a gradient to follow, and nothing beyond it:

    intensity(d) = activity / (1 + (d / d0)²)   for d <= range
                 = 0                            beyond the source's range

`d0` is the distance at which the counter reads half the source's `activity`, so it sets how fast
the field falls off; `range` is where the counter stops seeing the source at all. Half at `d0`, a
tenth at `3·d0`, a hundredth at `9·d0` — following the ratio of two readings is therefore a
distance estimate, which is exactly what the students have to build (types.Poi explains why the true
`distance` is not in the message by default).

**No line of sight, on purpose.** Nothing in `Source.intensity()` asks what stands between the
robot and the source: no ray, no shadow, no reflection. A gamma source does not care about the shelf
in front of it — the attenuating metre of steel and plastic costs a real detector a few per cent of
its counts and its teaching value nothing. That is the reason this sensor behaves the way the LIDAR
does not: in `production` the robot drives past a source it measures at 0.6 the whole time, because
a table is between its eyes and the source and nothing is between its antenna and the source. The
walls of the arena are only where the sources may not be (load_sources), never what a reading is
measured against.

Stdlib only, no numpy, and the noise comes from the simulation's one `sensors.Noise` stream.
"""
import math
from dataclasses import dataclass

from .sensors import Noise
from .types import Poi

KINDS = ("radiation",)              # the only field model here; a new kind needs an intensity()


@dataclass
class Source:
    """One Point of Interest: what it is called, where it is, how loud it reads, how far it reaches.

    `range_m` is the JSON key `range` — the longer name keeps it apart from `range_max` of the
    LIDAR (the sensor's limit) and from the built-in `range`, both of which mean something else.
    """
    name: str
    kind: str = "radiation"
    x: float = 0.0
    y: float = 0.0
    activity: float = 1.0
    range_m: float = 4.0
    d0: float = 1.0

    def distance(self, x: float, y: float) -> float:
        """Metres from the source to a position — the quantity the exercise has to reconstruct."""
        return math.dist((self.x, self.y), (x, y))

    def intensity(self, x: float, y: float) -> float:
        """The field at (x, y), noise-free. Through every wall, see the module docstring."""
        d = self.distance(x, y)
        if d > self.range_m:
            return 0.0
        return self.activity / (1.0 + (d / self.d0) ** 2)


def load_sources(value, world, d0_default: float = 1.0) -> list:
    """The `pois` config key -> [Source], refusing the three mistakes that ruin a scenario.

    Checked, with the name of the source in every message, because a mis-placed source is not a
    scenario that goes wrong quietly but one that cannot be solved at all:

    * **inside the walls** — the point has to be in the hall and outside every wall rectangle. A
      source in a wall is measurable from three sides and unreachable from none, which reads as a
      broken sensor rather than as a typo.
    * **range > 0** (and `d0 > 0`, `activity >= 0`) — a source that reaches nowhere is never heard.
    * **unique names** — the message carries the name and nothing else; two sources sharing one
      make the two readings impossible to tell apart.

    A source may be in a wall-free corner or in the middle of a room; `kind` has to be one of
    `KINDS`, because a field model that is not implemented here is not simulated at all. A source
    that does not name its own `d0` gets `d0_default`, which is `poi.d0` of the configuration — the
    fall-off of the hall, so to speak, that one demo file sets once instead of per source.
    """
    out = []
    for entry in value or []:
        if not isinstance(entry, dict):
            raise ValueError(f"poi entry {entry!r} is not an object with name/x/y/activity/range")
        name = str(entry.get("name") or "").strip()
        if not name:
            raise ValueError(f"poi entry {entry!r} has no name")
        kind = str(entry.get("kind", "radiation"))
        if kind not in KINDS:
            raise ValueError(f"poi '{name}': unknown kind '{kind}', this sensor simulates "
                             f"{', '.join(KINDS)}")
        try:
            x, y = float(entry["x"]), float(entry["y"])
            activity = float(entry.get("activity", 1.0))
            reach = float(entry.get("range", 4.0))
            d0 = float(entry.get("d0", d0_default))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f"poi '{name}': x, y, activity, range and d0 have to be numbers "
                             f"({exc})") from exc
        if reach <= 0.0 or d0 <= 0.0:
            raise ValueError(f"poi '{name}': range and d0 have to be > 0 m "
                             f"(got range {reach:g} m, d0 {d0:g} m)")
        if activity < 0.0:
            raise ValueError(f"poi '{name}': activity has to be >= 0 (got {activity:g})")
        _inside(name, x, y, world)
        if any(s.name == name for s in out):
            raise ValueError(f"poi name '{name}' is used twice — the counter reports one number, so "
                         "the window and the log could not say which of the two made it")
        out.append(Source(name=name, kind=kind, x=x, y=y, activity=activity,
                          range_m=reach, d0=d0))
    return out


def _inside(name: str, x: float, y: float, world) -> None:
    """Reject a source that is not in the open area of `world`, with a readable reason."""
    wide, high = world.size
    if not 0.0 < x < wide or not 0.0 < y < high:
        raise ValueError(f"poi '{name}' at ({x:g}, {y:g}) is outside the walls of world "
                         f"'{world.name}' ({wide:g} x {high:g} m)")
    for wall in world.walls or []:
        if wall.x0 <= x <= wall.x1 and wall.y0 <= y <= wall.y1:
            raise ValueError(f"poi '{name}' at ({x:g}, {y:g}) sits inside a wall of world "
                             f"'{world.name}' — a source needs open floor around it")


def loudest(sources, pose):
    """The source that dominates the counter at `pose` — the tutor's answer, not the robot's.

    One function for the three places that want it (`PoiSensor.read`, the readout line, the log)
    instead of three copies of `max(...)`: which source is loudest is field arithmetic, and field
    arithmetic in two places is two answers. None for a world without sources or for a robot that has
    no pose yet.
    """
    if not sources or pose is None:
        return None
    return max(sources, key=lambda s: s.intensity(pose.x, pose.y))


class PoiSensor:
    """A counter, not a ruler: it reports counts per second and never a distance.

    The reading is the loudest source's field with counting noise on top, which is what a wide-band
    counter measures and why the sensor still knows which source dominates: that answer is for the
    window and the log, never for the message (`types.Poi` is a stamp and an intensity).

    The noise is Poisson, which is what a counter has: `counts = intensity · poi.counts` per reading
    and `sigma = sqrt(counts)`, so the *relative* error is `1/sqrt(counts)` and grows as the field
    falls off. Measured with the shipped source and 400 counts per unit (poi.counts, the default):
    5.6 % at 0.5 m, 11 % at 2 m, 21 % at 4 m — a reading near the source is nearly exact and one at
    the edge of its range is a hint, which is the behaviour a gradient-following controller has to
    survive. `poi.counts: 0` switches the counter model off and reports the field itself.
    """

    def __init__(self, sources, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.sources = list(sources or [])
        self.noise = noise
        self.gain = float(cfg.get("counts", 400.0) or 0.0)

    def read(self, pose) -> Poi | None:
        """One measurement at `pose`; None when this world has no source at all."""
        if not self.sources:
            return None
        field = loudest(self.sources, pose).intensity(pose.x, pose.y)
        if self.gain <= 0.0:
            reading = field                           # no counter modelled: the field, exactly
        else:
            counts = field * self.gain
            # `Noise.gauss(0.0)` draws nothing, so a source out of range stays exactly 0.0 — and a
            # drive that never comes near one leaves the random stream untouched.
            reading = max(counts + self.noise.gauss(math.sqrt(counts)), 0.0) / self.gain
        return Poi(t=0.0, intensity=reading)
