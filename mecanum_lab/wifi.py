"""The radio link: one access point per hall, distance and walls are what kill it.

Every command in this simulator arrives over a radio. The teleop keys, the `cmd_vel` of a student
node, the wheel speeds of the reference solution — all of them are frames sent to an access point,
and until this module existed every one of them was delivered instantly and perfectly. That
assumption is worth breaking on purpose: a real robot that loses its AP does not stop existing, it
stops *being told*, and what it does in that moment is the part of the course that a kinematics
exercise cannot teach.

The model is a **link budget**, not a WiFi simulation. Nothing here knows channels, association,
OFDM, DHCP or retransmission — those are fields of their own and this lab has two other experiments.
What it knows is dBm, and only the three things that decide a link in a hall:

    rssi = tx_dbm - 10 * n * log10(d / d0) - k * wall_db + shadow          [dBm]

* `n` (`wifi.n`, 2.4) is the path-loss exponent: 2.0 is free space, ~2.4-3.0 is an indoor hall with
  furniture, 3-4 is reinforced concrete. Doubling the distance costs `10 n log10 2` = 7.2 dB at
  n = 2.4 — and because decibels are logarithmic, that is the same 7.2 dB between 2 m and 4 m as
  between 16 m and 32 m.
* `k` is the number of **wall crossings** on the straight line AP → robot: the tables and racks of
  the arena, counted with the ray/rectangle entry test of the LIDAR (`sensors._ray_rect`), so the
  radio and the laser agree about which rectangles exist. Painted floor (`-` and `|` in the grid)
  are paint and cross for free; so does open floor.
* `shadow` is a slow seeded fade of ±`wifi.shadow_db` dB. A link that stands still does sit at
  -71.000 dBm forever in a model without it, and a robot parked exactly on the threshold would
  flicker between up and down on the screen. Real 2.4 GHz links wander by a few dB while nothing
  moves; here they wander slowly and on purpose (one new target per `wifi.shadow_period`, reached
  by a straight ramp), which is what keeps the threshold a decision instead of a coin toss.
* `tx_dbm` is the level the AP delivers **at the reference distance `d0`**, not its radiated power.
  A 2.4 GHz AP transmitting +17 dBm arrives at roughly -35 dBm one metre away at robot height; the
  free-space loss of that first metre is a constant that never changes, so it is folded into this
  one number instead of being carried around as a second constant with no knob on it.

The receiver turns the level into the single number everything else is written against:

    q = clamp((rssi - floor) / (good - floor), 0, 1)     floor = -85 dBm, good = -50 dBm

`good` is the level above which a robot's link is simply as good as it gets (the flat part of the
rate-vs-signal curve), `floor` is roughly the sensitivity limit of a module at the data rate a robot
uses — a real 802.11 module delivers -85 … -90 dBm before it stops decoding, and below `floor` this
one reports q = 0. `1 - q` therefore means "how far down from the flat part of the curve we are",
which is what the two consumers use it for:

* **delivery**: every external command is dropped with probability `p_drop = (1 - q)²` and otherwise
  arrives `wifi.latency_ms * (1 + 2 (1 - q))` ms later — real sim time, so a late command moves the
  robot later, and a student can measure it. Squared, because a frame is lost when either of two
  things goes wrong (the frame itself, or its acknowledge on the way back), and `q = 0.9` should be
  a link nobody notices while `q = 0.3` is a link that cannot be driven on.
* **autonomy**: below `wifi.link_up_q` (0.15) for longer than `wifi.link_timeout` (1.5 s) the link is
  *down*: nothing external is delivered at all and the robot switches to `mode = "autonomy"` with its
  onboard rule — `stop` (default: holds position, which here is what the command watchdog already
  does) or `dead_reckoning` (keeps executing the last command it really received, which is how a real
  robot walks itself home — or into the far wall).

**The limits of the model, said out loud.** It is one AP with one antenna pattern (omnidirectional,
equal in every direction), it does not know that two APs would roam, that a laptop association
shares airtime, that retransmits make a bad link *slow* before they make it lossy (only lossy), or
that walls reflect — so there is no multipath here, only attenuation, while `gps.zones` models
multipath for the GPS on purpose. It never drops a message out of order and never duplicates one,
both of which a real link does. And the fade is a scripted wander, not a Rayleigh draw. All of that
is left out because each piece would need a second model to be honest, and the point of this one is
the two numbers a student can check with a tape measure and the readout line: q at 2 m of free floor,
and q behind two racks. Those two are in the README table and in `config/demo_wifi.json`, and both
come out of this file with the shipped constants.

Stdlib only; the noise comes from the simulation's one `sensors.Noise` stream, so with
`wifi.enabled: false` (the default) no random number is drawn here at all and every graded command
stream stays what it was.
"""
import math

from .sensors import Noise, _ray_rect
from .types import Link

FLOOR_DBM = -85.0             # receiver sensitivity class of a robot module: below this, q = 0
GOOD_DBM = -50.0              # above this the link is as good as it ever gets (q = 1)
EPS = 1e-9                    # "the wall starts exactly where the robot is" is not a crossing


def quality_of(rssi: float, floor: float = FLOOR_DBM, good: float = GOOD_DBM) -> float:
    """dBm -> 0..1, linear between the two levels; clamped at both ends, as a real AGC reads."""
    if good <= floor:
        raise ValueError(f"wifi.good_dbm ({good:g}) has to be above wifi.floor_dbm ({floor:g})")
    return min(max((rssi - floor) / (good - floor), 0.0), 1.0)


def ap_point(value, world) -> tuple:
    """Check one access point position against the hall it is supposed to be mounted in.

    Two numbers, inside the walls, not inside a wall: the same three checks `pois.load_sources()`
    runs, for the same reason — an AP in a rectangle is a scenario that cannot work, and the run
    would explain nothing. A hall whose AP is not known at all is not an error, it is a hall with no
    radio, and `access_point()` answers None for that.
    """
    try:
        x, y = float(value[0]), float(value[1])
    except (TypeError, ValueError, IndexError, KeyError) as exc:
        raise ValueError(f"wifi.ap has to be [x, y] in metres of the world, got {value!r}") from exc
    wide, high = world.size
    if not 0.0 < x < wide or not 0.0 < y < high:
        raise ValueError(f"access point at ({x:g}, {y:g}) is outside the walls of world "
                         f"'{world.name}' ({wide:g} x {high:g} m)")
    for wall in world.walls or []:
        if wall.x0 <= x <= wall.x1 and wall.y0 <= y <= wall.y1:
            raise ValueError(f"access point at ({x:g}, {y:g}) sits inside a wall of world "
                             f"'{world.name}' — an AP needs open floor below it")
    return (x, y)


def access_point(cfg: dict, world) -> tuple | None:
    """The AP of this hall: `wifi.ap` wins, then `wifi.ap_by_world.<name>`, else no radio at all.

    The per-world entry is the installer's answer ("the AP of the hall hangs above the loading area,
    next to the door where the robots start"), `wifi.ap` is the one the exercise moves — on the
    command line (`--set wifi.ap=[18,10]`) or in a launch argument — without touching the arena.
    A world neither of them names gets None: `wifi.enabled` then has nothing to compute against, and
    the engine says so once instead of modelling a link to nowhere.
    """
    given = (cfg or {}).get("ap")
    if given is None:
        given = ((cfg or {}).get("ap_by_world") or {}).get(world.name)
    return None if given is None else ap_point(given, world)


class Shadow:
    """The slow fade: one sampled level per period, walked to in a straight line.

    Deliberately not a random walk. A walk drifts, and a link that has been parked in a corner for a
    minute would end up somewhere its own budget cannot explain; sampled-and-interpolated stays
    inside ±amp forever and is continuous, which is all the knife-edge problem needs. One draw from
    the shared stream per period per robot — with `amp` 0 not even that one.
    """

    def __init__(self, noise: Noise, amp_db: float = 3.0, period_s: float = 2.0):
        self.noise = noise
        self.amp = max(float(amp_db), 0.0)
        self.period = max(float(period_s), 1e-3)
        # `Noise.uniform(0.0)` answers 0.0 without asking the generator, so a fade with
        # `shadow_db: 0` costs a run nothing at all.
        self.from_db, self.to_db = 0.0, noise.uniform(self.amp)
        self.left = self.period

    def update(self, dt: float) -> float:
        """Advance by one physics step and answer the current level offset in dB."""
        if self.amp <= 0.0:
            return 0.0
        self.left -= dt
        if self.left <= 0.0:
            self.from_db, self.to_db = self.to_db, self.noise.uniform(self.amp)
            self.left += self.period
        gone = min(1.0, (self.period - self.left) / self.period)
        return self.from_db + (self.to_db - self.from_db) * gone


class LinkState:
    """One robot's side of the link: what the antenna sees, what it lost, what is still on the wire.

    The counters live here and not on `types.Robot` because they are the radio's own bookkeeping —
    the robot has `mode` (which the engine flips) and nothing else. `mode_before` is how the engine
    gets back to `pass-through`/`wheels` when the link returns: which of the two it was is a fact
    about the student's node, not about the antenna.
    """

    def __init__(self, name: str, noise: Noise, shadow_db: float, shadow_period: float):
        self.name = name
        self.fade = Shadow(noise, shadow_db, shadow_period)
        self.shadow_db = 0.0
        self.quality, self.rssi_dbm = 1.0, GOOD_DBM
        self.distance, self.walls = 0.0, 0
        self.low_for, self.down = 0.0, False
        self.dropped, self.delivered = 0, 0
        self.latency_s = 0.0
        self.mode_before = "pass-through"
        self.pending: list = []                 # [(deliver_t, kind, payload)] — wire order

    def as_message(self, ap: tuple) -> Link:
        """The `/link` message: what the antenna sees, with the AP's position in it.

        `ap` is in the message on purpose. A student controller that knows where the access point is
        can decide to drive back into it *before* the failsafe fires — the whole difference between
        a robot that reports its link and one that is rescued by it.
        """
        return Link(t=0.0, quality=self.quality, rssi_dbm=self.rssi_dbm, ap=tuple(ap),
                    up=not self.down, dropped=self.dropped, latency_ms=1000.0 * self.latency_s)


class Wifi:
    """One access point, the link budget above it, and the delivery rules below it.

    Built once per simulation by `SimEngine._make_wifi()` — and not at all when `wifi.enabled` is
    false, which is the default and the reason the graded command streams of both experiments are
    unchanged. One `LinkState` per robot, created on the robot's first command or first step.

    The engine is the only caller: `step()` for one robot per physics step, `admit()` when something
    arrives from the bus, `due()` for what the wire has finished delivering, `message()` for the
    rate-limited `/link`. Nothing here touches a chassis or a topic — the radio decides *whether* and
    *when*, the engine decides *what that does to a motor*.
    """

    def __init__(self, world, ap, noise: Noise, cfg: dict | None = None):
        cfg = cfg or {}
        self.world, self.noise = world, noise
        self.ap = ap_point(ap, world)
        self.n = float(cfg.get("n", 2.4))
        self.tx_dbm = float(cfg.get("tx_dbm", -34.0))
        self.d0 = max(float(cfg.get("d0", 1.0)), EPS)
        self.wall_db = float(cfg.get("wall_db", 12.0))
        self.floor_dbm = float(cfg.get("floor_dbm", FLOOR_DBM))
        self.good_dbm = float(cfg.get("good_dbm", GOOD_DBM))
        self.link_up_q = min(max(float(cfg.get("link_up_q", 0.15)), 0.0), 1.0)
        self.link_timeout = max(float(cfg.get("link_timeout", 1.5)), 0.0)
        self.latency_ms = max(float(cfg.get("latency_ms", 20.0)), 0.0)
        self.shadow_db = max(float(cfg.get("shadow_db", 3.0)), 0.0)
        self.shadow_period = max(float(cfg.get("shadow_period", 2.0)), 1e-3)
        self.rate = float(cfg.get("rate", 5.0))
        # The onboard rule: what the robot does with itself while nothing is delivered. "stop" lets
        # the command watchdog do it (which is what a real failsafe is), "dead_reckoning" replays the
        # last command that really arrived.
        self.autonomy = str(cfg.get("autonomy", "stop"))
        self.states: dict[str, LinkState] = {}

    # ------------------------------------------------------------------------- the budget

    def walls_between(self, x: float, y: float) -> int:
        """How many wall rectangles the straight line AP → (x, y) runs through.

        The LIDAR's own entry test, so a beam and a radio wave disagree about nothing but the
        frequency: the same rectangles, the same `world.walls`, no second geometry to keep in sync.
        A rectangle the point is standing *in* counts as entered at 0.0 and is not a crossing — the
        robot is not inside a table, and if it is, the collision handler has that problem, not this.
        """
        ax, ay = self.ap
        dist = math.hypot(x - ax, y - ay)
        if dist <= EPS:
            return 0
        ux, uy = (x - ax) / dist, (y - ay) / dist
        return sum(1 for wall in self.world.walls or []
                   if (found := _ray_rect(ax, ay, ux, uy, wall)) is not None
                   and EPS < found[0] < dist - EPS)

    def budget(self, x: float, y: float, shadow_db: float = 0.0) -> tuple:
        """(rssi, quality, metres, wall crossings) at one spot — the model, with its terms visible.

        Public because the readout line and the tests want the terms, not one fused number: "q 0.42,
        12.3 m, 1 wall" is a sentence a student can check with a tape measure, q alone is not.
        """
        ax, ay = self.ap
        dist = math.hypot(x - ax, y - ay)
        walls = self.walls_between(x, y)
        rssi = (self.tx_dbm - 10.0 * self.n * math.log10(max(dist, self.d0) / self.d0)
                - walls * self.wall_db + shadow_db)
        return rssi, quality_of(rssi, self.floor_dbm, self.good_dbm), dist, walls

    def coverage(self, step: float = None) -> list:
        """The hall sampled on a grid — `[(x, y, quality, walls)]`, the data of the coverage layer.

        `step` defaults to the cell of the world, so `production` (40 × 24 m at 0.5 m) is 3840 spots
        and one ray test each. Measured on this machine: 0.06 s for the whole hall, once, which is
        why the window samples on the first frame that shows the layer and keeps the answer (§6.14).

        Sampled with `shadow_db = 0` on purpose. The slow fade is a few dB walking in and out of a
        corridor; a map that was redrawn with it would be a minute out of date and would make the
        cached picture a lie. The live number, fade included, is the bar of each robot in the network
        panel and in `/link` — the map is the shape of the room, not the weather.
        """
        step = float(step or (self.world.cell or 0.5))
        wide, high = self.world.size
        out = []
        y = step / 2.0
        while y < high:
            x = step / 2.0
            while x < wide:
                _rssi, q, _d, walls = self.budget(x, y)
                out.append((x, y, q, walls))
                x += step
            y += step
        return out

    def latency(self, quality: float) -> float:
        """Seconds on the wire at this quality: `latency_ms * (1 + 2 (1 - q))`.

        Three times as bad at q = 0 as at q = 1 — the classic "a loaded, marginal link is slow
        before it is gone" — and always at least the configured floor, so a good link is not free.
        """
        return 1e-3 * self.latency_ms * (1.0 + 2.0 * (1.0 - quality))

    def drop_chance(self, quality: float) -> float:
        """`p_drop = (1 - q)²`: q 0.90 loses 1 frame in 100, q 0.50 one in 4, q 0.20 one in 40."""
        return (1.0 - quality) ** 2

    # ---------------------------------------------------------------------- per robot state

    def state(self, name: str) -> LinkState:
        """The robot's link state, created on first sight (a robot that never drove is not missed)."""
        if name not in self.states:
            self.states[name] = LinkState(name, self.noise, self.shadow_db, self.shadow_period)
        return self.states[name]

    def forget(self, name: str) -> None:
        """Drop one robot's radio history — what was on the wire to it is not coming back."""
        self.states.pop(name, None)

    def reset(self) -> None:
        """A new run: no counters, nothing on the wire — the old drive's frames are not this one's."""
        self.states.clear()

    def step(self, name: str, pose, dt: float) -> LinkState:
        """One physics step for one robot: fade, budget, and the timer that decides up or down.

        The timer counts **simulation** seconds and only while q is under `link_up_q`, so the failsafe
        is `link_timeout` sim seconds after the level dropped — never after the wall clock, and never
        while a robot is only passing through a bad spot.
        """
        st = self.state(name)
        st.shadow_db = st.fade.update(dt)
        st.rssi_dbm, st.quality, st.distance, st.walls = self.budget(pose.x, pose.y, st.shadow_db)
        if st.quality < self.link_up_q:
            st.low_for += dt
        else:
            st.low_for = 0.0
        st.down = st.low_for > self.link_timeout
        return st

    # ------------------------------------------------------------------- delivery of commands

    def admit(self, name: str, pose, kind: str, payload, t: float) -> bool:
        """Put one external command on the wire; True if the radio took it.

        The coin is tossed at the moment the frame arrives at the AP, against the link budget
        *recomputed for that moment* (`step()` samples it once per physics step, and the first frame
        of a run arrives before the first step): a robot cannot retroactively improve a frame it never
        sent, and a frame must not be judged by the level of a spot the robot has already left.
        A link that is *down* delivers nothing at all, and both cases land in the same counter: from
        the outside a dropped frame and an unanswerable link look the same, which is exactly the
        student's problem.
        """
        st = self.state(name)
        if st.down:
            st.dropped += 1
            return False
        _rssi, quality, _metres, _walls = self.budget(pose.x, pose.y, st.shadow_db)
        if self.noise.chance(self.drop_chance(quality)):
            st.dropped += 1
            return False
        st.latency_s = self.latency(quality)
        st.pending.append((t + st.latency_s, kind, payload))
        return True

    def due(self, name: str, t: float) -> list:
        """The commands the wire has finished delivering, in the order they were sent.

        Head only, and only while it is due: with a quality-dependent latency a later frame could
        overtake an earlier one, which a real link does all the time. A student debugging a drive with
        `ros2 topic echo` cannot use that, so this link delivers in order and the command sequence on
        the robot is the sequence they wrote — only shifted in time.
        """
        st = self.states.get(name)
        if not st:
            return []
        out = []
        while st.pending and st.pending[0][0] <= t + EPS:
            _due, kind, payload = st.pending.pop(0)
            st.delivered += 1
            out.append((kind, payload))
        return out

    # --------------------------------------------------------------------- what the GUI asks

    def message(self, name: str) -> Link | None:
        """The next `/link` payload — None for a robot this radio has never seen."""
        st = self.states.get(name)
        return st.as_message(self.ap) if st else None

    def health(self, name: str) -> tuple:
        """(quality, dBm, metres, walls, up, dropped, sent, latency_ms, ap) for the readout line.

        `sent` is everything the radio was ever asked to deliver (`dropped + delivered`), so the two
        together read as a fraction: `lost 4/120` is the sentence a student writes in the logbook.

        Asked of the radio and never of the last message, in the rule of `SimEngine.gps_health()`: the
        interesting moment is the one where nothing is being delivered, and there is no fresh message
        to read then. Draws no random number, so the window may ask every frame.
        """
        st = self.states.get(name)
        if st is None:
            return (1.0, GOOD_DBM, math.hypot(self.ap[0], self.ap[1]), 0, True, 0, 0, 0.0, self.ap)
        return (st.quality, st.rssi_dbm, st.distance, st.walls, not st.down, st.dropped,
                st.dropped + st.delivered, 1000.0 * st.latency_s, self.ap)
