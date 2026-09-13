"""Pygame view of the simulator: reads engine.world and engine.robots, changes nothing.

Three parts, and only this module draws:

* `cam.py`  scale and centre of the view — wheel zoom at the cursor, drag pan, resize, `f`
* `menu.py`  which sensor layers are drawn; switching a layer off never touches the bus
* here      the layers themselves: floor, walls, scan, trail, estimate, robot with wheels

The world is drawn as a closed box: everything outside `world.size` is void, so a missing
border wall is visible as a gap instead of looking like more floor. Walls are filled blocks
without an outline — with a 1 m grid the outline read as a second, thinner wall. 1..9 jumps
to a robot, 0 shows everything again. Physics and ROS are never touched.
"""
import logging
import math

import pygame

from . import cam as kamera
from . import keys
from . import menu as menue
from . import overlays                                  # shadow, ghost, rubber
from . import pois                                      # the field behind the dose map
from .types import cfg_get

log = logging.getLogger("mecanum.render")

GREY = (210, 212, 218)
GPS_COLOR = (250, 210, 90)                         # measurement: flat and angular
KF_COLOR = (120, 240, 170)                         # estimate: bright and round
KF_SIGMA = 2.0                                # how many σ the GUI ellipse shows
CORNERS = ((1, 1), (1, -1), (-1, 1), (-1, -1))     # FL, FR, RL, RR in the body frame
# Roller axis of each wheel, body frame, same order as `CORNERS` (= `physics.WHEELS`). These four
# axes are the **X arrangement** that `physics.inverse_kinematics()` implements: a wheel driven
# along (1,-1) carries its rollers along (1,1), so FL and RR sit on one diagonal and FR and RL on
# the other. The other diagonal is the O arrangement — the same robot with the rollers mirrored,
# whose wheels would strafe the wrong way for every command in the readout.
ROLLERS = ((1, 1), (1, -1), (1, -1), (1, 1))
WHEEL_STROKES = 3                                     # strokes drawn per wheel
WHEEL_SPIN_GAIN = 0.25                              # wheel radius -> on-screen rotation, illustrative
WHEEL_WIDTH_RATIO = 0.6           # drawn width of a wheel, as a fraction of its drawn length
WHEEL_MIN_PX = 3.0               # below this a wheel is one pixel and says nothing about rollers
# Marker per robot as a list of (radius, angle in degrees); radius 1 = footprint.
# The marker of a robot, authored in the **body frame**: angle 0 is forward, counter-clockwise
# positive — the same convention as the wheels, the kinematics and the topics. The tip of the
# triangle and the point of the pentagon are at 0 because a marker that points somewhere else than
# the robot drives is a second heading indicator that contradicts the first.
SHAPES = {
    "triangle": [(1, 0), (1, 120), (1, 240)],
    "square": [(1.2, a) for a in (45, 135, 225, 315)],
    "diamond": [(1.25, a) for a in (0, 90, 180, 270)],
    "circle": [(1, a) for a in range(0, 360, 30)],
    "pentagon": [(1.15, a) for a in range(0, 360, 72)],
    "hexagon": [(1.15, a) for a in range(0, 360, 60)],
    "star": [v for a in range(0, 360, 36) for v in ((1.35, a), (.55, a + 18))],
    "cross": [v for a in range(0, 360, 45) for v in ((1.3, a), (.5, a + 22.5))],
}
# name -> (attribute toggled, edge name in the poll() dict). `keys.LAYERS` is the table of the
# window, this is the same rows plus the pause key — one dict for the event handler, no second list
# that could drift away from the panel and from the help line.
KEYS = {keys.PAUSE: ("paused", keys.PAUSE_EDGE), **keys.layer_table()}


def body(theta: float, dx: float, dy: float) -> tuple:
    """Rotate a vector from the body frame (x forward, y left) into the world."""
    c, s = math.cos(theta), math.sin(theta)
    return (dx * c - dy * s, dx * s + dy * c)


def screen(theta: float, dx: float, dy: float) -> tuple:
    """A body-frame direction as a vector **on the screen**: x right, y down.

    The world is right-handed with y to the left, the screen is not: its y grows downwards. So the
    screen direction of a body vector is `body(-theta, dx, -dy)` — the mirror of the handy form that
    was used here for years, which agreed for vectors along the body x-axis and quietly mirrored
    the ones with a y component. Harmless for the chassis plate — a rectangle looks the same with
    one of its axes flipped — and not harmless at all for a roller axis at 45°: the X arrangement of
    the wheels came out as an O, in the one picture a student is meant to learn the kinematics from.
    Everything that draws a *direction* goes through here; `px()` remains the one that draws a
    *position*, because that one already flipped y correctly.
    """
    return body(-theta, dx, -dy)


def rgb(color, factor: float = 1.0) -> tuple:
    """Color from types.PALETTE (0..1) -> pygame (0..255), optionally darkened."""
    return tuple(min(255, int(255 * v * factor)) for v in color)


def _value(color: tuple) -> list:
    """One color in either space this file uses -> 0..255, still a float, nothing clamped yet."""
    return [255 * v if v <= 1 else v for v in color]


def to255(color: tuple) -> tuple:
    """A color in either space this file uses -> the 0..255 tuple pygame paints with.

    Two spaces are in use here: `types.PALETTE` and the literal HUD colors are written 0..1,
    `rgb()` and `mix()` hand back 0..255. pygame does not read the first one — it truncates, so
    `(1, .85, .3)`, the amber of the goal marker, reaches the font as `(1, 0, 0)` and paints black
    letters on a floor that is itself near black: measured 1.3:1, which is not a dim label but an
    invisible one. `mix()` has accepted both spaces for the same reason (its note below); the rule
    belongs in one function instead of in the memory of every caller.
    """
    return tuple(min(255, int(v)) for v in _value(color)[:3])


def mix(a: tuple, b: tuple, part: float = .5) -> tuple:
    """Mix two colors; part 0..1 in favor of b.

    Accepts both colour spaces used in this file — `types.PALETTE` values (0..1) and colors
    already converted by `rgb()` (0..255). Without that, `mix(rgb(spec.rgb), ...)` multiplied by
    255 a second time and drew everything near white, which is how a red robot got a white box.
    """
    a, b = _value(a), _value(b)
    return tuple(int((1 - part) * x + part * y) for x, y in zip(a, b))


def _add(p: tuple, d: tuple) -> tuple:
    return (p[0] + d[0], p[1] + d[1])


def wheel_mounts(lx: float, ly: float) -> list:
    """Wheel centres in the body frame, in **metres**, in the order of `physics.WHEELS`.

    A wheel is drawn where it sits on the axle line — `lx` fore and aft, `ly` to the sides — and
    only its radius is exaggerated afterwards (`gui_style.wheel_scale`). Never its position: the
    lever arm between the wheels is the `arm` of `inverse_kinematics()`, so a wheel painted into
    the corner of the plate instead of on the axle line shows a student a robot that turns about a
    point its physics does not have. Both drive trains land on the same four points — for the
    steering car `lx` is half the wheel base and `ly` half the track (`steering.SteeringGeometry`).
    """
    return [(sx * lx, sy * ly) for sx, sy in CORNERS]


def shape(name: str, centre: tuple, radius: float, theta: float) -> list:
    """Corner points of one marker shape in pixels; an unknown name is drawn as a circle.

    One corner of `SHAPES` is a body-frame direction, `screen()` makes it a screen direction, and
    the two are multiplied by the same radius for every corner — so the shape turns with the robot
    and never stands mirrored, whatever the heading.
    """
    corners = []
    for span, angle in SHAPES.get(name) or SHAPES["circle"]:
        reach = radius * span
        direction = screen(theta, math.cos(math.radians(angle)), math.sin(math.radians(angle)))
        corners.append((centre[0] + reach * direction[0], centre[1] + reach * direction[1]))
    return corners


COVERAGE_COLD = (.62, .24, .22)        # layer `c`: nothing usable arrives at this spot
COVERAGE_MID = (.88, .70, .28)
COVERAGE_WARM = (.34, .78, .48)        # ... and here the access point has the whole room


def coverage_tone(quality: float) -> tuple:
    """Red through amber to green over a quality of 0..1 — one ramp, no legend needed to read it.

    Answers in 0..255 because `mix()` scales its 0..1 arguments there: a caller that runs the result
    through `rgb()` again gets white, which is the mistake this line of comment is about.
    """
    q = min(max(quality, 0.0), 1.0)
    return (mix(COVERAGE_COLD, COVERAGE_MID, q * 2) if q < .5
            else mix(COVERAGE_MID, COVERAGE_WARM, (q - .5) * 2))


# The dose ramp is the warm half of the ramp above, read the other way round: there green is "the
# access point has the room", here the warm end is "a counter there has plenty to count". Deliberately
# no fourth colour — a second palette in one window is a second legend, and amber is already the colour
# of the source symbol and of its field rings.
DOSE_WARM = COVERAGE_MID           # layer `i`: weak field, the counter still ticks
DOSE_HOT = COVERAGE_COLD           # ... and here it is at the source


def dose_tone(level: float) -> tuple:
    """Amber to red over a dose level of 0..1 (`pois.field_map()`), 0..255 as `coverage_tone()`.

    Only the warm half of the coverage ramp, because a dose has no good end: painting the far corner
    of the hall green would say "measured, harmless" where the honest statement is "the source does
    not reach here", and that is the colour of the floor the map is painted on.
    """
    return mix(DOSE_WARM, DOSE_HOT, min(max(level, 0.0), 1.0))


# ------------------------------------------------------------------ the layers a run starts with
# The short name of a layer in the config (`view.layers`) is the name of its attribute without the
# `show_` prefix, so there is one list of layers in this package — `keys.LAYERS` — and this module
# derives its names from it. `tests/test_view_menu_c.py` fails when the two stop agreeing.
LAYER_NAMES = tuple(attribute[len("show_"):] for attribute in keys.LAYER_ATTRIBUTES)

# The raw measurements are drawn only when somebody asks for them. Their numbers are on the topics,
# in the readout line and in the measurement log whether they are painted or not, and a student who
# has not yet interpreted a lidar scan learns nothing from dots they cannot read — what the window
# is for in the first minutes is the robot, its wheels and where it thinks it is.
RAW_LAYERS = ("scan", "trails", "gps", "ghost")

# Off in every profile but `sensors`, and not because they are measurements: the coverage map and the
# dose map are pictures of a *model* — a log-distance law and an inverse-square law, drawn over a floor
# plan — and a default view that paints a model over the floor teaches the model instead of the robot.
# `c` and `i` in the window, `--layers coverage` / `--layers dose` on the command line bring them into a
# run when somebody wants to see them.
HIDDEN_BY_DEFAULT = RAW_LAYERS + ("coverage", "dose")

VIEW_PROFILES = {"clean": tuple(n for n in LAYER_NAMES if n not in HIDDEN_BY_DEFAULT),
                 "sensors": LAYER_NAMES}
VIEW_PROFILE = "clean"                       # the view of a run nobody configured


def view_state(cfg: dict) -> dict:
    """attribute -> drawn: the profile named in `view.profile`, with `view.layers` written over it.

    Unknown layer names are ignored loudly rather than silently: a config that says
    `"layer": "lidar"` when the name is `scan` would otherwise look as if the layer existed and did
    nothing.
    """
    profile = str(cfg_get(cfg, "view.profile", VIEW_PROFILE) or VIEW_PROFILE)
    if profile not in VIEW_PROFILES:
        log.warning("unknown view profile '%s' — there is %s, taking '%s'", profile,
                    " and ".join(sorted(VIEW_PROFILES)), VIEW_PROFILE)
        profile = VIEW_PROFILE
    on = set(VIEW_PROFILES[profile])
    for name, wanted in (cfg_get(cfg, "view.layers", {}) or {}).items():
        if name not in LAYER_NAMES:
            log.warning("view layer '%s' does not exist — there is %s", name, ", ".join(LAYER_NAMES))
            continue
        on = (on | {name}) if wanted else (on - {name})
    return {f"show_{name}": name in on for name in LAYER_NAMES}


class Renderer:
    """One frame per draw(), key presses per poll(); state here, decisions there."""

    def __init__(self, engine, cfg: dict):
        style = cfg_get(cfg, "gui_style") or {}
        self.engine, self.cfg = engine, cfg
        self.pointer = None                      # last mouse position in the window, or None
        self._coverage_cache = None              # ((world, ap), samples) — see `_coverage()`
        self._dose_cache = None                  # ((world, sources), samples) — see `_dose()`
        self.col_wall = rgb(style.get("wall", (.34, .36, .42)))
        self.col_floor = rgb(style.get("floor", (.13, .14, .17)))
        self.col_void = rgb(style.get("void", (.06, .065, .08)))     # outside the world
        self.trail_len = int(style.get("trail_len", 400))
        self.wheel_scale = float(style.get("wheel_scale", 2.0))    # wheels: drawn bigger than 5 cm
        self.chassis_scale = float(style.get("chassis_scale", 1.0))  # body box, in addition to lx/ly
        # Every layer starts as the configuration says (`view_state` above), not as a hard-coded
        # True: what a student sees on first start is then a setting with a name, and a demo config
        # can ask for the layer it is about. Keys and clicks still switch anything, at any time.
        for attribute, drawn in view_state(cfg).items():
            setattr(self, attribute, drawn)
        self.teleop = False                          # node.run_loop sets this, changes the help
        self.paused = False
        self.kf_trail, self.trails, self.phase = {}, {}, {}           # per robot name
        self.ghost_trail = {}                      # where odometry believes each robot has been
        self.focus = None
        self.size = (int(cfg_get(cfg, "width", 1120)), int(cfg_get(cfg, "height", 700)))
        try:                                                    # deliberately no pygame.init():
            pygame.display.init()                               # the view needs neither
            pygame.font.init()                                  # audio nor joystick
        except pygame.error as exc:
            raise RuntimeError("no display — set SDL_VIDEODRIVER=dummy for headless runs"
                               ) from exc
        self.screen = pygame.display.set_mode(self.size, pygame.RESIZABLE)
        self.cam = kamera.Camera(self.size, engine.world.size or (10, 10),
                                 px_per_meter_min=float(style.get("px_per_meter_min", 50)))
        pygame.display.set_caption(f"mecanum_lab — {engine.world.name}")
        pygame.event.get()                                      # drop events of an old window
        self.font, self.big = pygame.font.Font(None, 17), pygame.font.Font(None, 21)
        self.menu = menue.Panel(self.font, self.big)
        # The right-button menu: one row per robot, opened at the pointer (see `_open_picker`).
        # Empty until it opens — which robots are driving is not known while the window is built.
        self.pick = menue.Panel(self.font, self.big, rows=(), head="place robot",
                                foot="click a robot · right button or esc closes",
                                at_pointer=True)
        self._pick_spot, self._pick_where = None, (0, 0)
        self.clock = pygame.time.Clock()
        self._drag, self._frame = None, 0
        self._t, self._closed, self._alive = float(engine.t), False, True
        self._events = {pygame.QUIT: self._ev_quit,
                            pygame.VIDEORESIZE: self._ev_resize,
                            pygame.MOUSEBUTTONDOWN: self._ev_mouse_down,
                            pygame.MOUSEMOTION: self._ev_mouse_motion,
                            pygame.MOUSEBUTTONUP: self._ev_mouse_up,
                            pygame.MOUSEWHEEL: self._ev_wheel,
                            pygame.KEYDOWN: self._event_key}

    # ------------------------------------------------------------------------- Kamera-Fassade
    @property
    def zoom(self) -> float:
        return self.cam.zoom

    @property
    def s(self) -> float:
        return self.cam.s

    @property
    def ox(self) -> float:
        return self.size[0] / 2 - self.cam.cx * self.s

    def px(self, x: float, y: float) -> tuple:
        """World meters -> pixels (the camera knows centre and scale)."""
        return self.cam.px(x, y)

    def _cam(self) -> None:
        """Scale for this frame; a focused robot stays in the middle unless one is dragging."""
        self.cam.update(self.engine.world.size or (10, 10))
        bots = list(self.engine.robots.values())
        if self.focus is not None and self.focus < len(bots) and not self._drag:
            pose = bots[self.focus].pose
            self.cam.center_on(pose.x, pose.y)
        elif self.focus is None:
            self.cam.clamp()

    def _text(self, txt, x, y, color, big=False, center=False, halo: bool = True) -> int:
        """One piece of text on the screen — every label of the window comes through here.

        Two things happen between a caller and the font, both because a label is drawn over
        whatever the map happens to be at that spot:

        `to255()` — the callers hand over both colour spaces (the HUD writes 0..1, `rgb()` and
        `mix()` write 0..255) and pygame truncates the first one into black, see `to255()`.

        `halo` — a one-pixel dark outline behind the letters, the way a map keeps its place names
        readable. On floor and void it is invisible (dark on dark, and the window is dark), but a
        wall is (86, 91, 107) and no colour of `types.PALETTE` clears 4.5:1 on that: the name of a
        red robot driving along a wall was 1.98:1, and in a hall full of walls a robot drives along
        walls most of the time. Darkening the pixels directly behind the letters helps over every
        background at once, where a brighter hue helps over one and fails over the next. It costs
        one extra render and four blits per label.
        """
        schrift = self.big if big else self.font
        img = schrift.render(txt, True, to255(color))
        ziel = img.get_rect(center=(x, y)) if center else img.get_rect(topleft=(x, y))
        if halo:
            rand = schrift.render(txt, True, self.col_void)
            for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                self.screen.blit(rand, ziel.move(dx, dy))
        self.screen.blit(img, ziel)
        return img.get_width()

    # ------------------------------------------------------------------------------ ein Frame

    def draw(self, cap: bool = True) -> None:
        """One complete frame. cap=False drops the frame-rate cap (for measurements)."""
        eng = self.engine
        dt = max(0.0, min(float(eng.t) - self._t, .5))      # only simulation time drives the rollers
        self._t, self._frame = float(eng.t), self._frame + 1
        self._cam()
        self.screen.fill(self.col_void)
        self._world()
        if self.show_zones:
            overlays.zones(self)                              # where the GPS gets bad
        if self.show_pois:
            overlays.poi_sources(self)                     # sources: measured, rarely seen
        if self.show_network:
            overlays.network(self)                    # the access point and what it is worth
        self.trails = {k: v for k, v in self.trails.items() if k in eng.robots}
        self.kf_trail = {k: v for k, v in self.kf_trail.items() if k in eng.robots}
        self.ghost_trail = {k: v for k, v in self.ghost_trail.items() if k in eng.robots}
        self.phase = {k: v for k, v in self.phase.items() if k in eng.robots}
        for robot in eng.robots.values():
            overlays.skid_marks(self, robot, dt)              # rubber under slipping wheels
            self._trail(robot)                              # kept while hidden, not thrown away
            self._ghost_trail(robot)                        # same rule for the believed pose
            if self.show_ghost:
                overlays.odom_ghost(self, robot)                  # before the real chassis
            if self.show_scan:
                self._scan_dots(robot)
            if self.show_gps:
                self._measured_point(robot)
            if self.show_kf:
                self._estimate(robot)
        for robot in eng.robots.values():
            self._robot(robot, dt)
            overlays.autonomy_mark(self, robot)     # amber outline, no layer: a fact, not a view
        if self.show_hud:
            self._hud()
        self.menu.draw(self.screen, (self.size[0], 52), self)
        self.pick.draw(self.screen, self._pick_where, self)
        if self.paused:
            self._text("PAUSE", self.size[0] / 2, 24, (250, 220, 120), True, True)
        self._pointer_coords()
        pygame.display.flip()
        if cap:
            self.clock.tick(int(cfg_get(self.cfg, "gui_rate", 30) or 30))

    def _world(self) -> None:
        """Floor inside the world box, void outside, walls as filled blocks, goal."""
        w, sc = self.engine.world, self.screen
        box = pygame.Rect(self.cam.rect)
        pygame.draw.rect(sc, self.col_floor, box)
        if self.show_coverage:
            self._coverage(sc, box)              # what the room does to a wave, under everything
        if self.show_dose:
            self._dose(sc, box)                  # what the room does not do to a field
        for wall in w.walls or []:
            # floor/ceil instead of int: neighbouring blocks then touch each other instead of
            # leaving a 1 px floor seam between them — that seam would read as a thin wall again.
            top, bottom = self.px(wall.x0, wall.y1), self.px(wall.x1, wall.y0)
            corner = (math.floor(top[0]), math.floor(top[1]))
            pygame.draw.rect(sc, self.col_wall, pygame.Rect(
                corner, (max(1, math.ceil(bottom[0]) - corner[0]),
                       max(1, math.ceil(bottom[1]) - corner[1]))))
        pygame.draw.rect(sc, mix(self.col_wall, (1, 1, 1), .3), box, 2)   # the world ends here
        if w.goal and self.show_goal:
            centre = self.px(w.goal.x, w.goal.y)
            for ring, radius in enumerate((16, 10, 4)):                       # bullseye
                pygame.draw.circle(sc, mix((1, .85, .3), (1, 1, 1), ring / 3), centre, radius,
                                   2 if ring else 0)
            self._text("goal", centre[0] - 12, centre[1] - 32, (1, .85, .3))

    def _coverage(self, sc, box: pygame.Rect) -> None:
        """Layer `c`: the hall painted by signal quality, one square per sample — off by default.

        Sampled once per room and access point (`wifi.coverage()` says what that costs and why the
        slow shadow fade is not in it), then drawn *under* the walls, so a rack that shadows a corner
        is painted over the corner it shadows: the map explains the room and never competes with it.

        The ramp is the quality of §6.14 itself, not a second scale — 1.0 where the access point has
        the room, 0.0 where nothing usable arrives. Where commands stop arriving is `wifi.link_up_q`,
        which the bar of each robot in the network panel is measured against: the map answers "would a
        robot over *there* still hear me", the bar answers "does *this* one".
        """
        radio = self.engine.wifi
        if radio is None:
            return                                  # no radio in this run, so no map of one
        key = (id(self.engine.world), tuple(radio.ap))
        if not self._coverage_cache or self._coverage_cache[0] != key:
            self._coverage_cache = (key, radio.coverage())
        step = self.engine.world.cell or 0.5
        for x, y, quality, _walls in self._coverage_cache[1]:
            # floor/ceil of both corners, as in `_world()`: two cells drawn with int() leave a 1 px
            # seam of untouched floor between them, and a map of a field should not be a grid.
            top, bottom = self.px(x - step / 2, y + step / 2), self.px(x + step / 2, y - step / 2)
            cell = pygame.Rect((math.floor(top[0]), math.floor(top[1])),
                               (max(1, math.ceil(bottom[0]) - math.floor(top[0])),
                                max(1, math.ceil(bottom[1]) - math.floor(top[1]))))
            if cell.colliderect(box):
                # The wash grows with the signal: a dead corner stays the colour of the floor it is,
                # a spot the access point reaches is painted in the colour of the ramp. A constant
                # 50 % mix of a saturated tone and a dark blue floor is a light grey everywhere, which
                # is a picture of nothing — the first version of this layer did exactly that.
                # `coverage_tone()` answers in 0..255 like every `mix()` result here, so it goes in
                # as it is: putting it through `rgb()` a second time is the near-white floor that the
                # docstring of `mix()` warns about, and that is what this layer drew first.
                pygame.draw.rect(sc, mix(self.col_floor, coverage_tone(quality),
                                         0.18 + 0.5 * quality), cell)

    def _dose(self, sc, box: pygame.Rect) -> None:
        """Layer `i`: the hall painted by what a counter would read there, one square per cell.

        Sampled once per world and source list (`pois.field_map()` says what that costs and why the
        answer is kept) and painted **under the walls**, like the coverage map — for the opposite
        reason. The radio map goes under the racks because a rack shadows the wave and should stay
        visible on top of the shadow it casts; the dose goes under the walls because the counter model
        has no wall in it at all, so the field runs through the rack unchanged and the wall drawn over
        the paint is the one thing left that tells a student where in the hall they are standing.

        The wash grows with the field, as in `_coverage()`: a corner no source reaches stays the colour
        of the floor it is, and colour only comes over the map where there is something to count. 1.0
        is at the source, 0.5 at its `d0`, 0.1 at `3·d0` — the same three numbers the readout line and
        the handout speak about, which is the point: picture and counter cannot disagree.
        """
        sources = overlays.poi_list(self)
        if not sources:
            return                                  # no source in this world, so no field to paint
        key = (id(self.engine.world),
               tuple((s.name, round(s.x, 3), round(s.y, 3), s.activity, s.range_m, s.d0)
                     for s in sources))
        if not self._dose_cache or self._dose_cache[0] != key:
            self._dose_cache = (key, pois.field_map(sources, self.engine.world))
        step = self.engine.world.cell or 0.5
        for x, y, level in self._dose_cache[1]:
            if level <= 0.0:
                continue            # zero field: the floor already says that, in the right colour
            top, bottom = self.px(x - step / 2, y + step / 2), self.px(x + step / 2, y - step / 2)
            cell = pygame.Rect((math.floor(top[0]), math.floor(top[1])),
                               (max(1, math.ceil(bottom[0]) - math.floor(top[0])),
                                max(1, math.ceil(bottom[1]) - math.floor(top[1]))))
            if cell.colliderect(box):
                pygame.draw.rect(sc, mix(self.col_floor, dose_tone(level), 0.16 + 0.52 * level), cell)

    def _pointer_coords(self) -> None:
        """The world coordinate under the pointer, written beside the pointer.

        `Camera.wx()` is the inverse of the mapping the frame is drawn with — the same one
        `zoom_to_at()` uses — so the number cannot drift away from the picture, and a student who
        clicks a spot and reads the number can check both. Shown only inside the world box: outside it
        the number would count metres of void, and outside the window there is nobody to show it to.

        Under SDL's dummy driver the pointer never moves, so the documentation pictures never grow a
        coordinate — which is the reason this is drawn from the last motion event rather than from
        `pygame.mouse.get_pos()`, which would ask the window manager what the mouse is doing.
        """
        if not self.show_hud:
            return
        if label := self.pointer_label():
            self._text(label, self.pointer[0] + 12, self.pointer[1] + 16, (1, .92, .70))

    def pointer_label(self) -> str:
        """The coordinate of the pointer as text, or `''` where there is nothing to read.

        Separate from the drawing for the same reason the readout line is: the string is a claim about
        the mapping (`Camera.wx()` is the inverse of the mapping the frame is drawn with) and can be
        checked with a ruler, while where it is painted is a matter of taste.
        """
        if not self.pointer or not pygame.Rect(self.cam.rect).collidepoint(self.pointer):
            return ""
        x, y = self.cam.wx(*self.pointer)
        return f"{x:.2f}, {y:.2f} m"

    def _scan_dots(self, robot) -> None:
        """Hit points in the robot's own color — the screen then says which dots are whose."""
        scan = robot.scan
        if not scan or not scan.ranges:
            return
        color, sc, size = rgb(robot.spec.rgb), self.screen, 3 if self.s < 90 else 4
        for i in range(0, len(scan.ranges), max(1, len(scan.ranges) // 180)):
            rng = scan.ranges[i]
            if not math.isfinite(rng) or rng <= scan.range_min:     # inf/nan = no hit
                continue
            wx, wy = _add((robot.pose.x, robot.pose.y),
                          body(robot.pose.theta + scan.angle_min + i * scan.angle_increment,
                               rng, 0))
            p = self.px(wx, wy)
            pygame.draw.rect(sc, color, (p, (size, size)))

    def _measured_point(self, robot) -> None:
        """The raw GPS fix as a cross — its own layer, so `g` works without `k`."""
        if robot.gps:
            px = self.px(robot.gps.x, robot.gps.y)
            pygame.draw.line(self.screen, GPS_COLOR, (px[0] - 5, px[1]), (px[0] + 5, px[1]), 2)
            pygame.draw.line(self.screen, GPS_COLOR, (px[0], px[1] - 5), (px[0], px[1] + 5), 2)

    def _estimate(self, robot) -> None:
        """Experiment 2: own estimate (diamond + σ ellipse) and the error tick to the truth.

        The ellipse is no decoration: report a σ that is too small and you see a filter
        running next to the truth while drawing a tiny scatter circle — exactly the
        self-deception that task K3 penalizes.
        """
        sc = self.screen
        kf = robot.kf
        if not kf:
            return
        trail = self.kf_trail.setdefault(robot.spec.name, [])
        if not trail or math.dist((kf.x, kf.y), trail[-1]) > .02:
            trail.append((kf.x, kf.y))
            del trail[:-max(1, self.trail_len)]
        if len(trail) > 1:
            pygame.draw.lines(sc, KF_COLOR, False, [self.px(x, y) for x, y in trail], 2)
        center, radius = self.px(kf.x, kf.y), max(4, KF_SIGMA * self.s * max(kf.sx, 1e-4))
        height = max(4, KF_SIGMA * self.s * max(kf.sy, 1e-4))
        pygame.draw.ellipse(sc, KF_COLOR, (center[0] - radius, center[1] - height,
                                          2 * radius, 2 * height), 1)
        pygame.draw.polygon(sc, KF_COLOR, [(center[0], center[1] - 6), (center[0] + 6, center[1]),
                                           (center[0], center[1] + 6), (center[0] - 6, center[1])])
        pygame.draw.line(sc, (240, 120, 120), center, self.px(robot.pose.x, robot.pose.y), 1)

    def _trail(self, robot) -> None:
        pts = self.trails.setdefault(robot.spec.name, [])
        here = (robot.pose.x, robot.pose.y)
        if not pts or math.dist(here, pts[-1]) > .03:
            pts.append(here)
            del pts[:-max(1, self.trail_len)]
        if self.show_trails and len(pts) > 1:
            pygame.draw.lines(self.screen, rgb(robot.spec.rgb, .6), False,
                              [self.px(x, y) for x, y in pts], 2)

    def _ghost_trail(self, robot) -> None:
        """Where odometry believes this robot has been: the same history as `._trail()`, the same cap.

        Written whether or not the layer is drawn, exactly like the truth trail — a line that starts at
        the pose of the moment the layer was switched on would tell a student the drift began now, while
        the whole point of the demo is the error that has been collecting for a minute. Same spacing and
        the same cap as the truth line, so the only difference between the two lines is the odometry
        error itself, and not a drawing decision; dimmer than the ghost plate, which says the same
        position twice.
        """
        o = getattr(robot, "odom", None)
        if not o:
            return
        pts = self.ghost_trail.setdefault(robot.spec.name, [])
        here = (o.x, o.y)
        if not pts or math.dist(here, pts[-1]) > .03:
            pts.append(here)
            del pts[:-max(1, self.trail_len)]
        if self.show_ghost and len(pts) > 1:
            col = mix(rgb(robot.spec.rgb), self.col_floor, .60)
            pygame.draw.lines(self.screen, col, False, [self.px(x, y) for x, y in pts], 2)

    def _robot(self, robot, dt: float) -> None:
        """Chassis, marker, four wheels with rolling strokes, name label.

        Two unit systems meet here and the line between them is drawn on purpose: everything that
        says **where** something is on the floor is in metres and goes through `self.px()`,
        everything that says **how big** a thing is drawn is in pixels. Both are named `half_l`
        and `be` in the version before this one, and the reassignment in the middle of these
        lines put a pixel number into a metre slot: the four wheels came out 10 m in front of and
        behind the robot — off every screen, which is how the wheel picture came to be "broken".

        The **marker is the one heading indicator**. A short line stuck out of the chassis past the
        collision circle used to point forward as well, and a plate with four wheels and a bar in
        front of it reads as a tank: the second indicator bought nothing, because every shape in
        `SHAPES` with a corner forward has that corner on the heading (asserted), and layer `v`
        draws where the robot is actually going.
        """
        pose, col = robot.pose, rgb(robot.spec.rgb)
        sc = self.screen
        lx, ly, wr, fp_m = self._geom(robot)
        centre, fp = self.px(pose.x, pose.y), max(6, fp_m * self.s)
        plate = ((lx + wr) * self.chassis_scale, (ly + wr) * self.chassis_scale)   # metres
        # The one exaggeration of this view: `wheel_scale` times the real 5 cm. The cap is the
        # collision circle — a tyre drawn past the circle the physics collides with would show a
        # robot that cannot fit through the gaps the grader measures it through.
        wheel_r = min(wr * self.wheel_scale, max(0.01, fp_m - ly))                  # metres
        u, v = screen(pose.theta, 1, 0), screen(pose.theta, 0, 1)                 # on screen
        chassis_body = self._plate(centre, u, v, plate[0] * self.s, plate[1] * self.s)
        pygame.draw.polygon(sc, mix(col, self.col_floor, .55), chassis_body)
        pygame.draw.polygon(sc, col, chassis_body, 1)
        pygame.draw.circle(sc, mix(col, self.col_floor, .82), centre, fp, 1)  # collision circle
        if self.show_wheels:
            headings = getattr(robot.chassis, "wheel_headings", [0.0] * 4)
            for i, (mx, my) in enumerate(wheel_mounts(lx, ly)):                # body frame, metres
                self._wheel(robot, pose.theta + headings[i], (mx, my), pose,
                            wheel_r, i, col, dt)
        marker_side = min(plate[0], plate[1]) * self.s
        pygame.draw.polygon(sc, col, shape(robot.spec.marker, centre, marker_side * .6,
                                           pose.theta))
        self._text(robot.spec.name, centre[0], centre[1] - plate[1] * self.s - 14, col,
                   center=True)
        if self.show_velocity:
            speed = math.hypot(robot.twist.vx, robot.twist.vy)
            if speed > .02:                                     # commanded velocity as a beam
                heading = pose.theta + math.atan2(robot.twist.vy, abs(robot.twist.vx) or 1e-6)
                pygame.draw.line(sc, mix(col, (1, 1, 1), .5), centre,
                                 self.px(*_add((pose.x, pose.y), body(heading, speed, 0))), 2)

    def _wheel(self, robot, heading, mount, pose, wheel_r, index, col, dt) -> None:
        """One wheel: tyre along its driving direction, roller strokes travelling along that.

        `mount` is the wheel centre in the **body frame in metres** (`wheel_mounts()`), `heading`
        the direction this wheel points in. The two are separate on purpose: the mount comes with
        the body, the heading belongs to the wheel, so a steered front wheel is drawn where it sits
        but turned by its own angle — and on a mecanum robot both angles are equal, because
        `physics.Chassis.wheel_headings()` is [0, 0, 0, 0] there.

        `wheel_r` is in metres and already exaggerated (`gui_style.wheel_scale`); the roller phase
        is scaled to `WHEEL_SPIN_GAIN`, because 12 rad/s at 30 fps would flicker instead of
        rolling. Both are aids for the eye, neither touches physics or a topic.
        """
        sc = self.screen
        phase = self.phase.setdefault(robot.spec.name, [0.0] * 4)
        phase[index] = (phase[index] + (robot.wheels[index] if index < len(robot.wheels) else 0.0)
                        * dt * WHEEL_SPIN_GAIN) % math.tau
        centre = self.px(*_add((pose.x, pose.y), body(pose.theta, *mount)))
        half_len = max(WHEEL_MIN_PX, wheel_r * self.s)          # pixels, along the rolling direction
        half_wid = max(WHEEL_MIN_PX * .5, half_len * WHEEL_WIDTH_RATIO)   # pixels, across it
        u, v = screen(heading, 1, 0), screen(heading, 0, 1)          # this wheel's axes on screen
        tyre = self._plate(centre, u, v, half_len, half_wid)
        pygame.draw.polygon(sc, mix(col, (0, 0, 0), .55), tyre)
        if self.s > 45:                                           # a 1 px edge below that is mush
            pygame.draw.polygon(sc, col, tyre, 1)
        roller = screen(heading, *ROLLERS[index])
        roller = (roller[0] * half_wid * 1.3, roller[1] * half_wid * 1.3)     # one stroke, full width
        for stroke in range(WHEEL_STROKES):
            share = (phase[index] / math.tau + stroke / WHEEL_STROKES) % 1.0
            walk = (share * 2 - 1) * (half_len - half_wid)         # stays inside the tyre
            here = _add(centre, (u[0] * walk, u[1] * walk))
            pygame.draw.line(sc, mix(col, (1, 1, 1), .35), here,
                             _add(here, roller), max(1, int(half_wid * .3)))

    @staticmethod
    def _plate(centre, u, v, half_l, half_w) -> list:
        """Rectangle around `centre` spanned by the screen vectors u (long) and v (wide)."""
        corners = []
        for su, sv in ((1, 1), (1, -1), (-1, -1), (-1, 1)):
            corners.append((centre[0] + u[0] * half_l * su + v[0] * half_w * sv,
                          centre[1] + u[1] * half_l * su + v[1] * half_w * sv))
        return corners

    def _hud(self) -> None:
        """Header line plus one line per robot; two columns from 5 robots up."""
        eng = self.engine
        self._text(f"{eng.world.name}  t={eng.t:6.1f}s  {self.clock.get_fps():4.0f} fps  "
                   f"{self.s:3.0f} px/m  robots: {len(eng.robots)}  task: {eng.task or '-'} "
                   f"{keys.help_line(self.teleop)}",
                   8, 6, (235, 235, 240), big=True)
        width = self.size[0] // 2 if len(eng.robots) > 4 else self.size[0]
        for i, r in enumerate(eng.robots.values()):
            o = (f"x={r.odom.x:+.2f} y={r.odom.y:+.2f} th={math.degrees(r.odom.theta):+.0f}"
                 if r.odom else "no odometry")
            x = 8 + (i // 10) * width
            y = 30 + (i % 10) * 17
            pygame.draw.rect(self.screen, rgb(r.spec.rgb), (x, y + 3, 9, 9))
            for label in [(r.spec.name, rgb(r.spec.rgb)), (r.spec.variant, GREY),
                          (r.mode, GREY),
                          (f"|v|={math.hypot(r.twist.vx, r.twist.vy):.2f} m/s", GREY),
                          (f"w={r.twist.omega:+.2f}", GREY), ("odom " + o, GREY),
                          *overlays.sensor_readout(self, r),
                          *overlays.command_readout(self, r),
                          *overlays.link_readout(self, r),
                          *overlays.poi_readout(self, r),
                          *overlays.steer_readout(self, r),
                          ("kf " + (f"x={r.kf.x:+.2f} y={r.kf.y:+.2f} "
                                    f"σ=({r.kf.sx:.2f},{r.kf.sy:.2f}) "
                                    f"Δ={r.kf_err:.2f} m" if r.kf else "-"), KF_COLOR),
                          *overlays.sigma_legend(self, r),
                          (f"dist={r.distance:.1f}m", GREY), (f"contacts={r.contacts}", GREY),
                          (r.mission_state, GREY)]:
                x += self._text(label[0], x + 13, y, label[1]) + 8

    def _geom(self, robot) -> list:
        """lx, ly, wheel radius, footprint — from the physics, otherwise from cfg.

        From the chassis and not from `config/default.json`: a `slow` or `steering-big` robot is
        then drawn at the size it collides at, and the picture cannot contradict the grader.
        """
        g = getattr(robot.chassis, "geom", None)
        return [getattr(g, key, cfg_get(self.cfg, "robot." + key, dflt))
                for key, dflt in (("lx", .14), ("ly", .13), ("r", .05), ("footprint_r", .21))]

    # --------------------------------------------------------------------------------- Eingaben

    def poll(self) -> dict:
        """Key presses and mouse since the last call; state flips here, the edge is reported."""
        flags = {"quit": False, "pause": False, "toggle_lidar": False,
                 "toggle_trail": False, "camera": None, "key": "", "menu": "",
                 "teleport": None, "resized": None}
        for ev in pygame.event.get():
            handler = self._events.get(ev.type)
            if handler:
                handler(ev, flags)
        return flags

    def _ev_quit(self, ev, flags) -> None:
        self._alive, flags["quit"] = False, True

    def _ev_resize(self, ev, flags) -> None:
        self.size = (ev.w, ev.h)
        self.screen = pygame.display.set_mode(self.size, pygame.RESIZABLE)
        self.cam.resize(self.size)
        flags["resized"] = self.size

    def _ev_mouse_down(self, ev, flags) -> None:
        """One click, one meaning — and the robot menu has the first claim on all three buttons.

        Left on one of its rows: that robot goes to the spot the menu was opened for. Left anywhere else
        on the panel: it closes. Only then come the view panel, the drag and the camera. The right button
        opens the menu over open floor and does nothing over a rack or the void — a menu that can only
        answer "no" is not an offer; `world.free()` says what is not open floor, the same rule the
        engine insists on when it is handed the spot.
        """
        chosen = self.pick.handle(ev)
        if chosen and self._pick_spot:
            flags["teleport"] = (chosen, self._pick_spot[0], self._pick_spot[1])
            self.pick.open = False
            self._pick_spot = None
            return
        if self.pick.inside(ev.pos):
            self.pick.open, self._pick_spot = False, None
        attribut = self.menu.handle(ev)
        if attribut:
            setattr(self, attribut, not getattr(self, attribut))
            flags["menu"] = attribut
        elif ev.button == 1 and not self.menu.inside(ev.pos) and not self.pick.inside(ev.pos):
            self._drag = ev.pos
        elif ev.button == 3:
            self._open_picker(ev.pos)
        elif ev.button == 2:
            self.cam.center()

    def _open_picker(self, pos) -> None:
        """Right button: offer every robot that drives for the spot under the pointer.

        The spot is saved in **world metres when the menu opens**, not read again when a row is clicked:
        the pointer may have wandered a pixel in between, and a placement that means the floor under the
        pointer at click time is a placement at two chances to miss. Nothing moves the camera while the
        menu is open (`inside()` blocks the drag), so the two are the same point anyway — one of them is
        simply the one that is remembered.

        A second right-click replaces the offer, which is how a menu is moved: no close-click needed.
        """
        names = sorted(getattr(self.engine, "robots", {}) or {})
        spot = self.cam.wx(*pos)
        if not names or not self._open_floor(spot):
            self.pick.open, self._pick_spot = False, None      # nothing to place, or nowhere to place it
            return
        self.pick.set_rows([(name, name, "") for name in names])
        self.pick.chips = {name: self._robot_chip(name) for name in names}
        self._pick_spot, self._pick_where, self.pick.open = spot, pos, True

    def _open_floor(self, spot) -> bool:
        """Is that a place a robot can stand on? `World.free()` decides, this only asks."""
        try:
            self.engine.world.free("robot", spot[0], spot[1])
            return True
        except (ValueError, AttributeError):
            return False

    def _robot_chip(self, name: str) -> tuple:
        """The colour a robot is drawn in, for its row in the menu — 0..255, the way `mix()` wants it."""
        robot = (getattr(self.engine, "robots", {}) or {}).get(name)
        spec = getattr(robot, "spec", None)
        return to255(getattr(spec, "rgb", None) or self.col_wall)

    def forget(self, name: str) -> None:
        """Drop what the window remembers of one robot's past — for whoever moved it somewhere else.

        A teleport is not a drive: the four histories keyed by name (truth trail, believed trail,
        estimate trail, wheel phase) still hold the old part of the hall, and drawing a line from there
        to here would show a journey that never happened. Called by the run loop after a placement, not
        by the click itself — the window only offers a spot, it does not move anything.
        """
        for history in (self.trails, self.ghost_trail, self.kf_trail, self.phase):
            history.pop(name, None)

    def _ev_mouse_motion(self, ev, flags) -> None:
        self.pointer = ev.pos                     # where the pointer is, for the coordinate readout
        self.menu.handle(ev)
        self.pick.handle(ev)                      # the hot row of the robot menu, if it is open
        if self._drag and not self.menu.inside(ev.pos) and not self.pick.inside(ev.pos):
            self.cam.pan(ev.pos[0] - self._drag[0], ev.pos[1] - self._drag[1])
            self._drag = ev.pos

    def _ev_mouse_up(self, ev, flags) -> None:
        self._drag = None

    def _ev_wheel(self, ev, flags) -> None:
        if ev.y:
            self.cam.zoom_to_at(pygame.mouse.get_pos(), 1.25 if ev.y > 0 else 0.8)

    def _event_key(self, ev, flags) -> None:
        """One key press, one meaning — `keys.py` says which, this only executes it.

        The teleop keys (w a s d, the arrows, q/e as turners) are *polled* by the run loop while
        they are held, so they never arrive here as a switch: `check_bindings()` in `keys.py` is
        what guarantees that no letter in `KEYS` is also in the driving block. `q` is the single
        key with two meanings and they are the two it always had — it turns while a key can drive,
        and ends the run when none can.
        """
        key = pygame.key.name(ev.key)
        if key == keys.QUIT and self.pick.open:
            # `esc` over an open robot menu closes the menu instead of ending the run — the hand that
            # reaches for esc there has just decided not to place anything, and losing the window to it
            # would be the one reply it did not mean to give.
            self.pick.open, self._pick_spot = False, None
            return
        flags["key"] = key
        if key in KEYS:
            attribut, flank = KEYS[key]
            setattr(self, attribut, not getattr(self, attribut))
            if flank:
                flags[flank] = True
        elif key == keys.QUIT or (key == "q" and not self.teleop):   # in teleop q turns
            self._alive, flags["quit"] = False, True
        elif key == keys.MENU:
            flags["menu"] = "menu"
            self.menu.toggle()
        elif key == keys.FIT:
            self.cam.center()
        elif key == keys.ALL_ROBOTS:
            self.focus, flags["camera"] = None, 0
        elif key in "123456789":
            self.focus, flags["camera"] = int(key) - 1, int(key) - 1
        elif key in keys.ZOOM_IN:
            self.cam.zoom_to(1.25)
        elif key in keys.ZOOM_OUT:
            self.cam.zoom_to(1 / 1.25)

    @property
    def ok(self) -> bool:
        return bool(self._alive and not self._closed and pygame.display.get_init())

    def close(self) -> None:
        if not self._closed:                                      # pygame.quit only once
            self._closed, self._alive = True, False
            pygame.quit()
