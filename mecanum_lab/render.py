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
from .types import cfg_get

log = logging.getLogger("mecanum.render")

GREY = (210, 212, 218)
GPS_COLOR = (250, 210, 90)                         # measurement: flat and angular
KF_COLOR = (120, 240, 170)                         # estimate: bright and round
KF_SICHERHEIT = 2.0                                # how many σ the GUI ellipse shows
CORNERS = ((1, 1), (1, -1), (-1, 1), (-1, -1))     # FL, FR, RL, RR in the body frame
# Roller axis of each wheel, body frame, same order as `CORNERS` (= `physics.WHEELS`). These four
# axes are the **X arrangement** that `physics.inverse_kinematics()` implements: a wheel driven
# along (1,-1) carries its rollers along (1,1), so FL and RR sit on one diagonal and FR and RL on
# the other. The other diagonal is the O arrangement — the same robot with the rollers mirrored,
# whose wheels would strafe the wrong way for every command in the readout.
ROLLERS = ((1, 1), (1, -1), (1, -1), (1, 1))
WHEEL_STROKES = 3                                     # strokes drawn per wheel
WHEEL_SPIN_GAIN = 0.25                              # wheel radius -> on-screen rotation, illustrative
WHEEL_WIDTH_RATIO = 0.6          # drawn width of a wheel, as a fraction of its drawn length
WHEEL_MAX_PART_OF_PLATE = 0.42   # the exaggerated radius never eats the chassis plate
WHEEL_MIN_PX = 3.0               # below this a wheel is one pixel, not a wheel
# Marker per robot as a list of (radius, angle in degrees); radius 1 = footprint.
SHAPES = {
    "triangle": [(1, -90), (1, 30), (1, 150)],
    "square": [(1.2, a) for a in (45, 135, 225, 315)],
    "diamond": [(1.25, a) for a in (0, 90, 180, 270)],
    "circle": [(1, a) for a in range(0, 360, 30)],
    "pentagon": [(1.15, a) for a in range(90, 450, 72)],
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


def rgb(color, factor: float = 1.0) -> tuple:
    """Color from types.PALETTE (0..1) -> pygame (0..255), optionally darkened."""
    return tuple(min(255, int(255 * v * factor)) for v in color)


def mix(a: tuple, b: tuple, part: float = .5) -> tuple:
    """Mix two colors; part 0..1 in favor of b.

    Accepts both colour spaces used in this file — `types.PALETTE` values (0..1) and colors
    already converted by `rgb()` (0..255). Without that, `mix(rgb(spec.rgb), ...)` multiplied by
    255 a second time and drew everything near white, which is how a red robot got a white box.
    """
    value = lambda w: [255 * v if v <= 1 else v for v in w]            # noqa: E731 - reads inline
    a, b = value(a), value(b)
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
    """Corner points of a marker shape in pixels, unknown names drawn as a circle.

    The minus before theta compensates the downward screen y, so the shape turns with the
    robot instead of standing mirrored.
    """
    return [(centre[0] + radius * rt * math.cos(math.radians(ang) - theta),
             centre[1] + radius * rt * math.sin(math.radians(ang) - theta))
            for rt, ang in SHAPES.get(name) or SHAPES["circle"]]


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
VIEW_PROFILES = {"clean": tuple(name for name in LAYER_NAMES if name not in RAW_LAYERS),
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
        self.col_wall = rgb(style.get("wall", (.34, .36, .42)))
        self.col_floor = rgb(style.get("floor", (.13, .14, .17)))
        self.col_void = rgb(style.get("void", (.06, .065, .08)))     # outside the world
        self.trail_len = int(style.get("trail_len", 400))
        self.wheel_scale = float(style.get("wheel_scale", 2.4))        # wheels: drawn bigger than 5 cm
        self.chassis_scale = float(style.get("chassis_scale", 1.0))  # body box, in addition to lx/ly
        # Every layer starts as the configuration says (`view_state` above), not as a hard-coded
        # True: what a student sees on first start is then a setting with a name, and a demo config
        # can ask for the layer it is about. Keys and clicks still switch anything, at any time.
        for attribute, drawn in view_state(cfg).items():
            setattr(self, attribute, drawn)
        self.teleop = False                          # node.run_loop sets this, changes the help
        self.paused = False
        self.kf_trail, self.trails, self.phase = {}, {}, {}           # per robot name
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

    @property
    def oy(self) -> float:
        return self.size[1] / 2 + self.cam.cy * self.s

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

    def _text(self, txt, x, y, color, big=False, center=False) -> int:
        img = (self.big if big else self.font).render(txt, True, color)
        self.screen.blit(img, img.get_rect(center=(x, y)) if center else (x, y))
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
        self.phase = {k: v for k, v in self.phase.items() if k in eng.robots}
        for robot in eng.robots.values():
            overlays.skid_marks(self, robot, dt)              # rubber under slipping wheels
            self._trail(robot)                              # kept while hidden, not thrown away
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
        if self.paused:
            self._text("PAUSE", self.size[0] / 2, 24, (250, 220, 120), True, True)
        pygame.display.flip()
        if cap:
            self.clock.tick(int(cfg_get(self.cfg, "gui_rate", 30) or 30))

    def _world(self) -> None:
        """Floor inside the world box, void outside, walls as filled blocks, goal."""
        w, sc = self.engine.world, self.screen
        feld = pygame.Rect(self.cam.rect)
        pygame.draw.rect(sc, self.col_floor, feld)
        for x0, y0, x1, y1 in (w.markings or []) if self.show_markers else []:
            pygame.draw.line(sc, mix(self.col_floor, (1, 1, 1), .4), self.px(x0, y0),
                             self.px(x1, y1), max(1, int(0.06 * self.s)))
        for wall in w.walls or []:
            # floor/ceil instead of int: neighbouring blocks then touch each other instead of
            # leaving a 1 px floor seam between them — that seam would read as a thin wall again.
            top, bottom = self.px(wall.x0, wall.y1), self.px(wall.x1, wall.y0)
            corner = (math.floor(top[0]), math.floor(top[1]))
            pygame.draw.rect(sc, self.col_wall, pygame.Rect(
                corner, (max(1, math.ceil(bottom[0]) - corner[0]),
                       max(1, math.ceil(bottom[1]) - corner[1]))))
        pygame.draw.rect(sc, mix(self.col_wall, (1, 1, 1), .3), feld, 2)   # the world ends here
        if w.goal and self.show_goal:
            centre = self.px(w.goal.x, w.goal.y)
            for ring, radius in enumerate((16, 10, 4)):                       # bullseye
                pygame.draw.circle(sc, mix((1, .85, .3), (1, 1, 1), ring / 3), centre, radius,
                                   2 if ring else 0)
            self._text("goal", centre[0] - 12, centre[1] - 32, (1, .85, .3))

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
        center, radius = self.px(kf.x, kf.y), max(4, KF_SICHERHEIT * self.s * max(kf.sx, 1e-4))
        height = max(4, KF_SICHERHEIT * self.s * max(kf.sy, 1e-4))
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

    def _robot(self, robot, dt: float) -> None:
        """Chassis, marker, heading, four wheels with rolling strokes, name label.

        Two unit systems meet here and the line between them is drawn on purpose: everything that
        says **where** something is on the floor is in metres and goes through `self.px()`,
        everything that says **how big** a thing is drawn is in pixels. Both are named `half_l`
        and `be` in the version before this one, and the reassignment in the middle of these
        lines put a pixel number into a metre slot: the four wheels came out 10 m in front of and
        behind the robot — off every screen, which is how the wheel picture came to be "broken".
        """
        pose, col = robot.pose, rgb(robot.spec.rgb)
        sc = self.screen
        lx, ly, wr, fp_m = self._geom(robot)
        centre, fp = self.px(pose.x, pose.y), max(6, fp_m * self.s)
        plate = ((lx + wr) * self.chassis_scale, (ly + wr) * self.chassis_scale)   # metres
        # The one exaggeration of this view: `wheel_scale` times the real 5 cm, never more than
        # 42 % of the plate, so the rollers stay readable without the wheel eating the chassis.
        wheel_r = min(wr * self.wheel_scale, WHEEL_MAX_PART_OF_PLATE * plate[0])   # metres
        u, v = body(-pose.theta, 1, 0), body(-pose.theta, 0, 1)                    # screen vectors
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
        pygame.draw.line(sc, col, centre,
                         self.px(*_add((pose.x, pose.y), body(pose.theta, fp_m * 1.25, 0))),
                         max(2, int(self.s * .03)))
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
        u, v = body(-heading, 1, 0), body(-heading, 0, 1)       # this wheel's axes on screen
        tyre = self._plate(centre, u, v, half_len, half_wid)
        pygame.draw.polygon(sc, mix(col, (0, 0, 0), .55), tyre)
        if self.s > 45:                                           # a 1 px edge below that is mush
            pygame.draw.polygon(sc, col, tyre, 1)
        roller = body(-heading, ROLLERS[index][0], ROLLERS[index][1])
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
        """lx, ly, wheel radius, footprint — from the physics, otherwise from cfg."""
        g = getattr(robot.chassis, "geom", None)
        return [getattr(g, key, cfg_get(self.cfg, "robot." + key, dflt))
                for key, dflt in (("lx", .14), ("ly", .13), ("r", .05), ("footprint_r", .21))]

    # --------------------------------------------------------------------------------- Eingaben

    def poll(self) -> dict:
        """Key presses and mouse since the last call; state flips here, the edge is reported."""
        flags = {"quit": False, "pause": False, "toggle_lidar": False,
                 "toggle_trail": False, "camera": None, "key": "", "menu": "",
                 "resized": None}
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
        attribut = self.menu.handle(ev)
        if attribut:
            setattr(self, attribut, not getattr(self, attribut))
            flags["menu"] = attribut
        elif ev.button == 1 and not self.menu.inside(ev.pos):
            self._drag = ev.pos
        elif ev.button in (2, 3):
            self.cam.center()

    def _ev_mouse_motion(self, ev, flags) -> None:
        self.menu.handle(ev)
        if self._drag and not self.menu.inside(ev.pos):
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
