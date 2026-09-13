"""Extra overlays for the window: GPS shadow zones, the odometry ghost, rubber from wheel slip.

Effects that make the invisible visible, none of which changes physics or a topic:

* `zones()` draws the rectangles of `gps.zones` (see sensors.GpsSensor) as hatched shadow, so a
  student can see *where* the fix goes bad before the logbook proves it. Blackout zones are drawn
  in the error colour.
* `odom_ghost()` draws where the odometry believes the robot is, next to the truth, with the
  distance between them in metres. Drift is otherwise a number in the readout nobody reads.
* `skid_marks()` leaves rubber on the floor while the wheels turn against an obstacle — the
  visible twin of the phantom distance that `robot.slip` produces.
* `sensor_readout()` builds the gps/lidar/imu parts of the per-robot readout line, quality and
  satellite count and chip temperature included — the sensor's own opinion, not its number.
* `steer_readout()` is the same for the second drive train: the two front wheel angles and the
  turning radius that follows from them. A steered car is drawn with its wheels turned by
  `render._wheel()`; what belongs here is the number next to the picture.
* `poi_sources()` draws the Points of Interest of the world (pois.py) and, with `debug_truth`, the
  rings of their field; `poi_readout()` puts the counter's current reading into the readout line.
* `command_readout()` says who last commanded a robot — a node's topic or the window's keys, they
  share `/cmd_vel`, and while both are running that is the only question with a straight answer.
* `sigma_legend()` says what the ellipse around the estimate is made of — the half-axis in metres and
  the same half-axis in pixels at the current zoom, because a green oval explains nothing by itself.
* `network()` draws the radio: the access point, the line from it to every robot fading with the link
  quality, and a quality bar per robot. `autonomy_mark()` is the amber outline of a robot the link
  gave up on, and `link_readout()` the same numbers as text.

Stdlib + pygame only, no state in the module (marks live on the renderer), stdlib drawing calls
only (CONTRACT section 1). Text uses the renderer's own blit so both use the same fonts.
"""
import math

import pygame

from .types import cfg_get

TRAIL_LIFE = 3.0                # seconds a skid mark stays on the floor
MARK_MAX = 600                  # bound for a long teleop session
GOOD_LINK = (130, 225, 150)     # q = 1: the line to the access point is a line
DEAD_LINK = (240, 120, 120)     # q = 0: the dashed stub that is left of it
LINK_AMBER = (250, 205, 90)     # the colour of a robot the radio left alone
BAR_W, BAR_H = 52, 9            # the quality bar beside a robot, in pixels


def running_zones(rend) -> list:
    """The zones of the running configuration.

    Read from the live config dict, not from a snapshot: `set_task` merges a task profile into
    that same dictionary, so what is drawn is what the GPS is about to measure. Three rectangles
    are parsed per frame — that is cheaper than a cache that could go stale.
    """
    from .sensors import _zones                                          # lazy, see module note
    gps = (getattr(rend, "cfg", None) or {}).get("gps") or {}
    return _zones(gps.get("zones"))


def zones(rend) -> None:
    """Hatched shadow where the fix degrades, red hatch where there is no fix at all."""
    from .render import GREY, mix                             # lazy: render imports this module
    sc, s = rend.screen, rend.s
    fur_schatten = mix(rend.col_floor, (250, 210, 90), .11)     # hint, not a second floor
    fur_dunkel = mix(rend.col_floor, (240, 120, 120), .26)
    # The label must not wear the colour of its own hatch. `fur_schatten` is 11 % amber on the
    # floor, so the name of the zone was 1.3:1 against the floor it sits on — the fill is meant to
    # recede behind the map, the word "multipath" is meant to be read. Same two hues, mixed away
    # from the floor instead of towards it; bright enough over floor, wall and hatch alike.
    schrift_schatten = mix(GREY, (250, 210, 90), .55)
    schrift_dunkel = mix(GREY, (240, 120, 120), .55)
    for zone in running_zones(rend):
        x0, y1 = rend.px(*zone["rect"][:2])                   # world metres -> pixels
        x1, y0 = rend.px(*zone["rect"][2:])
        corner = (min(x0, x1), min(y0, y1))
        half_l, be = abs(x1 - x0), abs(y0 - y1)                   # width and height on screen
        if half_l < 2 or be < 2:
            continue
        color = fur_dunkel if zone["block"] else fur_schatten
        schrift = schrift_dunkel if zone["block"] else schrift_schatten
        schritt = 26 if zone["block"] else 46                 # blackout reads denser than shadow
        for cross_offset in range(-int(be), int(half_l), schritt):  # 45° hatch: shadow over there
            started, end = max(0.0, -cross_offset), min(be, half_l - cross_offset)
            if end > started:
                pygame.draw.line(sc, color,
                                 (corner[0] + cross_offset + started, corner[1] + started),
                                 (corner[0] + cross_offset + end, corner[1] + end), 1)
        _dashed(sc, corner, (half_l, be), mix(color, (255, 255, 255), .45))
        if half_l > 90 * max(s / 40, 0.25):
            rend._text(zone_label(zone), max(6, corner[0] + 6),
                       above_bar(rend, corner[1] + 14), schrift)


def zone_label(zone: dict) -> str:
    """What the shadow is worth, straight out of the config: `shelf  σ×6, bias +0.8/-0.5 m`."""
    if zone["block"]:
        return zone["name"] + "  no fix"
    extra = f", bias {zone['bias'][0]:+.1f}/{zone['bias'][1]:+.1f} m" if any(zone["bias"]) else ""
    return f"{zone['name']}  σ×{zone['sigma_scale']:g}{extra}"


def odom_ghost(rend, robot) -> None:
    """Ghost chassis at the odometry pose, leader line to the truth, gap in metres."""
    from .render import mix, rgb, screen
    o = getattr(robot, "odom", None)
    if not o:
        return
    color, sc = rgb(robot.spec.rgb), rend.screen
    truth = rend.px(robot.pose.x, robot.pose.y)
    ghost = rend.px(o.x, o.y)
    gap = math.dist((robot.pose.x, robot.pose.y), (o.x, o.y))
    fill = mix(color, rend.col_floor, .74)
    edge = mix(color, rend.col_floor, .40)
    g = getattr(robot.chassis, "geom", None)
    half_l = (float(getattr(g, "lx", .14)) + float(getattr(g, "r", .05))) * rend.chassis_scale
    be = (float(getattr(g, "ly", .13)) + float(getattr(g, "r", .05))) * rend.chassis_scale
    u, v = screen(o.theta, 1, 0), screen(o.theta, 0, 1)      # the ghost is drawn like the real thing
    pygame.draw.polygon(sc, fill, _plate(ghost, u, v, half_l * rend.s, be * rend.s))
    pygame.draw.polygon(sc, edge, _plate(ghost, u, v, half_l * rend.s, be * rend.s), 1)
    if math.dist(truth, ghost) > 3:
        pygame.draw.line(sc, edge, ghost, truth, 1)
        rend._text(f"odom off {gap:.2f} m", (ghost[0] + truth[0]) / 2 + 8,
                   above_bar(rend, (ghost[1] + truth[1]) / 2 - 10), edge)


def skid_marks(rend, robot, dt: float) -> None:
    """Rubber on the floor while the wheels turn and the body cannot — `robot.slip` made visible."""
    from .render import body, mix, screen        # body() for the world offset, screen() for the stroke
    trail = rend.__dict__.setdefault("skid_trail", [])          # state on the renderer, not here
    g = getattr(robot.chassis, "geom", None)
    rder = getattr(robot, "wheels", None) or [0.0]
    if getattr(robot.chassis, "touching", False) and max(abs(w) for w in rder) > 0.5:
        half_l, be = float(getattr(g, "lx", .14)), float(getattr(g, "ly", .13))
        for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):   # one mark per wheel corner
            offset = body(robot.pose.theta, sx * half_l, sy * be)
            trail.append((robot.pose.x + offset[0], robot.pose.y + offset[1],
                         robot.pose.theta, 1.0))
        del trail[:-MARK_MAX]
    sc, bg = rend.screen, mix(rend.col_floor, (0, 0, 0), .55)
    alive = []
    for x, y, theta, life in trail:
        life -= dt / TRAIL_LIFE
        if life <= 0:
            continue
        alive.append((x, y, theta, life))
        center = rend.px(x, y)
        stroke = screen(theta, 5 * life, 0)      # the mark lies along the driven direction
        pygame.draw.line(sc, mix(bg, rend.col_floor, 1 - life),
                         (center[0] - stroke[0], center[1] - stroke[1]),
                         (center[0] + stroke[0], center[1] + stroke[1]), 2)
    rend.skid_trail[:] = alive


def sensor_readout(rend, robot) -> list:
    """The gps, lidar and imu segments of one readout line — with what each says about itself.

    Four numbers tell a student *why* a reading is what it is: the GPS quality, how many anchors it
    came from, the IMU's chip temperature, and how many messages never arrived. Three of them are
    fields of the messages (`Gps.quality`, `Gps.sats`, `Scan.missing`), the temperature is not —
    `sensor_msgs/Imu` has no such field — so it comes from the sensor here. Showing them in the line
    that is already read is what saves the second terminal with `ros2 topic echo` on it.

    Returns `[(text, color), …]` for `render._hud()` to append; drawing stays in render.py.
    """
    from .render import GREY, mix                        # lazy: render imports this module
    fix, rays, inertial = robot.gps, robot.scan, robot.imu
    quality, sats, lost = _gps_view(rend, robot)
    gps = "gps " + (f"x={fix.x:+.2f} y={fix.y:+.2f}" if fix else "no fix")
    gps += f" q{quality} {sats} sats" + (f" lost {lost}" if lost else "")
    amber = mix(GREY, (250, 205, 90), 0.55 if quality == 1 else 0.9)      # q1 warm, q0 loud
    out = [(gps, GREY if quality == 2 else amber)]
    if rays is not None and rays.missing:
        out.append((f"lidar {rays.missing} beams no echo", amber))
    out.append(("imu " + (f"ax={inertial.ax:+.2f} ay={inertial.ay:+.2f} gz={inertial.gz:+.3f} "
                          f"{inertial.temp:.1f} °C" if inertial else "no imu"), GREY))
    return out


def _gps_view(rend, robot) -> tuple:
    """(quality, anchors, messages lost) — asked of the receiver, not of the last message.

    While the robot sits in a blackout there is no fresh message to read, and the reason has to be
    visible right then. A renderer without a live engine (a test with a hand-built robot) falls
    back to what the last message said.
    """
    eng = getattr(rend, "engine", None)
    probe = getattr(eng, "gps_health", None)
    if probe is not None and robot.spec.name in getattr(eng, "robots", {}):
        return probe(robot.spec.name)
    return (robot.gps.quality, robot.gps.sats, 0) if robot.gps else (0, 0, 0)


def steer_readout(rend, robot) -> list:
    """The steered front axle in the readout line: both angles and the radius they make.

    The two angles are printed in the order of the wheel labels (`VL` then `VR`, CONTRACT §3) and not
    as "inner/outer": on a left turn the inner wheel is VL and turns further, on a right turn it is
    VR, and a readout that swaps its own labels with the steering direction explains nothing.
    Next to them the radius, because `R = wheel_base / tan(delta)` is what the driver has to slow
    down for. A mecanum robot returns an empty list, so the readout line of every graded run stays
    exactly as long as it was.

    Like `sensor_readout()` this builds text and draws nothing — placing it is `render._hud()`.
    """
    angles = getattr(robot.chassis, "steer", None)
    if not angles:
        return []
    from .render import GREY                                # lazy: render imports this module
    base = float(getattr(getattr(robot.chassis, "geom", None), "wheel_base", 0.0) or 0.0)
    delta = (angles[0] + angles[1]) / 2.0
    radius = f"  R={base / math.tan(delta):.2f} m" if base and abs(delta) > 1e-3 else ""
    return [(f"steer {math.degrees(angles[0]):+.1f}/{math.degrees(angles[1]):+.1f} deg{radius}",
             GREY)]


def poi_list(rend) -> list:
    """The sources of the running world, from the engine that validated them.

    Asked of the engine and not read out of `rend.cfg["pois"]`: the numbers the window draws have to
    be the ones the sensor measured, and one of them failed the wall test before it got here. A
    renderer built without an engine (a view test with a hand-made robot) gets an empty list.
    """
    eng = getattr(rend, "engine", None)
    probe = getattr(eng, "poi_sources", None)
    return list(probe()) if probe is not None else []


def poi_sources(rend) -> None:
    """The radiation source as a symbol; with `debug_truth`, the rings its field draws around it.

    The symbol is drawn even when the robot measures 0.0, and that is the whole lesson of the layer:
    a source is not an obstacle. The LIDAR in front of it reports the table, the counter reports the
    source, and the two do not agree because one of them needs a straight line and the other does not
    (pois.py). So the layer belongs next to the GPS shadow, not next to the walls.

    Without `debug_truth` only the name goes with the symbol — what the message already says. With
    it, the distances a reading can be traced back to are drawn as circles: half the activity at `d0`,
    a tenth at `3·d0` where that is still inside the range, and the `range` itself beyond which the
    counter stays at zero. Those circles are the tutor's view of the field, not another sensor.
    """
    from .render import mix                                   # lazy: render imports this module
    sc, s = rend.screen, rend.s
    truth = bool((getattr(rend, "cfg", None) or {}).get("debug_truth"))
    bright = mix(rend.col_floor, (250, 205, 90), 0.8)
    pale = mix(rend.col_floor, (250, 205, 90), 0.35)
    for src in poi_list(rend):
        centre = rend.px(src.x, src.y)
        if truth:
            rings = [(src.range_m, pale), (src.d0, bright)]
            if 3 * src.d0 <= src.range_m:      # the tenth ring only exists where there is a field
                rings.append((3 * src.d0, bright))
            for metres, color in rings:
                if int(metres * s) >= 3:                      # a 2 px circle is a dot, not a ring
                    pygame.draw.circle(sc, color, centre, int(metres * s), 1)
        pygame.draw.circle(sc, bright, centre, 5)
        for spoke in range(6):                                # the burst: a source, not a waypoint
            angle = math.tau * spoke / 6
            ux, uy = math.cos(angle), math.sin(angle)
            pygame.draw.line(sc, bright, (centre[0] + 7 * ux, centre[1] - 7 * uy),
                             (centre[0] + 13 * ux, centre[1] - 13 * uy), 2)
        if s > 26:
            rend._text(f"{src.name}" + (f"  a={src.activity:g} r={src.range_m:g} m d0={src.d0:g} m"
                                         if truth else ""),
                       centre[0] + 17, above_bar(rend, centre[1] - 20), bright)


def poi_readout(rend, robot) -> list:
    """The counter in the readout line: `poi src1 0.803` — the sensor without a second terminal.

    One number, and it is the one the students have to work from. `/poi` carries a stamp and that
    number and nothing else (§6.13), so the name of the loudest source is asked of the engine —
    `engine.loudest_poi()`, the tutor's view, the same border as the field rings of `poi_field()`.
    The metres stay out of the line even here: a readout that prints the distance ends the exercise.
    An empty list for a robot with no detector keeps the readout line of every graded run exactly as
    long as it was, in the same rule as `steer_readout()`.
    """
    msg = getattr(robot, "poi", None)
    if msg is None:
        return []
    from .render import GREY, mix                       # lazy: render imports this module
    # `spec.name`, not `name`: a `Robot` has no `name` of its own, and the double in the test that
    # checked this line had one — so the readout crashed the whole simulator on the first robot that
    # heard a source, which is the moment the window is worth having.
    source = (getattr(rend.engine, "loudest_poi", lambda _n: None)(robot.spec.name)
              if msg.intensity > 0.0 else None)
    return [(f"poi {getattr(source, 'name', '') or '-'} {msg.intensity:.3f}",
             mix(GREY, (250, 205, 90), 0.7) if source else GREY)]


def above_bar(rend, y: float) -> float:
    """Keep a label out of the readout block: one 17 px line per robot, plus the header."""
    return max(y, 48 + 17 * len(getattr(rend.engine, "robots", {}) or {}))


def sigma_legend(rend, robot) -> list:
    """What the ellipse around the estimate is worth: `ellipse 2σ: half-axis 0.34 × 0.21 m = 18 × 11 px`.

    The oval is `render._estimate()` and its half-axis is `KF_SIGMA · σ · px_per_metre` — three
    numbers from three places, and none of them is on screen. So a student looking at a green ellipse
    cannot tell the estimate's own uncertainty apart from the distance to the truth, which is the one
    thing task K3 punishes. The line names the same oval in the two units a reader has: metres on the
    floor and pixels at the current zoom.

    It goes into the readout line and not into the estimate layer on purpose: the σ of the estimate is
    a fact about the numbers that are printed there anyway, and a fact should not disappear because a
    drawing was switched off with `k`. (What the drawing does not show is the 4 px floor of the drawn
    radius: under it the oval on screen is larger than the metres it stands for.)
    """
    kf = getattr(robot, "kf", None)
    if kf is None:
        return []
    from .render import GREY, KF_SIGMA              # lazy: render imports this module
    metres = (KF_SIGMA * kf.sx, KF_SIGMA * kf.sy)
    pixels = [round(m * rend.s) for m in metres]
    return [(f"ellipse {KF_SIGMA:g}σ: half-axis {metres[0]:.2f} × {metres[1]:.2f} m"
             f" = {pixels[0]} × {pixels[1]} px at {rend.s:.0f} px/m", GREY)]


# ---------------------------------------------------------------------------- the radio link


def radio_health(rend, robot):
    """The radio's own answer about one robot, or None: a lab without `wifi.enabled` has no antenna.

    Asked of the engine and not of the last `/link` message, in the rule of `_gps_view()` — the moment
    worth seeing is the one where nothing is delivered and so nothing is stamped. A renderer without a
    live engine (a view test with a hand-built robot) gets None, and the layer stays empty.
    """
    eng = getattr(rend, "engine", None)
    probe = getattr(eng, "link_health", None)
    return probe(robot.spec.name) if probe is not None else None


def access_point(rend):
    """Where the hall's access point hangs — from the same answer the bars are drawn from."""
    for robot in (getattr(rend.engine, "robots", {}) or {}).values():
        view = radio_health(rend, robot)
        if view is not None:
            return view[8]
    return None


def link_text(view: tuple, colourless: bool = False) -> str:
    """`wifi q 0.42 -72.3 dBm 1 wall lost 4/120 61 ms` — the bar's numbers as one readable line.

    All of it terms a student can look up: the quality the model is written in, the level it came
    from, how many rectangles that line crosses (the reason the level is what it is), the frames lost
    of the frames ever asked for, and what the wire costs right now. With the link down the line
    starts with DOWN, because "q 0.00" and "nothing will arrive" are two different sentences.
    """
    q, dbm, _metres, walls, up, dropped, sent, latency, _ap = view
    state = "" if up else "DOWN "
    return (f"wifi {state}q {q:.2f} {dbm:.1f} dBm"
            + (f" {walls} wall{'s' if walls != 1 else ''}" if walls else " free")
            + (f" lost {dropped}/{sent}" if sent else "")
            + (f" {latency:.0f} ms" if latency and not colourless else ""))


def link_readout(rend, robot) -> list:
    """The link in the readout line, so the radio is observable without a second terminal.

    An empty list for a run without a radio keeps the readout line of every graded run exactly as long
    as it was — the rule of `steer_readout()` and `poi_readout()`.
    """
    from .render import GREY, mix                       # lazy: render imports this module
    view = radio_health(rend, robot)
    if view is None:
        return []
    colour = DEAD_LINK if not view[4] else mix(GREY, GOOD_LINK, view[0])
    return [(link_text(view), colour)]


def command_readout(rend, robot) -> list:
    """`cmd topic 0.04 s` / `cmd keys 0.02 s` / `cmd none` — who last commanded this robot.

    A student node and the window's teleop keys write to the *same* `/cmd_vel` (the keys publish only
    while one is held, a node publishes continuously), so the freshest timestamp is the only honest
    answer to "what is driving this one" — and after `cmd_timeout` without a frame, nothing is. With
    `wifi.enabled` this segment can honestly say `topic` while the radio refuses every frame; that is
    what the `wifi` segment next to it is for, and why both are in the line.
    """
    from .render import GREY, mix                        # lazy: render imports this module
    timeout = float(cfg_get(getattr(rend, "cfg", None), "cmd_timeout", 0.35))
    now = float(getattr(rend.engine, "t", 0.0))
    arrived = max(robot.t_vel, robot.t_cmd)
    if arrived < 0.0 or now - arrived > timeout:
        return [("cmd none", mix(GREY, LINK_AMBER, 0.6))]
    return [(f"cmd {'keys' if robot.cmd_keys >= arrived else 'topic'} "
             f"{now - arrived:.2f} s", GREY)]


def network(rend) -> None:
    """The access point, one line per robot that fades with its quality, one bar per robot.

    The line is the model in one stroke: it is drawn along the very straight line the budget is
    computed on, so the moment the bar drops as the line crosses a rack is the same fact as "the LIDAR
    reports that rectangle" — no coincidence, the radio reuses the LIDAR's ray test (wifi.py). The bar
    answers "how is this robot doing", `link_readout()` answers "what is the number", and both are
    drawn from one call into the radio, so they cannot disagree.
    """
    from .render import GREY, mix                        # lazy: render imports this module
    sc = rend.screen
    views = {r.spec.name: radio_health(rend, r) for r in (getattr(rend.engine, "robots", {})
                                                          or {}).values()}
    views = {k: v for k, v in views.items() if v is not None}
    if not views:
        return
    ap = next(iter(views.values()))[8]
    _ap_symbol(rend, ap)
    for robot in (getattr(rend.engine, "robots", {}) or {}).values():
        view = views.get(robot.spec.name)
        if view is None:
            continue
        q, _dbm, _metres, _walls, up, _dropped, _sent, _latency, _ap = view
        colour = mix(GREY, GOOD_LINK, q) if up else DEAD_LINK
        a, b = rend.px(*ap), rend.px(robot.pose.x, robot.pose.y)
        if up:
            pygame.draw.line(sc, colour, a, b, 2 if q > 0.66 else 1)
        else:
            _dashed_line(sc, a, b, colour)               # no link: the line is a memory of one
        bar = (b[0] - BAR_W // 2, b[1] - 30)
        pygame.draw.rect(sc, mix(GREY, rend.col_floor, .35), (*bar, BAR_W, BAR_H), 1)
        filled = int((BAR_W - 3) * q)
        if filled:
            pygame.draw.rect(sc, colour, (bar[0] + 1, bar[1] + 1, filled, BAR_H - 2))
        rend._text(link_text(view), bar[0] - 8, above_bar(rend, bar[1] - 15), colour)


def autonomy_mark(rend, robot) -> None:
    """The amber outline of a robot the link gave up on — drawn whatever the layers show.

    Deliberately not part of `network()`: a layer can be hidden, the fact that this robot is no longer
    being told cannot. Draws nothing unless `mode` says autonomy, so in every run without a radio the
    frame stays the frame it always was.
    """
    if getattr(robot, "mode", "") != "autonomy":
        return
    sc = rend.screen
    geom = getattr(robot.chassis, "geom", None)
    reach = float(getattr(geom, "footprint_r", 0.21)) * rend.s + 6.0
    centre = rend.px(robot.pose.x, robot.pose.y)
    pygame.draw.circle(sc, LINK_AMBER, centre, int(reach), 2)
    for tick in range(3):                       # three ticks: nothing is coming in from anywhere
        angle = math.pi / 2.0 + tick * 2.0 * math.pi / 3.0
        ux, uy = math.cos(angle), math.sin(angle)
        pygame.draw.line(sc, LINK_AMBER, (centre[0] + ux * reach, centre[1] - uy * reach),
                         (centre[0] + ux * (reach + 7), centre[1] - uy * (reach + 7)), 2)


def _ap_symbol(rend, ap) -> None:
    """The access point: a box with three arcs — the one symbol in this lab that means "radio".

    Drawn as a box and not as a burst like a radiation source: the source is a point that something
    comes out of, this is a point everything has to come back to. The arcs are one symbol for all
    qualities — the AP does not get worse, the line to it does.
    """
    from .render import mix                                   # lazy: render imports this module
    sc, s = rend.screen, rend.s
    centre = rend.px(*ap)
    colour = mix(rend.col_floor, GOOD_LINK, .85)
    pygame.draw.rect(sc, colour, (centre[0] - 5, centre[1] - 3, 10, 7))
    for step in (1, 2):
        box = pygame.Rect(centre[0] - 7 * step, centre[1] - 7 * step - 3, 14 * step, 14 * step)
        pygame.draw.arc(sc, colour, box, 0.15 * math.pi, 0.85 * math.pi, 1)
    if s > 26:
        rend._text("access point", centre[0] + 13, above_bar(rend, centre[1] - 16), colour)


def _dashed_line(sc, a, b, color) -> None:
    """A dashed line between two points: what is left of a link that carries nothing."""
    length = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
    for walk in range(0, int(length), 14):
        end = min(walk + 8, length)
        pygame.draw.line(sc, color, (a[0] + ux * walk, a[1] + uy * walk),
                         (a[0] + ux * end, a[1] + uy * end), 1)


def _plate(point, u, v, half_l, half_w) -> list:
    """Corners of a box around `point`, spanned by the screen vectors u (long) and v (wide)."""
    return [(point[0] + u[0] * half_l * su + v[0] * half_w * sv,
             point[1] + u[1] * half_l * su + v[1] * half_w * sv)
            for su, sv in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def _dashed(sc, corner, size, color) -> None:
    """Dashed border around one zone rectangle."""
    x, y = corner
    breite, height = size
    for edge in ((x, y, x + breite, y), (x, y + height, x + breite, y + height),
                  (x, y, x, y + height), (x + breite, y, x + breite, y + height)):
        length = math.hypot(edge[2] - edge[0], edge[3] - edge[1]) or 1.0
        dx, dy = (edge[2] - edge[0]) / length, (edge[3] - edge[1]) / length
        for wandern in range(0, int(length), 12):
            end = min(wandern + 7, length)
            pygame.draw.line(sc, color, (edge[0] + dx * wandern, edge[1] + dy * wandern),
                             (edge[0] + dx * end, edge[1] + dy * end), 1)
