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

Stdlib + pygame only, no state in the module (marks live on the renderer), stdlib drawing calls
only (CONTRACT section 1). Text uses the renderer's own blit so both use the same fonts.
"""
import math

import pygame

SPUR_LEBEN = 3.0                # seconds a skid mark stays on the floor
MARK_MAX = 600                  # bound for a long teleop session


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
    from .render import mix                                   # lazy: render imports this module
    sc, s = rend.screen, rend.s
    fur_schatten = mix(rend.col_floor, (250, 210, 90), .11)     # hint, not a second floor
    fur_dunkel = mix(rend.col_floor, (240, 120, 120), .26)
    for zone in running_zones(rend):
        x0, y1 = rend.px(*zone["rect"][:2])                   # world metres -> pixels
        x1, y0 = rend.px(*zone["rect"][2:])
        corner = (min(x0, x1), min(y0, y1))
        half_l, be = abs(x1 - x0), abs(y0 - y1)                   # width and height on screen
        if half_l < 2 or be < 2:
            continue
        color = fur_dunkel if zone["block"] else fur_schatten
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
                       above_bar(rend, corner[1] + 14), color)


def zone_label(zone: dict) -> str:
    """What the shadow is worth, straight out of the config: `shelf  σ×6, bias +0.8/-0.5 m`."""
    if zone["block"]:
        return zone["name"] + "  no fix"
    zusatz = f", bias {zone['bias'][0]:+.1f}/{zone['bias'][1]:+.1f} m" if any(zone["bias"]) else ""
    return f"{zone['name']}  σ×{zone['sigma_scale']:g}{zusatz}"


def odom_ghost(rend, robot) -> None:
    """Ghost chassis at the odometry pose, leader line to the truth, gap in metres."""
    from .render import body, mix, rgb
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
    u, v = body(-o.theta, 1, 0), body(-o.theta, 0, 1)
    pygame.draw.polygon(sc, fill, _plate(ghost, u, v, half_l * rend.s, be * rend.s))
    pygame.draw.polygon(sc, edge, _plate(ghost, u, v, half_l * rend.s, be * rend.s), 1)
    if math.dist(truth, ghost) > 3:
        pygame.draw.line(sc, edge, ghost, truth, 1)
        rend._text(f"odom off {gap:.2f} m", (ghost[0] + truth[0]) / 2 + 8,
                   above_bar(rend, (ghost[1] + truth[1]) / 2 - 10), edge)


def skid_marks(rend, robot, dt: float) -> None:
    """Rubber on the floor while the wheels turn and the body cannot — `robot.slip` made visible."""
    from .render import body, mix
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
        life -= dt / SPUR_LEBEN
        if life <= 0:
            continue
        alive.append((x, y, theta, life))
        center = rend.px(x, y)
        stroke = body(-theta, 5 * life, 0)
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

    One number, and it is the one the students have to work from: the distance is not in the message
    (`poi.publish_distance` is off) and so is not in the line either — a readout that prints the
    answer ends the exercise. An empty list for a robot with no detector keeps the readout line of
    every graded run exactly as long as it was, in the same rule as `steer_readout()`.
    """
    msg = getattr(robot, "poi", None)
    if msg is None:
        return []
    from .render import GREY, mix                       # lazy: render imports this module
    loud = msg.intensity > 0.0 and msg.name != ""
    text = f"poi {msg.name or '-'} {msg.intensity:.3f}" \
        + (f" @{msg.distance:.2f} m" if msg.distance is not None else "")
    return [(text, mix(GREY, (250, 205, 90), 0.7) if loud else GREY)]


def above_bar(rend, y: float) -> float:
    """Keep a label out of the readout block: one 17 px line per robot, plus the header."""
    return max(y, 48 + 17 * len(getattr(rend.engine, "robots", {}) or {}))


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
