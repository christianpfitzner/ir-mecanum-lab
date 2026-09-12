"""Extra overlays for the window: GPS shadow zones, the odometry ghost, rubber from wheel slip.

Three small effects that make the invisible visible, none of which changes physics or a topic:

* `zones()` draws the rectangles of `gps.zones` (see sensors.GpsSensor) as hatched shadow, so a
  student can see *where* the fix goes bad before the logbook proves it. Blackout zones are drawn
  in the error colour.
* `odom_ghost()` draws where the odometry believes the robot is, next to the truth, with the
  distance between them in metres. Drift is otherwise a number in the readout nobody reads.
* `skid_marks()` leaves rubber on the floor while the wheels turn against an obstacle — the
  visible twin of the phantom distance that `robot.slip` produces.

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
