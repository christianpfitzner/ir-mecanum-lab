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


def zonen_im_lauf(rend) -> list:
    """The zones of the running configuration.

    Read from the live config dict, not from a snapshot: `set_task` merges a task profile into
    that same dictionary, so what is drawn is what the GPS is about to measure. Three rectangles
    are parsed per frame — that is cheaper than a cache that could go stale.
    """
    from .sensors import _zonen                                          # lazy, see module note
    gps = (getattr(rend, "cfg", None) or {}).get("gps") or {}
    return _zonen(gps.get("zones"))


def zones(rend) -> None:
    """Hatched shadow where the fix degrades, red hatch where there is no fix at all."""
    from .render import mix                                   # lazy: render imports this module
    sc, s = rend.screen, rend.s
    fur_schatten = mix(rend.col_floor, (250, 210, 90), .11)     # hint, not a second floor
    fur_dunkel = mix(rend.col_floor, (240, 120, 120), .26)
    for zone in zonen_im_lauf(rend):
        x0, y1 = rend.px(*zone["rect"][:2])                   # world metres -> pixels
        x1, y0 = rend.px(*zone["rect"][2:])
        ecke = (min(x0, x1), min(y0, y1))
        la, be = abs(x1 - x0), abs(y0 - y1)                   # width and height on screen
        if la < 2 or be < 2:
            continue
        farbe = fur_dunkel if zone["block"] else fur_schatten
        schritt = 26 if zone["block"] else 46                 # blackout reads denser than shadow
        for querversatz in range(-int(be), int(la), schritt):  # 45° hatch: shadow over there
            anfang, ende = max(0.0, -querversatz), min(be, la - querversatz)
            if ende > anfang:
                pygame.draw.line(sc, farbe,
                                 (ecke[0] + querversatz + anfang, ecke[1] + anfang),
                                 (ecke[0] + querversatz + ende, ecke[1] + ende), 1)
        _gestrichelt(sc, ecke, (la, be), mix(farbe, (255, 255, 255), .45))
        if la > 90 * max(s / 40, 0.25):
            rend._text(_zonen_text(zone), max(6, ecke[0] + 6),
                       _uber_der_leiste(rend, ecke[1] + 14), farbe)


def _zonen_text(zone: dict) -> str:
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
    farbe, sc = rgb(robot.spec.rgb), rend.screen
    truth = rend.px(robot.pose.x, robot.pose.y)
    schaum = rend.px(o.x, o.y)
    abstand = math.dist((robot.pose.x, robot.pose.y), (o.x, o.y))
    leib = mix(farbe, rend.col_floor, .74)
    kant = mix(farbe, rend.col_floor, .40)
    g = getattr(robot.chassis, "geom", None)
    la = (float(getattr(g, "lx", .14)) + float(getattr(g, "r", .05))) * rend.chassis_scale
    be = (float(getattr(g, "ly", .13)) + float(getattr(g, "r", .05))) * rend.chassis_scale
    u, v = body(-o.theta, 1, 0), body(-o.theta, 0, 1)
    pygame.draw.polygon(sc, leib, _platte(schaum, u, v, la * rend.s, be * rend.s))
    pygame.draw.polygon(sc, kant, _platte(schaum, u, v, la * rend.s, be * rend.s), 1)
    if math.dist(truth, schaum) > 3:
        pygame.draw.line(sc, kant, schaum, truth, 1)
        rend._text(f"odom off {abstand:.2f} m", (schaum[0] + truth[0]) / 2 + 8,
                   _uber_der_leiste(rend, (schaum[1] + truth[1]) / 2 - 10), kant)


def skid_marks(rend, robot, dt: float) -> None:
    """Rubber on the floor while the wheels turn and the body cannot — `robot.slip` made visible."""
    from .render import body, mix
    spur = rend.__dict__.setdefault("skid_spur", [])          # state on the renderer, not here
    g = getattr(robot.chassis, "geom", None)
    rder = getattr(robot, "wheels", None) or [0.0]
    if getattr(robot.chassis, "touching", False) and max(abs(w) for w in rder) > 0.5:
        la, be = float(getattr(g, "lx", .14)), float(getattr(g, "ly", .13))
        for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):   # one mark per wheel corner
            verschub = body(robot.pose.theta, sx * la, sy * be)
            spur.append((robot.pose.x + verschub[0], robot.pose.y + verschub[1],
                         robot.pose.theta, 1.0))
        del spur[:-MARK_MAX]
    sc, grund = rend.screen, mix(rend.col_floor, (0, 0, 0), .55)
    lebendig = []
    for x, y, theta, leben in spur:
        leben -= dt / SPUR_LEBEN
        if leben <= 0:
            continue
        lebendig.append((x, y, theta, leben))
        mittig = rend.px(x, y)
        strich = body(-theta, 5 * leben, 0)
        pygame.draw.line(sc, mix(grund, rend.col_floor, 1 - leben),
                         (mittig[0] - strich[0], mittig[1] - strich[1]),
                         (mittig[0] + strich[0], mittig[1] + strich[1]), 2)
    rend.skid_spur[:] = lebendig


def _uber_der_leiste(rend, y: float) -> float:
    """Keep a label out of the readout block: one 17 px line per robot, plus the header."""
    return max(y, 48 + 17 * len(getattr(rend.engine, "robots", {}) or {}))


def _platte(punkt, u, v, lang, breit) -> list:
    """Corners of a box around `punkt`, spanned by the screen vectors u (long) and v (wide)."""
    return [(punkt[0] + u[0] * lang * su + v[0] * breit * sv,
             punkt[1] + u[1] * lang * su + v[1] * breit * sv)
            for su, sv in ((1, 1), (1, -1), (-1, -1), (-1, 1))]


def _gestrichelt(sc, ecke, groesse, farbe) -> None:
    """Dashed border around one zone rectangle."""
    x, y = ecke
    breite, hoehe = groesse
    for kante in ((x, y, x + breite, y), (x, y + hoehe, x + breite, y + hoehe),
                  (x, y, x, y + hoehe), (x + breite, y, x + breite, y + hoehe)):
        laenge = math.hypot(kante[2] - kante[0], kante[3] - kante[1]) or 1.0
        dx, dy = (kante[2] - kante[0]) / laenge, (kante[3] - kante[1]) / laenge
        for wandern in range(0, int(laenge), 12):
            ende = min(wandern + 7, laenge)
            pygame.draw.line(sc, farbe, (kante[0] + dx * wandern, kante[1] + dy * wandern),
                             (kante[0] + dx * ende, kante[1] + dy * ende), 1)
