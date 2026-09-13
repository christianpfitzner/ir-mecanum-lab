#!/usr/bin/env python3
"""How the lab works, drawn from the code that does it: the chassis and the loop.

Two panels, one figure, nothing drawn by hand:

  A  **the chassis** — the plate, the four wheels on their axle points, each wheel's roller axis, the
     three body velocities, and the one rule that makes a strafe possible. Every number in this panel
     comes from the modules the exercise is graded with: `render.wheel_mounts()` says where the wheels
     sit, `render.ROLLERS` which way their rollers lie, `physics.Geometry` how far apart that is, and
     `physics.inverse_kinematics()` what a command costs the four motors. Before the first pixel is
     drawn, `check_roller_axes()` asks `physics.forward_kinematics()` for every wheel on its own
     whether the velocity that wheel produces stands perpendicular to the roller axis the figure is
     about to draw. A mecanum picture whose rollers do not match its equations is worse than no
     picture — it teaches the mirrored robot — so the tool stops with an error instead of writing one.

  B  **the loop** — one command becoming four wheel speeds becoming a pose becoming six sensor
     messages becoming an estimate. The topic names are `types.topic()` and the message types are
     `types.MSG_SPECS`, so this figure cannot go on naming a topic that was renamed. Two cards carry
     a red border: the two places where the simulator lets a number lie on purpose (wheel slip at an
     obstacle, wrong wheel constants in `odom.geometry`). One card is dashed: the truth topic, the
     only message that is never noisy and the only one that is off by default.

The canvas is 1400 x 900 px, text is 14 px or larger, and `Sheet.place()` refuses a layout in which
two labels overlap or one leaves the canvas: a documentation figure with colliding labels is a figure
nobody trusts, so that is a failed run rather than a bad screenshot.

Headless (`SDL_VIDEODRIVER=dummy`), deterministic (no clock, no random — run it twice and the bytes
match), stdlib + pygame only (CONTRACT section 1).

    python3 tools/labmap.py                       # docs/img/howitworks.png
    python3 tools/labmap.py --out /tmp/map.png --scale 700
"""
import argparse
import math
import os
import sys

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")       # a picture needs no screen
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame                                           # noqa: E402

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
from mecanum_lab import physics, render                 # noqa: E402
from mecanum_lab.types import DEFAULT_CONFIG, MSG_SPECS, topic          # noqa: E402

WIDTH, HEIGHT = 1400, 900
MARGIN, GAP, FOOTER = 28, 24, 26
PANEL_TOP, PANEL_BOTTOM = 96, HEIGHT - MARGIN - FOOTER
PANEL_A_WIDTH = 470
PER_M = 620                     # pixels per metre in panel A; the robot is 0.38 m across
ROBOT = "alice"

BG, CARD = (248, 248, 251), (255, 255, 255)
INK, SOFT, FAINT = (32, 36, 46), (96, 102, 116), (150, 156, 168)
PLATE, WHEEL = (223, 232, 246), (58, 64, 78)
BODY, ROLLER = (66, 106, 190), (206, 120, 20)
DRIVE, MEASURE, TRUTH, LIE = (26, 128, 92), (96, 102, 116), (140, 88, 180), (196, 56, 48)


class Guard(Exception):
    """The figure would lie or come out unreadable: wrong geometry, overlapping, cut off."""


# ------------------------------------------------------------------------------------- the canvas
class Sheet:
    """A surface with a memory of where it has already put text.

    `place()` is why this is a class and not a list of draw calls: labels and cards register their
    rectangle, so an overlap or something outside the canvas ends the run with the two boxes named.
    Lines and arrows are not obstacles (they are what the labels are about), but a label that lands
    on another label is a mistake, and the layout of a 900 px tall figure is not guessable.
    """

    def __init__(self, width=WIDTH, height=HEIGHT):
        pygame.font.init()
        self.sc = pygame.Surface((width, height))
        self.sc.fill(BG)
        self.used, self.fonts, self.texts = [], {}, []

    def font(self, size):
        self.fonts.setdefault(size, pygame.font.Font(None, size))
        return self.fonts[size]

    def place(self, rect, what="element"):
        rect = pygame.Rect(rect)
        if not self.sc.get_rect().contains(rect):
            raise Guard(f"{what} at {rect.topleft} {rect.size} would leave the "
                        f"{self.sc.get_width()}x{self.sc.get_height()} canvas")
        for other, name in self.used:
            if rect.colliderect(other):
                raise Guard(f"{what} at {rect.topleft} {rect.size} overlaps {name} at "
                            f"{other.topleft} {other.size}")
        self.used.append((rect, what))
        return rect

    def label(self, x, y, string, size=17, colour=INK, centre=False, name=None):
        """Text with two pixels of air around it registered as taken."""
        img = self.font(size).render(string, True, colour)
        rect = img.get_rect(center=(x, y)) if centre else img.get_rect(topleft=(x, y))
        self.place(rect.inflate(4, 2), name or f"'{string[:30]}'")
        self.texts.append(string)
        self.sc.blit(img, rect)
        return rect

    def card(self, x, y, width, title, lines=(), edge=INK, dashed=False, size=17):
        """A bordered box with a title and up to two soft-coloured lines under it."""
        wide = max([self.font(size).size(title)[0]] +
                   [self.font(size - 2).size(line)[0] for line in lines]) + 24
        rect = pygame.Rect(x, y, max(width, wide), 28 + 17 * len(lines) + 8)
        self.place(rect.inflate(4, 4), f"card '{title[:26]}'")
        self.texts.extend([title, *lines])
        pygame.draw.rect(self.sc, CARD, rect)
        if dashed:
            dashed_rect(self.sc, rect, edge)
        else:
            pygame.draw.rect(self.sc, edge, rect, 2)
        self.sc.blit(self.font(size).render(title, True, INK), (rect.x + 10, rect.y + 6))
        for index, line in enumerate(lines):
            self.sc.blit(self.font(size - 2).render(line, True, SOFT),
                         (rect.x + 10, rect.y + 27 + 17 * index))
        return rect

    def arrow(self, start, end, colour=INK, width=3, dashed=False):
        """One straight segment with a head. Its ends have to be on the canvas; its middle may cross
        anything, because a wire in a wiring diagram crosses wires and cards all the time. What may not
        happen is a wire that runs off the figure — that is what the dashed truth line did on its way to
        the sensor row, and a line that leaves the page has no ending for the reader to follow."""
        for where, point in (("start", start), ("end", end)):
            if not (0 <= point[0] <= self.sc.get_width() and 0 <= point[1] <= self.sc.get_height()):
                raise Guard(f"arrow {where} {tuple(int(v) for v in point)} leaves the canvas")
        if dashed:
            dashed_line(self.sc, start, end, colour, width)
        else:
            pygame.draw.line(self.sc, colour, start, end, width)
        head(self.sc, start, end, colour, width)

    def route(self, start, end, colour, width=2, dashed=False, via="horizontal"):
        """A right-angle route: two straight segments read as a wiring diagram, one diagonal does not."""
        corner = (end[0], start[1]) if via == "vertical_first" else (start[0], end[1])
        self.arrow(start, corner, colour, width, dashed)
        self.arrow(corner, end, colour, width, dashed)

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        pygame.image.save(self.sc, path)


def head(sc, a, b, colour, width):
    """Arrow head at `b`, pointing away from `a` — the pygame draw set has none of its own."""
    dx, dy = (b[0] - a[0], b[1] - a[1])
    length = math.hypot(dx, dy) or 1.0
    dx, dy = dx / length, dy / length
    barbs = max(7, 3 * width + 3)
    for turn in (2.7, -2.7):
        cos_t, sin_t = math.cos(turn), math.sin(turn)
        pygame.draw.line(sc, colour, b, (b[0] - barbs * (dx * cos_t - dy * sin_t),
                                       b[1] - barbs * (dx * sin_t + dy * cos_t)),
                         max(1, width - 1))


def dashed_line(sc, a, b, colour, width=2, stitch=9, gap=6):
    length = math.hypot(b[0] - a[0], b[1] - a[1]) or 1.0
    ux, uy = (b[0] - a[0]) / length, (b[1] - a[1]) / length
    walked, on = 0.0, True
    while walked < length:
        step = min(stitch if on else gap, length - walked)
        if on:
            pygame.draw.line(sc, colour, (a[0] + ux * walked, a[1] + uy * walked),
                             (a[0] + ux * (walked + step), a[1] + uy * (walked + step)), width)
        walked, on = walked + step, not on


def dashed_rect(sc, rect, colour):
    x, y, wide, tall = rect
    for a, b in (((x, y), (x + wide, y)), ((x + wide, y), (x + wide, y + tall)),
                 ((x + wide, y + tall), (x, y + tall)), ((x, y + tall), (x, y))):
        dashed_line(sc, a, b, colour, 2)


# -------------------------------------------------------------------------------- the geometry rule
def wheel_report(geometry):
    """One row per wheel: its name, the roller axis the figure will draw, and how far from 90 degrees.

    The check the whole panel stands on: a roller wheel pushes only across its rollers, so the
    velocity of *its own centre* when that one wheel turns and the other three stand still has to be
    perpendicular to the roller axis. The centre's velocity and not the body velocity, because in a
    turn the two differ and only the first is what the tyre sees. `physics.forward_kinematics()` is
    the same equation the grader's T1 measures, so the figure and the task cannot drift apart here.
    """
    rows = []
    for index, (sx, sy) in enumerate(render.CORNERS):
        speeds = [0.0] * 4
        speeds[index] = 1.0
        vx, vy, omega = physics.forward_kinematics(geometry, speeds)
        centre_v = (vx - omega * sy * geometry.ly, vy + omega * sx * geometry.lx)
        axis = render.ROLLERS[index]
        both = math.hypot(*centre_v) * math.hypot(*axis)
        if both == 0:
            raise Guard(f"{physics.WHEELS_EN[index]} produces no motion at all — there is nothing to draw")
        rows.append((physics.WHEELS_EN[index], axis,
                     (centre_v[0] * axis[0] + centre_v[1] * axis[1]) / both))
    return rows


def check_roller_axes(geometry):
    """Refuse to draw the figure when the rollers and the equations disagree.

    The tolerance is a degree and a half, which is what the stock chassis's `lx != ly` costs; a
    mirrored diagonal — the O arrangement, the same robot with the other diagonal — comes out at
    0.99, and no tolerance covers that.
    """
    off = [(name, value) for name, _axis, value in wheel_report(geometry) if abs(value) > 0.05]
    if off:
        raise Guard("roller axes do not match the kinematics: "
                    + ", ".join(f"{name} is {value:+.2f} from perpendicular" for name, value in off)
                    + " — this figure would show a robot that cannot strafe the way its readout does")


# ------------------------------------------------------------------------------------- panel A
def panel_chassis(sheet, area, geometry, per_m):
    """The robot at 620 px/m: where the wheels are, how their rollers lie, what a body velocity is."""
    sheet.label(area.x, area.y, "A   the chassis: four wheels, one X", 25, INK)
    sheet.label(area.x, area.y + 28, "body frame: x forward, y to the left, theta CCW (CONTRACT 5)",
                15, SOFT)
    cx, cy = area.x + 200, area.y + 250
    middle = (cx, cy)

    def px(x_metres, y_metres):
        """Metres in the body frame -> pixels, y up: the handout's picture, not the window's."""
        return (cx + x_metres * per_m, cy - y_metres * per_m)

    long_side, wide_side = (geometry.lx + geometry.r) * per_m, (geometry.ly + geometry.r) * per_m
    plate = pygame.Rect(int(cx - long_side), int(cy - wide_side), int(2 * long_side),
                        int(2 * wide_side))
    pygame.draw.rect(sheet.sc, PLATE, plate)
    pygame.draw.rect(sheet.sc, BODY, plate, 2)
    wheel_long = min(geometry.r * 2.0, geometry.footprint_r - geometry.ly) * per_m
    wheel_wide = wheel_long * render.WHEEL_WIDTH_RATIO
    for (name, _axis, _off), (sx, sy), (mx, my) in zip(wheel_report(geometry), render.CORNERS,
                                                       render.wheel_mounts(geometry.lx, geometry.ly)):
        point = px(mx, my)
        tyre = pygame.Rect(int(point[0] - wheel_long / 2), int(point[1] - wheel_wide / 2),
                           int(wheel_long), int(wheel_wide))
        pygame.draw.rect(sheet.sc, WHEEL, tyre)
        pygame.draw.rect(sheet.sc, (28, 32, 40), tyre, 1)
        axis = render.ROLLERS[render.CORNERS.index((sx, sy))]
        norm = math.hypot(*axis)
        ux, uy = axis[0] / norm, -axis[1] / norm       # y flips: pixels go down, the body frame up
        span = wheel_long * 0.82
        for share in (-0.34, 0.0, 0.34):               # the three rollers of the wheel
            here = (point[0] + ux * span * share, point[1] + uy * span * share)
            pygame.draw.line(sheet.sc, ROLLER,
                             (here[0] - ux * span / 2, here[1] - uy * span / 2),
                             (here[0] + ux * span / 2, here[1] + uy * span / 2), 3)
        sheet.label(point[0] + sx * (wheel_long * .75), point[1] - sy * (wheel_wide * .5 + 15),
                    f"{name} / {physics.WHEELS[render.CORNERS.index((sx, sy))]}", 15, INK, centre=True)

    # a leader to one roller axis, so the orange strokes have a name
    fl = px(*render.wheel_mounts(geometry.lx, geometry.ly)[0])
    sheet.arrow((fl[0] + 8, fl[1] - 8), (fl[0] + 60, fl[1] - 60), ROLLER, 2)
    sheet.label(fl[0] + 112, fl[1] - 76, "roller axis at 45 deg", 15, ROLLER, centre=True)
    sheet.label(fl[0] + 112, fl[1] - 56, "the wheel pushes across it", 14, SOFT, centre=True)

    # the three body velocities, on the robot: what one `cmd_vel` asks for. The y arrow stops under the
    # subtitle: it is 0.34 m of sideways in a figure at 620 px/m, and the last 40 px of it were drawn
    # across the line "body frame: x forward, y to the left" — a line that tells the reader how to read
    # the arrow, drawn through it.
    sheet.arrow(middle, px(0.0, 0.28), DRIVE, 4)
    sheet.arrow(middle, px(0.36, 0.0), DRIVE, 4)
    arc(sheet, middle, 0.155 * per_m, 25, 150, DRIVE)
    sheet.label(area.x + 112, area.y + 68, "vy  sideways", 15, DRIVE, name="vy label")
    sheet.label(px(0.30, -0.075)[0], px(0.30, -0.075)[1], "vx  forward", 15, DRIVE, centre=True,
                name="vx label")
    sheet.label(*px(0.215, 0.075), "omega", 15, DRIVE, centre=True, name="omega label")
    sheet.label(area.x, plate.bottom + 26, f"plate {2 * (geometry.lx + geometry.r):.2f} x "
                f"{2 * (geometry.ly + geometry.r):.2f} m, wheels at +/-{geometry.lx:.2f} x "
                f"+/-{geometry.ly:.2f} m, a = {geometry.arm:.2f} m", 15, SOFT)

    note = ("One rule makes a strafe possible.",
            "A wheel can push only across its own roller axis; along it, it rolls freely.",
            "Front left and rear right are mounted one way, front right and rear left the other.",
            "All four the same sign: the sideways parts cancel, the forward parts add — forward.",
            "FL and RR against FR and RL: the forward parts cancel, the rest adds — a strafe.",
            "Turn rate costs the lever arm a = lx + ly: a wheel further out means fewer rad/s.")
    sheet.label(area.x, area.y + 520, note[0], 18, INK, name="rule title")
    for number, line in enumerate(note[1:]):
        sheet.label(area.x, area.y + 548 + 20 * number, line, 15, SOFT, name=f"rule line {number}")
    speeds = physics.inverse_kinematics(geometry, 0.0, 0.30, 0.0)
    tail = area.y + 548 + 20 * len(note[1:]) + 8
    sheet.label(area.x, tail,
                "inverse_kinematics(vx=0, vy=0.30, omega=0)  ->  "
                + "   ".join(f"{name} {speed:+5.1f}" for (name, _a, _o), speed
                            in zip(wheel_report(geometry), speeds)) + " rad/s",
                16, DRIVE, name="strafe numbers")
    sheet.label(area.x, tail + 24,
                "opposite signs on the two pairs, equal size: the line above is the frame on the left",
                14, SOFT, name="strafe caption")


def arc(sheet, origin, radius, from_deg, to_deg, colour):
    """The turn-rate arrow: pygame draws the arc, the head has to be added by hand."""
    box = pygame.Rect(int(origin[0] - radius), int(origin[1] - radius), int(2 * radius),
                      int(2 * radius))
    pygame.draw.arc(sheet.sc, colour, box, math.radians(from_deg), math.radians(to_deg), 4)
    stop = math.radians(to_deg)
    end = (origin[0] + radius * math.cos(stop), origin[1] - radius * math.sin(stop))
    before = (origin[0] + (radius + 24) * math.cos(stop + 0.35),
              origin[1] - (radius + 24) * math.sin(stop + 0.35))
    head(sheet.sc, before, end, colour, 4)


# ------------------------------------------------------------------------------------- panel B
def panel_loop(sheet, area):
    """The command-to-estimate chain, with the two places a number is allowed to lie marked.

    The grid: three columns, chain down the left two, and a free column on the right so the
    `odom.geometry` note can sit next to the one card it corrupts instead of throwing an arrow across
    the whole panel. The six sensors hang off one dashed bus line, because that is what they are: six
    readings of one truth, none of them the truth.
    """
    sheet.label(area.x, area.y, "B   the loop: command, wheels, truth, six sensors, estimate",
                25, INK)
    sheet.label(area.x, area.y + 28,
                "topic names from types.topic(), message types from types.MSG_SPECS", 15, SOFT)
    left, top, step, row_h = area.x, area.y + 66, 84, 74
    near, far = 214, 244
    note_x = left + 2 * (near + 46) + 46                      # the free column on the right
    cmd = sheet.card(left, top, near, topic("twist", ROBOT),
                     [MSG_SPECS["twist"][0].split("/")[-1], "what you want"], DRIVE)
    ik = sheet.card(left + near + 46, top, far, "inverse_kinematics()",
                    ["one Twist, four speeds", "clamped to max_speed"], INK)
    wheels = sheet.card(left, top + step, near, topic("wheels", ROBOT),
                        [MSG_SPECS["wheels"][0].split("/")[-1], geometry_label() + ", rad/s"], DRIVE)
    chassis = sheet.card(left + near + 46, top + step, far, "physics.Chassis.step()",
                         ["motor lag, kinematics,", "pose, wall collision"], INK)
    slip = sheet.card(left, top + 2 * step, near, "blocked by a wall",
                      ["body stops, wheels spin on", "robot.slip = " + str(slip_value())], LIE)
    truth = sheet.card(left + near + 46, top + 2 * step, far, topic("truth", ROBOT),
                       [MSG_SPECS["truth"][0].split("/")[-1], "no noise, off by default"],
                       TRUTH, dashed=True)
    sheet.arrow((cmd.right + 4, cmd.centery), (ik.left - 4, ik.centery), DRIVE, 3)
    sheet.route((ik.left + 24, ik.bottom), (wheels.centerx, wheels.top - 2), DRIVE, 3,
                via="vertical_first")
    sheet.route((ik.right - 24, ik.bottom), (chassis.centerx, chassis.top - 2), INK, 2,
                via="vertical_first")
    sheet.arrow((wheels.right + 4, wheels.centery), (chassis.left - 4, chassis.centery), DRIVE, 3)
    sheet.arrow((chassis.centerx, chassis.bottom), (truth.centerx, truth.top - 2), TRUTH, 2,
                dashed=True)
    sheet.route((slip.right + 4, slip.centery), (chassis.left + 20, chassis.bottom), LIE, 2)

    bus_y = top + 2 * step + row_h + 34
    sheet.label(area.x, bus_y - 18, "one truth, six measurements of it — each with its own flaws",
                17, SOFT, name="sensor row title")
    feed_x = note_x + 210                       # the truth comes down at the right, clear of the cards
    sheet.route((truth.right, truth.centery), (feed_x, truth.centery), TRUTH, 2, dashed=True)
    sheet.arrow((feed_x, truth.centery), (feed_x, bus_y), TRUTH, 2, dashed=True)
    wide, gap, per_row = 182, 18, 3
    cards = {}
    sensors = (("gps", "quality, sats, lost", MEASURE), ("imu", "bias walks with temperature", MEASURE),
               ("odom", "counters, not the floor", LIE), ("scan", "no echo stays inf", MEASURE),
               ("poi", "a field, not a distance", MEASURE), ("link", "the radio, not the floor", MEASURE))
    for index, (kind, note, colour) in enumerate(sensors):
        x = area.x + index % per_row * (wide + gap)
        row = index // per_row
        y = bus_y + 30 + row * 84
        cards[kind] = sheet.card(x, y, wide, topic(kind, ROBOT),
                                 [MSG_SPECS[kind][0].split("/")[-1], note], colour)
        if row == 0:
            sheet.arrow((cards[kind].centerx, bus_y), (cards[kind].centerx, cards[kind].top - 2),
                        colour, 2)
        else:
            # The second row hangs off the bus at the *left* of its column: a drop at the column centre
            # would start above the first row and be drawn down through its text, and a figure whose
            # lines cut through the words on the cards is a figure nobody reads.
            corner = area.x + (index % per_row) * (wide + gap) - gap // 2
            sheet.route((corner, bus_y), (cards[kind].left - 2, cards[kind].centery), colour, 2,
                        via="vertical_first")
    wrong = sheet.card(cards["odom"].right + 24, cards["odom"].y, 210, "odom.geometry",
                       ["a wheel radius the", "chassis does not have"], LIE)
    sheet.arrow((wrong.left - 4, wrong.centery), (cards["odom"].right + 4, cards["odom"].centery),
                LIE, 2)

    below = bus_y + 30 + 2 * 84
    sheet.label(area.x, below + 2, "what the node makes of all six", 17, SOFT,
                name="node row title")
    node = sheet.card(area.x + 2 * (wide + gap) + 30, below + 26, 240, "your node",
                      ["reads, decides, estimates"], INK)
    answer = sheet.card(area.x, below + 26, 240, topic("kf", ROBOT),
                        [MSG_SPECS["kf"][0].split("/")[-1], "position and its own sigma"], DRIVE)
    # One collector instead of six crossings. Six routes from six cards to one node cut each other and
    # the cards between them, and the reading of it is a mess; the node takes all six together, so the
    # figure draws one line into the node and reaches it from every card through the gaps between the
    # columns. Nothing in this figure is drawn over the words on a card.
    collector = bus_y + 30 + 84 + 76                          # under both rows, above the next label
    sheet.arrow((area.x + 3 * (wide + gap) - gap, collector), (area.x, collector), MEASURE, 2)
    for index, card in enumerate(cards.values()):
        if index < per_row:                                   # first row: out the right, down the gap
            from_x, from_y = card.right + 2, card.centery
            down_x = card.right + gap // 2
            sheet.route((from_x, from_y), (down_x, collector), MEASURE, 2)
        else:                                                 # second row: straight down
            sheet.arrow((card.centerx, card.bottom), (card.centerx, collector), MEASURE, 2)
    sheet.arrow((area.x + (wide + gap), collector), (node.centerx, node.top - 2), MEASURE, 2)
    sheet.arrow((node.left - 4, node.centery), (answer.right + 4, answer.centery), DRIVE, 4)
    legend(sheet, pygame.Rect(area.x, below + 108, area.width, PANEL_BOTTOM - below - 112))


def slip_value():
    """The default slip factor, so the card quotes the config rather than a number typed here."""
    return DEFAULT_CONFIG["robot"]["slip"]


def geometry_label():
    """The wheel order as the handout prints it — from the module, not from this file."""
    return " ".join(physics.WHEELS)


def legend(sheet, area):
    """Every line style in the figure next to the sentence it means."""
    sheet.label(area.x, area.y, "the styles", 17, SOFT, name="legend title")
    rows = (("command — what you asked for", DRIVE, False),
            ("measurement — what arrived", MEASURE, False),
            ("truth — never noisy, off by default", TRUTH, True),
            ("a number allowed to lie", LIE, False))
    for index, (text, colour, dashed) in enumerate(rows):
        y = area.y + 24 + index * 22
        sheet.arrow((area.x + 6, y), (area.x + 62, y), colour, 3, dashed=dashed)
        sheet.label(area.x + 74, y - 8, text, 15, SOFT, name=f"legend '{text[:16]}'")
    x = area.x + 330
    sheet.label(x, area.y, "the two places the simulator lies on purpose", 17, LIE,
                name="lies title")
    for index, line in enumerate((
            "robot.slip — at a wall the body stops, the wheels keep the speed the motor demands, and",
            "the odometry integrates what the wheels report: metres that were never driven.",
            "odom.geometry — the integrator may believe a wheel radius or lever arm the chassis does",
            "not have, so every metre is wrong by the same factor and nothing ever averages it out.")):
        sheet.label(x, area.y + 26 + 19 * index, line, 14, SOFT, name=f"lies line {index}")


# ------------------------------------------------------------------------------------------ whole
def make(path, per_m=PER_M):
    """Draw both panels into one PNG. Layout mistakes raise Guard instead of shipping."""
    pygame.display.init()
    robot = DEFAULT_CONFIG["robot"]
    geometry = physics.Geometry(**{key: robot[key] for key in ("lx", "ly", "r", "footprint_r")})
    check_roller_axes(geometry)                       # before a single pixel
    sheet = Sheet()
    area_a = pygame.Rect(MARGIN, PANEL_TOP, PANEL_A_WIDTH, PANEL_BOTTOM - PANEL_TOP)
    area_b = pygame.Rect(MARGIN + PANEL_A_WIDTH + GAP, PANEL_TOP,
                         WIDTH - 2 * MARGIN - PANEL_A_WIDTH - GAP, PANEL_BOTTOM - PANEL_TOP)
    for area in (area_a, area_b):
        pygame.draw.rect(sheet.sc, CARD, area, 1)
    panel_chassis(sheet, area_a, geometry, per_m)
    panel_loop(sheet, area_b)
    sheet.label(WIDTH // 2, 32, "The mecanum lab: the chassis, and the loop around it", 29, INK,
                centre=True, name="title")
    sheet.label(WIDTH // 2, 62, "drawn by tools/labmap.py from the modules it shows — run it again "
                                "and the file is byte for byte the same", 15, SOFT, centre=True,
                name="subtitle")
    sheet.label(WIDTH // 2, HEIGHT - 14, "signs: CONTRACT section 5  ·  topics: section 6  ·  "
                "both drive trains share this loop (steering.py replaces one box)", 14, FAINT,
                centre=True, name="footer")
    sheet.save(path)
    print(f"{path}: {sheet.sc.get_width()}x{sheet.sc.get_height()} px · panel A chassis at "
          f"{per_m:g} px/m, panel B the loop")
    pygame.quit()
    return path


def main(argv=None):
    parser = argparse.ArgumentParser(description="Draw how the lab works into one PNG.")
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "img", "howitworks.png"))
    parser.add_argument("--scale", type=float, default=PER_M, metavar="PX_PER_M",
                        help=f"pixels per metre in panel A (default {PER_M:g}); panel B is a "
                             "diagram and keeps its own sizes")
    args = parser.parse_args(argv)
    make(args.out, args.scale)


if __name__ == "__main__":
    main()
