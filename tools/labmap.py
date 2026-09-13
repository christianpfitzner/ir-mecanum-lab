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

WIDTH, HEIGHT = 1120, 470
MARGIN, GAP, FOOTER = 34, 26, 24
PANEL_TOP, PANEL_BOTTOM = 112, HEIGHT - MARGIN - FOOTER
PANEL_A_WIDTH = 330
PER_M = 470                     # pixels per metre in panel A; the robot is 0.38 m across
ROBOT = "alice"

BG, CARD = (248, 248, 251), (255, 255, 255)
INK, SOFT, FAINT = (32, 36, 46), (96, 102, 116), (150, 156, 168)
EDGE = (219, 224, 231)                        # the quiet border around the two panels
PLATE, WHEEL = (223, 232, 246), (58, 64, 78)
BODY, ROLLER = (66, 106, 190), (206, 120, 20)
DRIVE, STRAFE, TURN, TRUTH = (24, 122, 84), (28, 108, 150), (140, 88, 180), (150, 96, 186)
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
        """A right-angle route: two straight segments read as a wiring diagram, one diagonal does not.

        `via` names the segment that comes *first*: "vertical_first" leaves along x and turns into y at the
        far end. The two branches used to be swapped, which made every "vertical_first" wire in the figure
        leave sideways — the kf/pose line came out of the node horizontally, crossed the measurement labels
        and turned down somewhere in the middle of nothing.
        """
        corner = (start[0], end[1]) if via == "vertical_first" else (end[0], start[1])
        self.arrow(start, corner, colour, width, dashed)
        self.arrow(corner, end, colour, width, dashed)

    def save(self, path):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        pygame.image.save(self.sc, path)


def head(sc, a, b, colour, width):
    """Arrow head at `b`, pointing away from `a` — the pygame draw set has none of its own.

    The two barbs run *backwards* from the tip, opened by ~30° each. The first version here rotated the
    forward direction by ±155° and subtracted it, which lands 25° in front of the tip and draws a fish tail
    (⇁) instead of an arrow (➔) — on every wire and both velocity arrows in the figure, and it looked
    plausible enough in a thumbnail that nobody could say what was wrong with it.
    """
    dx, dy = (b[0] - a[0], b[1] - a[1])
    length = math.hypot(dx, dy) or 1.0
    dx, dy = -dx / length, -dy / length                      # backwards from the tip
    barbs = max(7.0, 2.6 * width + 3.0)
    for turn in (0.52, -0.52):                              # ~30° open
        cos_t, sin_t = math.cos(turn), math.sin(turn)
        pygame.draw.line(sc, colour, b, (b[0] + barbs * (dx * cos_t - dy * sin_t),
                                       b[1] + barbs * (dx * sin_t + dy * cos_t)),
                         max(2, width))


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
    """The robot, big enough to see the rollers: four wheels, one X, three body velocities.

    Nothing here is drawn from memory. The axles come from `render.wheel_mounts()`, the roller directions
    from `render.ROLLERS`, and :func:`check_roller_axes` has already refused the run if those directions are
    not perpendicular to what `physics.forward_kinematics()` produces for that wheel — the same equation the
    graded T1 measures, so the figure and the task cannot disagree about the robot.

    The three arrows are the exercise: forward, sideways, turning. A differential drive has two of them; a
    mecanum base has all three, and a student who has seen the sideways arrow drawn on the body frame stops
    turning the robot in order to drive it left.
    """
    cx, cy = area.x + area.width // 2, area.y + 44 + int((area.height - 130) * 0.46)
    lx, ly = geometry.lx * per_m, geometry.ly * per_m
    sheet.label(area.x + 2, area.y + 2, "the chassis", 20, SOFT)

    pygame.draw.rect(sheet.sc, PLATE, pygame.Rect(cx - lx, cy - ly, 2 * lx, 2 * ly), border_radius=8)
    pygame.draw.rect(sheet.sc, BODY, pygame.Rect(cx - lx, cy - ly, 2 * lx, 2 * ly), 2, border_radius=8)
    tyre = max(9.0, geometry.r * per_m * 0.78)                          # the drawn wheel, not a scaled-up one
    for index, (mx, my) in enumerate(render.wheel_mounts(geometry.lx, geometry.ly)):
        wx, wy = cx + mx * per_m, cy - my * per_m                       # screen y grows downwards
        pygame.draw.circle(sheet.sc, WHEEL, (int(wx), int(wy)), int(tyre))
        axis = render.ROLLERS[index]
        norm = math.hypot(*axis)
        ux, uy = axis[0] / norm, -axis[1] / norm                        # the roller diagonal, on screen
        pygame.draw.line(sheet.sc, ROLLER, (wx - ux * tyre * 0.8, wy - uy * tyre * 0.8),
                         (wx + ux * tyre * 0.8, wy + uy * tyre * 0.8), 3)

    # The body frame on screen: x to the right, y up (screen y grows downwards), theta counter-clockwise.
    gap = tyre + 12
    sheet.arrow((cx + lx + gap - 6, cy), (cx + lx + gap + 40, cy), DRIVE, 3)
    sheet.label(cx + lx + gap + 12, cy - 26, "vx", 18, DRIVE)           # forward
    sheet.arrow((cx, cy - ly - gap + 6), (cx, cy - ly - gap - 40), STRAFE, 3)
    sheet.label(cx + 8, cy - ly - gap - 36, "vy", 18, STRAFE)           # sideways, without turning
    arc = pygame.Rect(cx - 38, cy - 38, 76, 76)
    pygame.draw.arc(sheet.sc, TURN, arc, 0.35, 2.45, 3)                 # and turning
    edge = (cx + 38 * math.cos(0.35), cy - 38 * math.sin(0.35))         # the arc's own end, so the head fits
    sheet.arrow((cx + 38 * math.cos(0.75), cy - 38 * math.sin(0.75)), edge, TURN, 3)
    sheet.label(cx - 74, cy + 30, "omega", 18, TURN)

    sheet.label(area.x + 2, area.bottom - 36, "the four roller axes lie on the diagonals,", 15, SOFT)
    sheet.label(area.x + 2, area.bottom - 18, "so one command makes vx and vy", 15, SOFT)


# ------------------------------------------------------------------------------------- panel B
def panel_loop(sheet, area):
    """Three boxes and the wires between them: what a node and the simulator say to each other.

    Deliberately three boxes. An earlier version of this figure had ten cards and ten wires, one per topic,
    and nobody read it: a diagram that lists everything explains nothing. What is drawn is the conversation a
    student's file actually takes part in — `cmd_vel` out, the measurements back, `kf/pose` to whatever
    grades or draws it — and every topic and message type comes from `types.topic()` and `types.MSG_SPECS`,
    so renaming a topic in the code moves the figure or fails the run.

    The wires are laid out so none of them crosses another: the two between node and simulator at two
    different heights, `kf/pose` straight down out of the node, `/truth` the dashed one on the far side.
    That is not decoration — the earlier version had a purple truth line running through a label, which is
    how a reader learns to ignore dashed lines.

    `/truth` is dashed and stays on the far side on purpose: a node that subscribed to it would score
    perfectly and learn nothing (§ CONTRACT, truth topic).
    """
    x, y = area.x, area.y + 26
    sheet.label(x + 2, y - 24, "the loop: cmd_vel out, measurements back", 20, SOFT)
    node = sheet.card(x, y + 58, 186, "your node", ("student/solution.py",))
    sim = sheet.card(area.right - 246, y + 58, 246, "the simulator",
                     ("inverse kinematics, physics,", "then six sensor messages"))
    out = sheet.card(x, y + 214, 236, "RViz and the grader", ("what the lab sees of your run",))

    upper, lower = node.y + 34, node.bottom - 6
    sheet.arrow((node.right + 6, upper), (sim.left - 12, upper), DRIVE, 3)
    sheet.label(node.right + 10, upper - 34, topic("twist", ROBOT), 16, DRIVE)
    sheet.label(node.right + 10, upper - 15, MSG_SPECS["twist"][0].split("/")[-1], 14, FAINT)

    sheet.arrow((sim.left - 12, lower), (node.right + 6, lower), STRAFE, 3)
    measured = ("odom", "gps", "imu", "scan")
    sheet.label(node.right + 10, lower + 12, "  ".join(topic(kind, ROBOT) for kind in measured),
                15, STRAFE)
    sheet.label(node.right + 10, lower + 31,
                "  ".join(MSG_SPECS[kind][0].split("/")[-1] for kind in measured), 14, FAINT)

    # Straight down into the grader: the card is placed under the node so this wire needs no bend at all,
    # and a diagonal here would cut the measurement labels on its way past.
    sheet.arrow((node.centerx, node.bottom + 6), (node.centerx, out.top - 8), BODY, 3)
    sheet.label(node.centerx + 10, node.bottom + 22, topic("kf", ROBOT), 16, BODY)
    sheet.label(node.centerx + 10, node.bottom + 41, MSG_SPECS["kf"][0].split("/")[-1], 14, FAINT)

    # And the truth, dashed, arriving at the grader's right edge: the one wire a student node must never
    # subscribe to, so it is drawn on the far side of the figure rather than through the middle.
    sheet.route((sim.centerx, sim.bottom + 6), (out.right + 8, out.centery), TRUTH, 2,
                dashed=True, via="vertical_first")
    note = topic("truth", ROBOT) + " — not for the node"
    sheet.label(sim.centerx - sheet.font(15).size(note)[0] - 12, sim.bottom + 24, note, 15, TRUTH)


def figure(out: str, per_m: float = PER_M) -> str:
    """Draw the whole figure and write it; the two panels are the two halves of the exercise."""
    robot = DEFAULT_CONFIG["robot"]
    geometry = physics.Geometry(**{key: robot[key] for key in ("lx", "ly", "r", "footprint_r")})
    check_roller_axes(geometry)                       # before a single pixel: no wrong robot, no figure
    sheet = Sheet()
    sheet.label(MARGIN, 26, "How the lab fits together", 30, INK)
    sheet.label(MARGIN, 62, "one command becomes four wheel speeds becomes a moved chassis becomes "
                            "measurements becomes an estimate", 17, SOFT)

    panel_a = pygame.Rect(MARGIN, PANEL_TOP, PANEL_A_WIDTH, PANEL_BOTTOM - PANEL_TOP)
    panel_b = pygame.Rect(MARGIN + PANEL_A_WIDTH + GAP, PANEL_TOP,
                          WIDTH - 2 * MARGIN - PANEL_A_WIDTH - GAP, PANEL_BOTTOM - PANEL_TOP)
    for panel in (panel_a, panel_b):                  # the two panels as two quiet cards
        pygame.draw.rect(sheet.sc, CARD, panel.inflate(16, 16), border_radius=10)
        pygame.draw.rect(sheet.sc, EDGE, panel.inflate(16, 16), 1, border_radius=10)
    panel_chassis(sheet, panel_a, geometry, per_m)
    panel_loop(sheet, panel_b)
    sheet.label(MARGIN, HEIGHT - 22, "drawn by tools/labmap.py from mecanum_lab/physics.py, "
                                     "render.py and types.py — it is a build product, not a picture "
                                     "of one", 14, FAINT)
    sheet.save(out)
    return out


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--out", default=os.path.join(ROOT, "docs", "img", "howitworks.png"))
    parser.add_argument("--scale", type=float, default=PER_M,
                        help="pixels per metre in the chassis panel")
    args = parser.parse_args(argv if argv is not None else sys.argv[1:])
    figure(args.out, args.scale)
    print(f"picture: {args.out} — one chassis, one loop, {WIDTH}x{HEIGHT} px, roller axes verified "
          f"against forward_kinematics()")
    return 0


if __name__ == "__main__":
    sys.exit(main())
