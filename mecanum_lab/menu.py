"""Two click menus, one piece of furniture: which layers are drawn, and where a robot is put.

Only the drawing is switched off in the view panel. The simulator publishes odom, scan, gps, imu and
the estimate on the bus at full rate regardless of what this panel shows, so hiding the lidar dots
does not hide a single message on `/<robot>/scan`. That is the point of the panel: students see which
layer belongs to which topic and can look at the ROS side without a full screen.

The second panel is the one the right button opens — a list of the robots that are driving, each row
meaning "put this one here". It is the same column of clickable rows with a different payload, so it
is the same class: a robot's *name* where the view panel has a renderer attribute, and the colour chip
of the robot where the view panel has a checkbox. Where it opens is the difference too: the view panel
keeps its corner, the robot menu opens under the pointer, because the place it speaks about is wherever
the pointer just was.

Both panels are translucent, so the map underneath stays readable. The view panel sits in the top
right corner, where `m` shows and hides it; the robot menu is gone again as soon as it has answered.
"""
import pygame

from . import keys

# (payload, label, hint) for the view panel — `keys.LAYERS` is the table, so the key printed in this
# panel is the key the window listens for. That used to be two lists, and they had already drifted: the
# panel promised `w` for the wheels while `w` drove. The payload of a view row is the renderer attribute
# it switches, of a robot row the name of the robot that would be placed.
LAYERS = keys.menu_rows()
HEAD = "view layers"
FUSS = "m shows and hides · click a row"
WIDTH, ROW, PAD = 210, 19, 6           # WIDTH is the minimum: `width()` widens it for long rows


class Panel:
    """A column of clickable rows; `handle` turns a click into the payload of one of them.

    Starts closed: a panel over the map would hide exactly the corner a goal is often placed in, and a
    robot menu that hangs around is an obstacle. `m` (or the HUD line that says so) opens the view
    panel, the right button opens the robot menu at the pointer; a click on a row answers, a click
    anywhere else on the panel closes it again.
    """

    def __init__(self, font, big, rows=None, head: str = HEAD, foot: str = FUSS,
                 at_pointer: bool = False):
        self.font, self.big = font, big
        self.head, self.foot, self.at_pointer = head, foot, at_pointer
        self.open, self.hot = False, -1
        self.rect = pygame.Rect(0, 0, WIDTH, ROW * len(LAYERS) + 52)
        self.set_rows(rows if rows is not None else LAYERS)

    def set_rows(self, rows) -> None:
        """The rows this panel offers now — and the height that belongs to them.

        The robot menu is built when it opens, because which robots are driving is not known when the
        window is. A panel that kept the height of its last content would leave a strip of translucent
        nothing over the map after a robot left the run.
        """
        self.rows = tuple(tuple(row) for row in rows)
        self.chips = {}                             # payload -> colour in front of its row, see `draw`
        self.rect.height = ROW * len(self.rows) + 52
        self.hot = -1

    def toggle(self) -> bool:
        self.open = not self.open
        return self.open

    def row_rect(self, index: int) -> pygame.Rect:
        """Hit area of one row, in screen coordinates."""
        return pygame.Rect(self.rect.x, self.rect.y + 24 + index * ROW, self.rect.width - 2, ROW)

    def handle(self, ev) -> str:
        """One event, the payload of one row if a row was clicked or hovered ('' otherwise)."""
        if not self.open:
            return ""
        if ev.type == pygame.MOUSEMOTION:
            self.hot = next((i for i in range(len(self.rows)) if self.row_rect(i).collidepoint(ev.pos)),
                            -1)
            return ""
        if ev.type != pygame.MOUSEBUTTONDOWN or ev.button != 1:
            return ""
        for i, row in enumerate(self.rows):
            if self.row_rect(i).collidepoint(ev.pos):
                return row[0]
        return ""

    def inside(self, pos) -> bool:
        """True while the mouse is on the panel — a drag there must not pan the map."""
        return bool(self.open and self.rect.collidepoint(pos))

    def width(self) -> int:
        """Panel width: wide enough for the longest row, so nothing is cut off at the edge.

        The panel used to be a fixed 210 px, which is less than its own footer line: the letters of
        all layers, printed below the rows, ran out of the surface and were simply gone — a clipped
        text nobody saw because the rest of the window looked fine.
        """
        widest = max([self.big.size(self.head)[0], self.font.size(self.foot)[0]]
                     + [self.font.size(label)[0] for _payload, label, _hint in self.rows])
        return max(WIDTH, widest + 2 * PAD + 34)          # 34: marker in front, hint behind

    def draw(self, screen, xy, state) -> None:
        if not self.open:
            return
        wide, hoch_screen = self.width(), screen.get_height()
        if self.at_pointer:
            # Under the pointer, but never over the edge of the window: a menu whose last row is off
            # screen is a robot that cannot be placed.
            self.rect.x = int(min(xy[0], screen.get_width() - wide - 4))
            self.rect.y = int(min(xy[1], hoch_screen - self.rect.height - 4))
        else:
            self.rect.x, self.rect.y = int(xy[0] - wide - 8), int(xy[1])
        self.rect.width = wide
        sheet = pygame.Surface((wide, self.rect.height), pygame.SRCALPHA)
        sheet.fill((14, 16, 22, 205))
        sheet.blit(self.big.render(self.head, True, (235, 235, 240)), (8, 5))
        for i, (payload, label, hint) in enumerate(self.rows):
            top = 24 + i * ROW
            if i == self.hot:
                pygame.draw.rect(sheet, (58, 64, 82, 235), pygame.Rect(1, top - 1, wide - 3, ROW))
            marker = pygame.Rect(7, top + 3, 13, 13)
            farbe = self.chips.get(payload)
            if farbe is None:
                # A switch: the checkbox says whether that layer is drawn right now.
                an = getattr(state, payload, False)
                pygame.draw.rect(sheet, (90, 200, 140, 255) if an else (70, 74, 88, 200), marker)
                color = (226, 228, 236) if an else (150, 154, 166)
            else:
                # Not a switch: the colour the robot is drawn in, so eight rows of names are eight rows
                # of the colours the map already shows. A row of a robot menu is not a checkbox — it
                # promises a tick that would never come.
                pygame.draw.rect(sheet, tuple(farbe) + (255,), marker)
                color = (226, 228, 236)
            sheet.blit(self.font.render(label, True, color), (marker.right + 7, top + 2))
            if hint:
                sheet.blit(self.font.render(hint, True, (126, 130, 144)), (wide - 20, top + 2))
        sheet.blit(self.font.render(self.foot, True, (150, 154, 168)), (8, self.rect.height - 19))
        pygame.draw.rect(sheet, (96, 102, 120, 255), sheet.get_rect(), 1)
        screen.blit(sheet, self.rect)
