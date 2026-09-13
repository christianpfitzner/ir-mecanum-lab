"""Small click menu for the view: which sensor layers are drawn — and which are not.

Only the drawing is switched off here. The simulator publishes odom, scan, gps, imu and the
estimate on the bus at full rate regardless of what this panel shows, so hiding the lidar
dots does not hide a single message on `/<robot>/scan`. That is the point of the panel: students
see which layer belongs to which topic and can look at the ROS side without a full screen.

The panel is translucent and sits in the top right corner, so the map underneath stays
readable; `m` hides the panel itself.
"""
import pygame

from . import keys

# attribute on the renderer -> (label in the panel, key that also toggles it) — `keys.LAYERS` is the
# table, so the key printed in this panel is the key the window listens for. That used to be two
# lists, and they had already drifted: the panel promised `w` for the wheels while `w` drove.
LAYERS = keys.menu_rows()
HEAD = "view layers"
FUSS = "m shows and hides · click a row"
WIDTH, ROW, PAD = 210, 19, 6           # WIDTH is the minimum: `width()` widens it for long rows


class Panel:
    """A column of checkmarks; `handle` turns a click into a layer name.

    Starts closed: a panel over the map would hide exactly the corner a goal is often placed
    in. `m` (or the HUD line that says so) opens it, clicking a row toggles a layer.
    """

    def __init__(self, font, big):
        self.font, self.big = font, big
        self.open, self.hot = False, -1
        self.rect = pygame.Rect(0, 0, WIDTH, ROW * len(LAYERS) + 52)

    def toggle(self) -> bool:
        self.open = not self.open
        return self.open

    def row_rect(self, index: int) -> pygame.Rect:
        """Hit area of one row, in screen coordinates."""
        return pygame.Rect(self.rect.x, self.rect.y + 24 + index * ROW, self.rect.width - 2, ROW)

    def handle(self, ev) -> str:
        """One event, one attribute name if a row was clicked or hovered ('' otherwise)."""
        if not self.open:
            return ""
        if ev.type == pygame.MOUSEMOTION:
            self.hot = next((i for i in range(len(LAYERS)) if self.row_rect(i).collidepoint(ev.pos)),
                            -1)
            return ""
        if ev.type != pygame.MOUSEBUTTONDOWN or ev.button != 1:
            return ""
        for i, (attribut, _text, _taste) in enumerate(LAYERS):
            if self.row_rect(i).collidepoint(ev.pos):
                return attribut
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
        widest = max([self.big.size(HEAD)[0], self.font.size(FUSS)[0]]
                     + [self.font.size(text)[0] for _attr, text, _key in LAYERS])
        return max(WIDTH, widest + 2 * PAD + 34)          # 34: checkbox in front, key hint behind

    def draw(self, screen, xy, state) -> None:
        if not self.open:
            return
        wide = self.width()
        self.rect.x, self.rect.y = int(xy[0] - wide - 8), int(xy[1])
        self.rect.width = wide
        sheet = pygame.Surface((wide, self.rect.height), pygame.SRCALPHA)
        sheet.fill((14, 16, 22, 205))
        sheet.blit(self.big.render(HEAD, True, (235, 235, 240)), (8, 5))
        for i, (attribut, text, taste) in enumerate(LAYERS):
            an, top = getattr(state, attribut, False), 24 + i * ROW
            if i == self.hot:
                pygame.draw.rect(sheet, (58, 64, 82, 235),
                                 pygame.Rect(1, top - 1, wide - 3, ROW))
            checkbox = pygame.Rect(7, top + 3, 13, 13)
            pygame.draw.rect(sheet, (90, 200, 140, 255) if an else (70, 74, 88, 200), checkbox)
            color = (226, 228, 236) if an else (150, 154, 166)
            sheet.blit(self.font.render(text, True, color), (checkbox.right + 7, top + 2))
            sheet.blit(self.font.render(taste, True, (126, 130, 144)), (wide - 20, top + 2))
        sheet.blit(self.font.render(FUSS, True, (150, 154, 168)), (8, self.rect.height - 19))
        pygame.draw.rect(sheet, (96, 102, 120, 255), sheet.get_rect(), 1)
        screen.blit(sheet, self.rect)
